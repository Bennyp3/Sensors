"""
Wearable_Receiver.py -- runs on the COMPUTER, not the Pico.

Connects to the Pico ("Wearable") over Bluetooth Low Energy and:
    * draws the live PPG and ECG waves in a window (last 10 s)
    * prints the 2-second status line in this terminal
    * logs every status line and press event to a CSV file
    * treats the capacitive press sensor as a "patient event" button, like
      the event button on a Holter / cardiac event monitor: while it is
      touched the plot background is shaded, and the start and end of each
      event (with its length) are written to the CSV log
    * runs the GO-BEYOND algorithm: from the activity level (accelerometer),
      the PPG Kalman heart rate and the skin temperature (thermistor Kalman),
      compared with published standards for the wearer's age, it gives a
      health zone every 2 s: GREEN (healthy) / YELLOW (caution) / RED (alert)

Install once (in a terminal / command prompt):
    pip install bleak matplotlib
Run:
    python Wearable_Receiver.py
    (it asks for the wearer's age first)
Close the plot window (or press Ctrl+C) to stop. Bluetooth must be on, and
the Pico does NOT need to be paired in Windows settings.

Messages from the Pico (see Wearable_Main.py):
    W,<ppg>,<ecg>            wave point, 25 per second
    S,<fields...>            status every 2 s, in STATUS_FIELDS order
    E,TOUCHED / E,RELEASED   press sensor event
"""

import asyncio
import collections
import csv
import datetime
import threading

from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DEVICE_NAME = "Wearable"                                  # BLE_NAME on the Pico
UART_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"          # Pico -> computer

POINTS_PER_S = 25        # wave points per second (Pico: 250 Hz / PLOT_EVERY)
WINDOW_S = 10            # seconds of wave shown in the plot

# must match STATUS_FIELDS in Wearable_Main.py
STATUS_FIELDS = ("ecg_bpm", "ecg_off", "ppg_bpm_raw", "ppg_bpm_kf",
                 "therm_raw_c", "therm_kf_c", "bmp_c", "diff_c",
                 "touched", "td_avg_us", "accel_x_g", "accel_y_g",
                 "accel_z_g", "loop_max_ms", "enmo_mg")

# ---------------------------------------------------------------------------
# GO-BEYOND standards (the full references are listed in the report)
# ---------------------------------------------------------------------------
# Activity level from wrist ENMO, averaged over the last few status lines.
# Hildebrand et al. 2014, Med Sci Sports Exerc 46(9):1816-1824, wrist,
# adults: moderate-to-vigorous activity starts at about 100.6 mg; below
# about 44.8 mg counts as inactive (sedentary).
INACTIVE_BELOW_MG = 44.8
ACTIVE_ABOVE_MG   = 100.6
ACTIVITY_REPORTS  = 3       # average ENMO over the last 3 status lines (6 s)

# Heart rate. Maximum HR: Tanaka, Monahan & Seals 2001, J Am Coll Cardiol
# 37(1):153-156:  HRmax = 208 - 0.7 x age.
# Inactive: normal resting HR 60-100 BPM (American Heart Association).
# Light: up to the top of the ACSM "light" band, 63 % of HRmax.
# Active: the CDC / ACSM target zone for moderate-to-vigorous exercise,
#         64-93 % of HRmax.
REST_HR_LOW, REST_HR_HIGH = 60, 100
LIGHT_MAX_FRAC = 0.63
ACTIVE_MIN_FRAC, ACTIVE_MAX_FRAC = 0.64, 0.93

# Skin temperature: normal skin temperature at rest in comfortable
# conditions is about 32-35 C (mean skin temperature about 33-34 C, the
# thermal comfort range used by Fanger / ISO 7730). Above about 35 C the
# skin-to-core gradient shrinks and heat strain rises (Sawka et al. 2012,
# Exp Physiol 97(3):327-332).
SKIN_LOW_C, SKIN_HIGH_C = 32.0, 35.0

GO_BEYOND_FIELDS = ("age", "activity", "enmo_avg_mg", "hr_used_bpm",
                    "hr_low_bpm", "hr_high_bpm", "hr_status", "skin_status",
                    "zone", "reason")

# ---------------------------------------------------------------------------
# Shared between the Bluetooth thread and the plot (protected by `lock`)
# ---------------------------------------------------------------------------
lock = threading.Lock()
ppg_wave = collections.deque([0] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
ecg_wave = collections.deque([0] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
touch_flags = collections.deque([False] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
status_text = ["Searching for '{}'...".format(DEVICE_NAME)]
zone_info = {"zone": "", "text": "Health zone: waiting for data"}
enmo_history = collections.deque(maxlen=ACTIVITY_REPORTS)
age = [None]                                 # entered at start-up
event = {"active": False, "start": None}     # current patient event


def set_status_text(text):
    with lock:
        status_text[0] = text


# ---------------------------------------------------------------------------
# CSV log: one row per status line or press event
# ---------------------------------------------------------------------------
log_name = "wearable_log_{}.csv".format(datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
log_file = open(log_name, "w", newline="")
log = csv.writer(log_file)
log.writerow(("time",) + STATUS_FIELDS + GO_BEYOND_FIELDS + ("event", "event_length_s"))


def log_row(values, go_beyond=None, event="", event_length=""):
    now = datetime.datetime.now().isoformat(timespec="seconds")
    if go_beyond is None:
        go_beyond = [""] * len(GO_BEYOND_FIELDS)
    log.writerow([now] + values + go_beyond + [event, event_length])
    log_file.flush()                 # keep the file complete if we stop suddenly


# ---------------------------------------------------------------------------
# Turning messages into readable text
# ---------------------------------------------------------------------------
def format_status(v):
    """Same layout as the Pico's USB output, on one line."""
    def get(name, unit=""):
        return v[name] + unit if v[name] else "--"

    ecg = "ECG off: " + v["ecg_off"] if v["ecg_off"] else "ECG " + get("ecg_bpm") + " BPM"
    ppg = "PPG {}/{} BPM".format(get("ppg_bpm_raw"), get("ppg_bpm_kf"))
    temp = "Therm {} | KF {} | BMP {} | Diff {}".format(
        get("therm_raw_c", " C"), get("therm_kf_c", " C"),
        get("bmp_c", " C"), get("diff_c", " C"))
    press = "Press {} (td {})".format(
        "TOUCHED" if v["touched"] == "1" else "untouched", get("td_avg_us", " us"))
    accel = "Accel X {} Y {} Z {} g, ENMO {}".format(
        get("accel_x_g"), get("accel_y_g"), get("accel_z_g"), get("enmo_mg", " mg"))
    return " | ".join((ecg, ppg, temp)) + "\n    " + " | ".join(
        (press, accel, "Loop max " + get("loop_max_ms", " ms")))


def handle_line(line):
    parts = line.split(",")
    kind = parts[0]
    if kind == "W" and len(parts) == 3:
        try:
            ppg, ecg = int(parts[1]), int(parts[2])
        except ValueError:
            return
        with lock:
            ppg_wave.append(ppg)
            ecg_wave.append(ecg)
            touch_flags.append(event["active"])   # shade this point if touched
    elif kind == "S" and len(parts) == 1 + len(STATUS_FIELDS):
        values = dict(zip(STATUS_FIELDS, parts[1:]))
        # the status line also carries the touch state, so a missed E message
        # (or starting the receiver mid-press) still gets the shading right
        set_event(values["touched"] == "1")
        text = format_status(values)
        result = go_beyond(values)
        print(text)
        print("    " + result["text"])
        set_status_text(text)
        with lock:
            zone_info["zone"] = result["zone"]
            zone_info["text"] = result["text"]
        log_row(parts[1:], [str(result[name]) for name in GO_BEYOND_FIELDS])
    elif kind == "E" and len(parts) == 2:
        set_event(parts[1] == "TOUCHED")


def number(text):
    """Status field -> float, or None if the field is empty."""
    try:
        return float(text)
    except ValueError:
        return None


def go_beyond(values):
    """GO-BEYOND algorithm: health zone from activity + heart rate + skin temp.

    1. Activity level from the average ENMO (accelerometer).
    2. Expected heart-rate range for that activity level and the wearer's age.
    3. Is the PPG Kalman heart rate inside, below or above that range?
    4. Is the skin temperature (thermistor Kalman) inside its normal band?
    5. Zone: GREEN = both normal, YELLOW = one out of range,
             RED = both out of range, or a resting heart rate above 100 BPM
             while inactive (resting tachycardia).
    """
    r = dict.fromkeys(GO_BEYOND_FIELDS, "")
    r["age"] = age[0]

    # 1. activity level
    enmo = number(values["enmo_mg"])
    if enmo is not None:
        enmo_history.append(enmo)
    if not enmo_history:
        r["zone"] = "NO DATA"
        r["text"] = "Health zone: NO DATA (no accelerometer reading)"
        return r
    enmo_avg = sum(enmo_history) / len(enmo_history)
    if enmo_avg < INACTIVE_BELOW_MG:
        activity = "inactive"
    elif enmo_avg < ACTIVE_ABOVE_MG:
        activity = "light"
    else:
        activity = "active"
    r["activity"] = activity
    r["enmo_avg_mg"] = round(enmo_avg, 1)

    # 2. expected heart rate for this activity level and age
    hr_max = 208 - 0.7 * age[0]
    if activity == "inactive":
        hr_low, hr_high = REST_HR_LOW, REST_HR_HIGH
    elif activity == "light":
        hr_low, hr_high = REST_HR_LOW, LIGHT_MAX_FRAC * hr_max
    else:
        hr_low, hr_high = ACTIVE_MIN_FRAC * hr_max, ACTIVE_MAX_FRAC * hr_max
    r["hr_low_bpm"] = round(hr_low)
    r["hr_high_bpm"] = round(hr_high)

    # 3. heart rate (PPG Kalman) against the expected range
    hr = number(values["ppg_bpm_kf"])
    skin = number(values["therm_kf_c"])
    if hr is None or skin is None:
        r["zone"] = "NO DATA"
        r["text"] = "Health zone: NO DATA ({}: {})".format(
            activity, "no PPG heart rate" if hr is None else "no skin temperature")
        return r
    r["hr_used_bpm"] = round(hr)
    if hr < hr_low:
        r["hr_status"] = "low"
    elif hr > hr_high:
        r["hr_status"] = "high"
    else:
        r["hr_status"] = "normal"

    # 4. skin temperature against its normal band
    if skin < SKIN_LOW_C:
        r["skin_status"] = "cool"
    elif skin > SKIN_HIGH_C:
        r["skin_status"] = "hot"
    else:
        r["skin_status"] = "normal"

    # 5. combine into one zone
    problems = []
    if r["hr_status"] != "normal":
        problems.append("HR {} for {} ({:.0f} BPM, expected {:.0f}-{:.0f})".format(
            r["hr_status"], activity, hr, hr_low, hr_high))
    if r["skin_status"] != "normal":
        problems.append("skin {} ({:.1f} C, normal {:.0f}-{:.0f})".format(
            r["skin_status"], skin, SKIN_LOW_C, SKIN_HIGH_C))
    if activity == "inactive" and hr > REST_HR_HIGH:
        r["zone"] = "RED"                 # resting tachycardia
    elif len(problems) == 2:
        r["zone"] = "RED"
    elif len(problems) == 1:
        r["zone"] = "YELLOW"
    else:
        r["zone"] = "GREEN"
    r["reason"] = "; ".join(problems) if problems else "all within standards"
    label = {"GREEN": "HEALTHY", "YELLOW": "CAUTION", "RED": "ALERT"}[r["zone"]]
    r["text"] = "Health zone: {} {} | {} (ENMO {:.0f} mg) | HR {:.0f} BPM | skin {:.1f} C | {}".format(
        r["zone"], label, activity, enmo_avg, hr, skin, r["reason"])
    return r


def set_event(active):
    """Start or end a patient event; log it to the CSV when it changes."""
    if active == event["active"]:
        return
    now = datetime.datetime.now()
    with lock:
        event["active"] = active
        start = event["start"]
        event["start"] = now if active else None
    if active:
        print(">>> PATIENT EVENT START")
        log_row([""] * len(STATUS_FIELDS), event="EVENT_START")
    else:
        length = (now - start).total_seconds()
        print(">>> PATIENT EVENT END ({:.1f} s)".format(length))
        log_row([""] * len(STATUS_FIELDS), event="EVENT_END",
                event_length=round(length, 1))


# ---------------------------------------------------------------------------
# Bluetooth: find the Pico, connect, receive, reconnect if the link drops
# ---------------------------------------------------------------------------
async def ble_main():
    buffer = bytearray()

    def on_notify(_sender, data):
        # packets can split a line, so collect bytes and cut at each newline
        buffer.extend(data)
        while b"\n" in buffer:
            end = buffer.index(b"\n")
            line = buffer[:end].decode(errors="replace").strip()
            del buffer[:end + 1]
            if line:
                handle_line(line)

    while True:
        print("Searching for '{}'...".format(DEVICE_NAME))
        set_status_text("Searching for '{}'...".format(DEVICE_NAME))
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
        if device is None:
            continue
        try:
            async with BleakClient(device) as client:
                buffer.clear()
                await client.start_notify(UART_TX, on_notify)
                print("Connected to {} ({})".format(DEVICE_NAME, device.address))
                set_status_text("Connected - waiting for the first status line...")
                while client.is_connected:
                    await asyncio.sleep(0.5)
        except Exception as e:                  # link dropped, out of range...
            print("Bluetooth error:", e)
        print("Disconnected - will reconnect.")


def ble_thread():
    asyncio.run(ble_main())


# ---------------------------------------------------------------------------
# Live plot (main thread)
# ---------------------------------------------------------------------------
def ask_age():
    """Ask for the wearer's age (the heart-rate standards depend on it)."""
    while True:
        text = input("Wearer's age in years: ").strip()
        try:
            years = int(text)
        except ValueError:
            print("Please type a whole number, e.g. 21")
            continue
        if 1 <= years <= 120:
            if years < 18:
                print("Note: the heart-rate standards used are for adults.")
            return years
        print("Please type an age between 1 and 120")


def main():
    age[0] = ask_age()
    hr_max = 208 - 0.7 * age[0]
    print("Age {}: HRmax = 208 - 0.7 x {} = {:.0f} BPM (Tanaka 2001)".format(
        age[0], age[0], hr_max))
    print("Logging to", log_name)
    threading.Thread(target=ble_thread, daemon=True).start()

    fig, (ax_ppg, ax_ecg) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    t = [i / POINTS_PER_S - WINDOW_S for i in range(WINDOW_S * POINTS_PER_S)]
    (ppg_line,) = ax_ppg.plot(t, list(ppg_wave))
    (ecg_line,) = ax_ecg.plot(t, list(ecg_wave), color="tab:red")
    ax_ppg.set_ylabel("PPG (counts)")
    ax_ecg.set_ylabel("ECG (counts)")
    ax_ecg.set_xlabel("time (s)")
    title = fig.suptitle("", fontsize=9, family="monospace")
    zone_colors = {"GREEN": "#b7e4b0", "YELLOW": "#ffe08a", "RED": "#f4a6a6"}
    zone_box = fig.text(0.5, 0.01, "", ha="center", va="bottom", fontsize=10,
                        family="monospace", bbox=dict(boxstyle="round", fc="white"))
    fig.subplots_adjust(bottom=0.16)            # room for the zone banner
    shading = []                                # patient-event shading, redrawn each frame

    def update(_frame):
        with lock:
            ppg_line.set_ydata(list(ppg_wave))
            ecg_line.set_ydata(list(ecg_wave))
            flags = list(touch_flags)
            active = event["active"]
            text = status_text[0]
            zone, zone_text = zone_info["zone"], zone_info["text"]
        title.set_text(("PATIENT EVENT\n" if active else "") + text)
        zone_box.set_text(zone_text)
        zone_box.get_bbox_patch().set_facecolor(zone_colors.get(zone, "white"))
        title.set_color("tab:red" if active else "black")

        # shade the background wherever the press sensor was touched; the
        # shading scrolls left with the wave because it is stored per point
        for patch in shading:
            patch.remove()
        shading.clear()
        for ax in (ax_ppg, ax_ecg):
            shading.append(ax.fill_between(
                t, 0, 1, where=flags, step="mid", color="gold", alpha=0.35,
                transform=ax.get_xaxis_transform(), linewidth=0))
            ax.relim()                          # rescale y to the visible wave
            ax.autoscale_view(scalex=False)
        return ppg_line, ecg_line, title, zone_box

    # keep a reference to the animation, or it stops updating
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()                                  # returns when the window closes
    log_file.close()
    print("Stopped. Log saved to", log_name)


if __name__ == "__main__":
    main()
