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

Install once (in a terminal / command prompt):
    pip install bleak matplotlib
Run:
    python Wearable_Receiver.py
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
                 "accel_z_g", "loop_max_ms")

# ---------------------------------------------------------------------------
# Shared between the Bluetooth thread and the plot (protected by `lock`)
# ---------------------------------------------------------------------------
lock = threading.Lock()
ppg_wave = collections.deque([0] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
ecg_wave = collections.deque([0] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
touch_flags = collections.deque([False] * (WINDOW_S * POINTS_PER_S), maxlen=WINDOW_S * POINTS_PER_S)
status_text = ["Searching for '{}'...".format(DEVICE_NAME)]
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
log.writerow(("time",) + STATUS_FIELDS + ("event", "event_length_s"))


def log_row(values, event="", event_length=""):
    now = datetime.datetime.now().isoformat(timespec="seconds")
    log.writerow([now] + values + [event, event_length])
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
    accel = "Accel X {} Y {} Z {} g".format(
        get("accel_x_g"), get("accel_y_g"), get("accel_z_g"))
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
        print(text)
        set_status_text(text)
        log_row(parts[1:])
    elif kind == "E" and len(parts) == 2:
        set_event(parts[1] == "TOUCHED")


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
def main():
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
    shading = []                                # patient-event shading, redrawn each frame

    def update(_frame):
        with lock:
            ppg_line.set_ydata(list(ppg_wave))
            ecg_line.set_ydata(list(ecg_wave))
            flags = list(touch_flags)
            active = event["active"]
            text = status_text[0]
        title.set_text(("PATIENT EVENT\n" if active else "") + text)
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
        return ppg_line, ecg_line, title

    # keep a reference to the animation, or it stops updating
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()                                  # returns when the window closes
    log_file.close()
    print("Stopped. Log saved to", log_name)


if __name__ == "__main__":
    main()
