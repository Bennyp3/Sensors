# Wearable Sensor Tracker (ELEE 4230/6230, Fall 2026)

MicroPython on a Raspberry Pi Pico 2 W. One student (Ben) building a wearable
tracker for the Sensors and Transducers course. The code is graded and explained
in a report, so prefer small, readable, well-commented changes over clever ones.

## Goal of this work

Merge four working standalone scripts into **one program** that runs all sensors
together on the Pico:

| Script | What it does |
|---|---|
| `Analog_MUX.py` | PPG heart rate + thermistor temperature through one ADC and an analog mux, BMP280 reference, Kalman filters |
| `AD8232.py` | ECG heart rate and electrode-off detection |
| `Accel_BMP280.py` | LSM6DSO accelerometer/gyro + BMP280 on a shared I2C bus |
| `Cap_Press.py` | Capacitive press detector using a PIO state machine and a comparator |

**Approach: build up one sensor at a time.** Start from the working mux program
(`Analog_MUX.py`: PPG + thermistor + BMP280). Add one sensor, have Ben flash it and
confirm it works on the hardware, then add the next. Suggested order: LSM6DSO
(shares the BMP280's I2C bus), then ECG, then the press sensor (the hardest,
because it blocks; see the merge problems below). Do not add the next sensor
until Ben reports the current step works.

**Rules for the merge**

- Keep the four standalone scripts unchanged as references. Write the merged
  program as a new file.
- Do not change calibration constants or filter tuning unless asked.
- Ben flashes and tests on the Pico himself (Thonny). Claude Code cannot run the
  hardware, so explain what to test and what output to expect.
- `bme280.py` and `lsm6dsox.py` (MicroPython drivers) are imported by the scripts
  but are not included with them. Ask Ben to copy them off the Pico.

## Pin map (all in use at once)

| Pico pin | Function | Source script |
|---|---|---|
| GP0 (SDA), GP1 (SCL) | I2C0, **shared by BMP280 and LSM6DSO** | Accel_BMP280.py, Analog_MUX.py |
| GP14 (pin 19) | AD8232 LO+ (leads-off, input) | AD8232.py |
| GP15 (pin 20) | AD8232 LO- (leads-off, input) | AD8232.py |
| GP17 (pin 22) | Mux select, via R1 into Q1 base. **HIGH = PPG, LOW = thermistor** | Analog_MUX.py |
| GP20 | Press sensor drive: square wave out through R2 to the sensor | Cap_Press.py |
| GP21 | Press sensor sense: comparator output in | Cap_Press.py |
| GP26 (ADC0, pin 31) | Mux output (PPG or thermistor), through a buffer | Analog_MUX.py |
| GP27 (ADC1, pin 32) | AD8232 OUTPUT (ECG) | AD8232.py |

Free ADC pin: GP28 (ADC2). PIO state machine 0 on PIO0 is used by the press sensor.
All ADC reads use `adc.read_u16() >> 4` (12-bit value, 0-4095), 3.3 V reference.
**Pico I2C and ADC pins are not 5 V tolerant.**

## I2C bus (important)

The LSM6DSO accelerometer and the BMP280 share **one SDA line (GP0) and one SCL
line (GP1)**. They have different addresses, so one `I2C(0, sda=Pin(0), scl=Pin(1))`
object is created once and passed to both drivers. Never create two I2C objects
on the same bus.

| Device | Address | Strap pins |
|---|---|---|
| BMP280 | 0x76 | CSB to 3.3 V (selects I2C), SDO to GND |
| LSM6DSO | 0x6A | CS to 3.3 V (selects I2C), SDO to GND |

- Both breakout boards wire SDA to the module's SDA/SDI pin and SCL to SCL/SCK.
  SDO goes to GND (address select), not to the data line.
- `Analog_MUX.py` uses `freq=400000`; `Accel_BMP280.py` uses `freq=100000`.
  The merged program needs one speed. 100 kHz is the known-good choice; try
  400 kHz only if loop timing needs it and both sensors still read correctly.
- Drivers: `bme280.BME280(i2c=i2c)` (used for the BMP280) and
  `lsm6dsox.LSM6DSOX(i2c)`. API as used in the scripts:
  - `bmp.read_compensated_data()` returns `(temp, pressure, humidity)`.
    Temperature in C is `temp / 100`; pressure in hPa is `pressure / 25600`.
    Humidity is meaningless on a BMP280.
  - `imu.accel()` and `imu.gyro()` each return `(x, y, z)`. Units come from the
    driver; check them before labelling output.

## Sensors and signal chains

### PPG (heart rate)
BPW34 photodiode with a TSAL6100 IR LED (about 20 mA). Raw analog signal goes
through the mux to GP26. Sampled at 250 Hz (`SAMPLE_MS = 4`), smoothed with a
4-sample moving average, then beat detection (see below). BPM is the median of
beat intervals over the last 6 s, then passed through a Kalman filter.

### Thermistor (skin temperature)
Wheatstone bridge, INA121 (gain 3.547, RG = 19.63 kohm), REF about 1.5 V from a
buffered divider. The signal goes through the mux to GP26, read once per second.
An earlier problem (readings scattering by about +/-8 C) was a **hardware fix**,
not a code change, so the settle and averaging values in `Analog_MUX.py`
(`SETTLE_MS = 1`, `N_AVG = 8`) are correct as written.
Linear calibration: 0.000 V = 30 C and 3.193 V = 10 C (about -6.26 C per V),
plus `CAL_OFFSET_C = 1.8`. The BMP280 is the independent reference
(`Diff = Therm KF - BMP280`).

### ECG
AD8232 output to GP27. 250 Hz. Same envelope-threshold beat detector as the PPG.
`LO+` and `LO-` read high when an electrode is off; the sample is treated as
invalid (`None`) and a warning prints at most once per second.

### Accelerometer
LSM6DSO on the shared I2C bus (accel and gyro).

### Capacitive press sensor
A square wave from GP20 charges the sensor capacitor (about 112 pF) through
R2 = 1.421 Mohm. An LM339 comparator (reference 1.65 V) output goes to GP21.
A PIO state machine counts how long the comparator takes to go high (td). A
touch raises the capacitance, so td goes up. Touched = td above
`TOUCH_ABOVE_US = 30.0` for 5 cycles in a row (`DEBOUNCE`).

## Beat detection (duplicated in Analog_MUX.py and AD8232.py)

The same algorithm, written twice. Merge it into one reusable function or class:

```
high = max(sample, high - 1);  low = min(sample, low + 1)
threshold = low + 0.6 * (high - low)
rising crossing of threshold, with (high - low) > 100  -> beat
ignore beats closer than 300 ms; BPM only from intervals of 300-2000 ms
```

Also share one `KalmanFilter` class. Tuning in the scripts:
temperature `(est=25.0, E_EST=1.0, E_MEA=0.25, Q=0.01)`,
BPM `(est=None, E_EST=10.0, E_MEA=5.0, Q=1.0)`.

## Design constraints in the existing code (keep these)

- **The mux is switched in software, not with 1 kHz PWM.** MicroPython cannot sync
  an ADC read to PWM edges, so a free-running PWM mixes channels. GP17 stays HIGH
  (PPG) and drops LOW briefly once per second to read the thermistor, then
  returns HIGH. Each switch needs a settling delay, and PPG beat detection is
  blanked for 200 ms afterward (`BLANK_MS`).
- **Timing uses `time.ticks_ms()` and `ticks_diff`**, a 4 ms schedule that
  resyncs if it falls behind. Keep that pattern.
- On exit, GP17 is left HIGH (PPG channel) in a `finally` block. Keep it.
- Debug switches exist (`TEMP_ENABLED`, `SHOW_WAVE`, `PLOT_EVERY`) for the Thonny
  plotter. Keep equivalent switches in the merged program.

## Known merge problems to solve (decisions not made yet)

1. **The press sensor blocks.** `measure_td_us()` sleeps `2 * HALF_US` = about 2 ms
   per call (`HALF_US = 1000`). Calling it every 4 ms tick leaves little time for
   anything else. Options: call it only every Nth tick, or move both halves of
   the square wave into the PIO program so the CPU just reads the FIFO.
2. **Loop budget.** Each 4 ms tick would read PPG and ECG, and once per second
   the thermistor (blocking), BMP280 and LSM6DSO over I2C. Measure the worst-case
   loop time (print it during testing) before assuming it fits.
3. **Two ADC channels in one loop.** PPG (GP26) and ECG (GP27) are both sampled
   each tick on the same ADC. Read each with its own `ADC(Pin(...))` object.
4. **Press-sensor docstring disagrees with its constants.** The top of
   `Cap_Press.py` says td is about 110 us untouched, 133 us light, 220 us hard
   (that is 0.693 x R x C for 1.421 Mohm and 112 pF). The later comment and the
   threshold say about 14 us untouched, 40-67 us pressed, threshold 30 us. The code
   uses the constants, which were presumably measured. Ask Ben before changing.

## Things to confirm with Ben

- The wiring comment in `AD8232.py` says 3.3 V from the Pico's 3V3 OUT pin
  (pin 36). That comment is out of date; see the power section below.
- `CAL_OFFSET_C = 1.8` was set against the BMP280 on the old supply. Re-check it.

## Power and course constraints

- Standalone and battery powered. Two 9 V batteries in series (~18 V) feed a
  regulator, then a hand-built op-amp plus BC547/BC557 virtual-ground buffer makes
  +4.5 V / 0 V / -4.5 V. Pico GND is the virtual ground (0 V).
- The Pico runs from the +4.5 V rail into VSYS through one diode. VBUS stays
  unconnected. USB and the diode OR together.
- **All sensors are powered from Ben's own 3.3 V rail** (MCP1700-3302E LDO), not
  from the Pico. That includes the AD8232 (and its SDN pin), the BMP280, the
  LSM6DSO, and the press sensor's LM339 comparator and 10k pull-up. Comments in
  the old scripts that say "3V3 OUT (pin 36)" are out of date. All grounds are
  common (the virtual ground).
- **Constraints:** Ben must build the +/-4.5 V supply himself (no ready-made split
  supply), and sensors may **not** be powered from the Pico's own 3.3 V pin. The
  LM324 is on the 0/+4.5 V rails driving the ADC buffer; do not reuse it for the
  virtual-ground buffer.
- The supply is **still being debugged** (rails sag under load; parts failed). Do
  not treat odd sensor readings as software bugs until the rails are confirmed.
- Optional: `Pin('WL_GPIO1', Pin.OUT).value(1)` forces the Pico's regulator into
  PWM mode, which cuts supply ripple and may reduce ADC noise on battery power.
- Networking is out of scope for now.

## Running on the Pico

Saving the final program on the Pico as `main.py` makes it run automatically at
every power-up. While developing, run it from Thonny so errors show up.

## Open items

1. Merged program (this work).
2. Confirm `CAL_OFFSET_C` against the BMP280 on the new supply.
3. Finish the +/-4.5 V supply debug.
4. Fill in the yellow cells in `Sensors_Check_List_Ben_Parks.xlsx` (costs,
   inspections, calibration).
