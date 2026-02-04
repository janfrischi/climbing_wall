# Climbing Wall Force Validation & Calibration

Small utilities to calibrate and validate 3-axis force/strain sensors using Phidget VoltageRatioInput devices.

Prerequisites
- Python 3.8+
- Phidget22 Python package (native library required for your OS)
- numpy, matplotlib

Quick start
1. Edit sensor configuration (serial numbers / channels) in:
   - [`validation.py`](validation.py)
   - [`calibration.py`](calibration.py)
2. Run calibration to compute gain/offsets:
   - python3 calibration.py
3. Run validation to tare and read weights:
   - python3 validation.py

Note: The [`demonstration.py`](demonstration.py) script is intended to run on the Raspberry Pi (ESP32 host in this workspace). It reads Phidget sensor data on the device, streams CSV-style force frames over a serial port, and the frontend consumes/visualizes that stream. See [`demonstration.main`](demonstration.py), [`demonstration.get_hand_foot_forces`](demonstration.py) and [`demonstration.get_total_weight_full_board`](demonstration.py) for the streaming/collection logic.

Key scripts & functions
- [`validation.tare_scale`](validation.py): collect zero-offsets.
- [`validation.get_total_weight`](validation.py): continuous measurement loop computing total weight.
- [`validation.main`](validation.py): device setup, tare, and start validation.
- [`calibration.get_voltage`](calibration.py): read mean voltages for calibration.
- [`calibration.plot_calibration_curve`](calibration.py): visualize fitted line and data.

Notes
- Weight per axis is computed as $W_x=(V_x-V_{x0})\cdot gain_x$ (same for $W_y,W_z$).
- Total weight magnitude is computed as $W=\sqrt{W_x^2+W_y^2+W_z^2}$.
- The virtual environment used in development is in `.climb/` — install packages there or in your environment.

Files
- [validation.py](validation.py) — validation/tare/measurement loop
- [calibration.py](calibration.py) — calibration and plotting