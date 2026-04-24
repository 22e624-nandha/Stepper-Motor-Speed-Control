# Stepper Motor Speed Control

A stepper motor control project with an ESP32-based Bluetooth command/Graphical interface and a Python open-loop step-loss simulation.

## Project Structure

- `stepper_speed_control_app/stepper_speed_control_app.ino`
  - ESP32 Arduino sketch for controlling a stepper motor driver using `STEP_PIN` and `DIR_PIN`.
  - Accepts Bluetooth commands over `BluetoothSerial` to start/stop the motor, change direction, set frequency, and run a step-angle sequence with acceleration ramp.
- `stepper_simulation.py`
  - Python Tkinter-based simulation that demonstrates open-loop stepper motor behavior, load torque, and step loss under load.

## Features

- Bluetooth command control via ESP32
- Continuous motor speed control (frequency-based)
- Step-angle mode with soft acceleration ramp
- Direction control forward/reverse
- Status reporting and progress updates over Bluetooth
- Open-loop simulation of step-loss and load effects in Python

## Hardware Used

- ESP32 development board
- 57CM23-3A Leadshine stepper motor
- DM542E microstep driver
- 29V 2A AC-to-DC converter adapter
- 10mm and 20mm radius 3D-printed pulley
- Android mobile device for Bluetooth control

## Software Used

- Arduino IDE
- PlatformIO IDE extension for VS Code
- Flutter for the Android Bluetooth control app
- Android Studio for app development
- Python 3 with Tkinter for the stepper motor open-loop simulation

## Android App Features

All control features are implemented in the Android app itself.

- Speed control
- Direction control
- On/off motor control
- Step-angle-specific motor control
- RPM-based speed control
- Number of rotations setting (e.g. set 2 rotations and the motor moves 2 rotations then stops automatically)
- Technical guide section describing FOC idea and concept
- AI technical advice using Gemini 2.5 Flash Lite API integration

## ESP32 Command Protocol

The ESP32 sketch listens for newline-terminated commands over Bluetooth and parses the following commands:

- `F` — Set direction forward
- `R` — Set direction reverse
- `G` — Start motor output
- `S` — Stop motor output
- `FRQ:<frequency>` — Set continuous step frequency in Hz
- `STEP:<angle>,<rotations>,<ppr>,<speed_hz>` — Run step-angle mode
- `STEP_STOP` — Emergency stop step-angle mode
- `STATUS` — Request status report

### `STEP:` command parameters

- `<angle>` — Step angle in degrees for each rotation chunk
- `<rotations>` — Number of rotations to execute
- `<ppr>` — Pulses per revolution of the stepper driver
- `<speed_hz>` — Target pulse frequency in Hz

Example:

- `FRQ:100` sets the motor output frequency to 100 Hz
- `STEP:180,2,200,250` commands a 180° move for 2 rotations with 200 pulses/revolution at 250 Hz

## Running the ESP32 Sketch

1. Open `stepper_speed_control_app/stepper_speed_control_app.ino` in Arduino IDE or PlatformIO.
2. Configure board settings for your ESP32 model.
3. Upload the sketch to the ESP32.
4. Connect to Bluetooth device name `Stepper_Control` from your phone or PC.
5. Send commands as plain text terminated with newline.

## Using the Simulation

1. Make sure Python 3 is installed.
2. Run the simulation with:

```bash
python stepper_simulation.py
```

3. The simulation window lets you adjust RPM, load, and commanded steps.
4. It visualizes expected vs actual motion, step loss, and load-related behavior.

## Notes

- The ESP32 sketch uses `tone()` on `STEP_PIN` to generate pulse trains. This is suitable for open-loop stepper control but does not implement full closed-loop feedback.
- Step-angle mode gradually ramps from a low startup frequency to the target frequency to avoid missed steps on startup.
- The simulation is intended for demonstration and conceptual understanding of step loss under load.

## Author

- NANDHAKUMAR J
- 22E624
- PSGTECH BE EEE SW STUDENT
