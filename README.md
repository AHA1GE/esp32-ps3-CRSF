# ESP32-PS3-CRSF
This project allow you use a PS3 controller with a ESP32 to send CRSF commands to a Crossfire transmit module.
PS3 part based on [Ps3Controller library](https://github.com/jvpernis/esp32-ps3) by jeffrey van Pernis.
CRSF part based on kkbin505's [Arduino-Transmitter-for-ELRS](https://github.com/kkbin505/Arduino-Transmitter-for-ELRS) project.

## Features
- Pair PS3 controller with ESP32
- Send CRSF commands to Crossfire module

## Usage
- Download and install the [Arduino IDE](https://www.arduino.cc/en/software)
- Install the ESP32 board support by following the instructions [here](https://docs.espressif.com/projects/esp-faq/en/latest/development-environment/IDE-plugins.html#how-to-add-esp32-development-board-on-arduino-ide).
- Install the [Ps3Controller library](https://github.com/jvpernis/esp32-ps3) by jeffrey van Pernis.
- Download this repository and open the `ESP32-PS3-CRSF.ino` file in the Arduino IDE.
- Connect your ESP32 to your computer and select the correct port and board in the Arduino IDE.
- Upload the sketch to your ESP32.
- Connect/solder the CRSF pin to the TX1 pin(defined in source code) of the ESP32.
- If necessary, connect/solder power and ground pins
- Power on the ESP32 and Crossfire module
