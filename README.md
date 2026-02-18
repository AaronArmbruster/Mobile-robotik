# Mobile Robotik
Dieses Repository enthält das Software-Framework für die Lehrveranstaltung [Mobile Robotik](https://online.tugraz.at/tug_online/ee/rest/pages/slc.tm.cp/course/575774).

Repository klonen
```
git clone --recurse-submodules https://git.sai.tugraz.at/courses/mobile-robotik.git mobile_robotik
```

## Setup
Dieser Abschnitt erklärt, wie dieses Framework eingerichtet wird.

### Dynamixel SDK
Um mit den Motoren des Turtlebots zu interagieren, musst du das [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) installieren.

```
cd external/dynamixel_sdk/c++/build/linux_sbc/
sudo make install
```

**Wenn du das SDK auf deinem eigenen Rechner statt auf dem Turtlebot installieren möchtest, verwende das Build-Skript, das zur Architektur deines Systems passt (z.B. `linux64`)!**

### OpenCR
Installiere die Firmware für das [OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/), welche von diesem Framework benötigt wird, mithilfe des [Arduino CLI](https://docs.arduino.cc/arduino-cli/).

Richte die OpenCR-Unterstützung ein und installiere die Abhängigkeiten
```
arduino-cli config add board_manager.additional_urls "file://$(pwd)/external/opencr/arduino/opencr_release/package_opencr_index.json"
arduino-cli core install OpenCR:OpenCR
```

Firmware kompilieren und auf das OpenCR flashen
Dazu muss das OpenCR per USB mit deinem PC verbunden sein.
```
cd external/opencr/arduino/opencr_arduino/opencr/libraries/OpenCR/examples/10.\ Etc/usb_to_dxl/
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR --build-path build
arduino-cli upload --fqbn OpenCR:OpenCR:OpenCR --build-path build --port /dev/ttyACM0
```

**Die Option `--port` kann je nach System unterschiedlich sein!**

## Dein Code

Kompiliere deinen Code
```
mkdir build
cd build/
cmake ..
make
```

Die ausführbaren Dateien deines Codes befinden sich in `build/ass#/task#`

---

# Mobile Robotik
This repository contains the software frameowrk for the course [Mobile Robotik](https://online.tugraz.at/tug_online/ee/rest/pages/slc.tm.cp/course/575774).

Clone the repository
```
git clone --recurse-submodules https://git.sai.tugraz.at/courses/mobile-robotik.git mobile_robotik
```

## Setup
This section will explain how to set up this framework.

### Dynamixel SDK
To interact with the Turtlebot's motores, you need to install the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

```
cd external/dynamixel_sdk/c++/build/linux_sbc/
sudo make install
```

**If you want to install the SKD on your machine instead of the Turtlebot, use the build script that matches your machine's architecture (e.g., `linux64`)!**

### OpenCR
Install the [OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)'s firmware, which is required by this framework, using the [Arduino CLI](https://docs.arduino.cc/arduino-cli/).

Set up and install the dependencies for the OpenCR board.
```
arduino-cli config add board_manager.additional_urls "file://$(pwd)/external/opencr/arduino/opencr_release/package_opencr_index.json"
arduino-cli core install OpenCR:OpenCR
```

Build and flash the firmware to the OpenCR.
You need to connect the OpenCR to your PC (via USB).
```
cd external/opencr/arduino/opencr_arduino/opencr/libraries/OpenCR/examples/10.\ Etc/usb_to_dxl/
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR --build-path build
arduino-cli upload --fqbn OpenCR:OpenCR:OpenCR --build-path build --port /dev/ttyACM0
```

**The `--port` option might be differen!**

## Your code

Build your code
```
mkdir build
cd build/
cmake ..
make
```

The executables of your code are located in `build/ass#/task#`