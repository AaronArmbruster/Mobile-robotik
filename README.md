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
arduino-cli config add board_manager.additional_urls "file://ABSOLUTE_PATH_TO_THIS_REPOSITORY/external/opencr/arduino/opencr_release/package_opencr_index.json"
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

Build the your code
```
mkdir build
cd build/
cmake ..
make
```

The executables of your code are located in `build/ass#/task#`