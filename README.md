# Tarzan 
Zephyr Project to design control system for Rudra's R25 rover using STM32 Nucleo H7a3zi_q.

## Directory Structure 
```
── tarzan
    ├── drive
    │   ├── boards
    │   │   └── app.overlay
    │   ├── CMakeLists.txt
    │   ├── prj.conf
    │   └── src
    │       ├── calibrate_imu.c
    │       ├── cdc_test.c
    │       ├── gps_test.c
    │       ├── imu_test.c
    │       ├── limit-switch_test.c
    │       ├── mag_test.c
    │       ├── main_abex.c
    │       ├── main_auto.c
    │       ├── main.c
    │       ├── sbus_test.c
    │       ├── stepper_test.c
    │       ├── temp.c
    │       └── uart_raw_data.c
    ├── include
    │   └── Tarzan
    │       └── lib
    │           ├── arm.h
    │           ├── cobs.h
    │           ├── drive.h
    │           └── sbus.h
    ├── lib
    │   ├── arm.c
    │   ├── cobs.c
    │   ├── drive.c
    │   └── sbus.c
    ├── misc
    │   ├── pcb
    │   │   ├── Nucleo_PCB_v1.0
    │   │   └── Nucleo_PCB_v2.0
    │   └── sample_data
    │       ├── Arduino_stepper_pulse
    │       ├── gps_data.txt
    │       ├── v0.2.2_20us_OC
    │       └── v0.2.2_50us_OC
    ├── README.md
    ├── sample
    │   ├── boards
    │   │   └── app.overlay
    │   ├── CMakeLists.txt
    │   ├── prj.conf
    │   ├── sample.yml 
    │   └── src
    │       └── sbus_test.c
    ├── scripts
    │   └── requirements.txt
    ├── tests
    │   ├── cobs_data.txt
    │   ├── datarx_invert.txt
    │   ├── drive_test.c
    │   ├── gps_visualizer.py
    │   ├── sbus_data
    │   ├── sbus_packet
    │   └── sbus_parse.c
    ├── west.yml
    └── zephyr
        ├── CMakeLists.txt
        ├── Kconfig
        └── module.yml
```
## Installation Guide 
**To Procced with the Installation you are required to install all the zephyr dependencies (refer to Zephyr Docs)**
1. Firstly create a zephyr workspace as following
```
mkdir Tarzan_ws
cd Tarzan_ws
```
2. Create a python virtual envoirment to install west
```
python3 -m venv .venv
source .venv/bin/activate
```
3. Init the workspace by pulling the repo using west
```
west init -m https://github.com/dagaayush1205/Tarzan 
west config --global update.narrow true
west update
pip install -r zephyr/scripts/requirements-base.txt
source zephyr/zephyr-env.sh
```
4. If everything is installed correctly you can build src code
```
west build -p=always -o=-j8 -b nucleo_h7a3zi_q Tarzan/drive
```
