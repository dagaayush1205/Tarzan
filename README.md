# Tarzan 
Zephyr Project to design control system for Rudra's R25 rover using STM32 Nucleo H7a3zi_q.

## Directory Structure 

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
west init -m https://github.com/dagaayush2105/Tarzan 
west config --global update.narrow true
west update
pip install -r zephyr/scripts/requirements-base.txt
source zephyr/zephyr-env.sh
```
4. If everythin is installed correctly you can build src code
```
west build -p=always -o=-j8 -b nucleo_h7a3ziq Tarzan/drive
```
