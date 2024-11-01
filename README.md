# dual_motor_hat_ROS2
ROS2 Driver for DFRobot DFR0592 motor driver hat with encoder inputs. Will be used with dfrobot micro metal gear motors w/encoders.

# Custom Firmware
This is designed for the custom alternate firmware with encoder ticks and overall better design found at 
https://gitlab.telecom-paris.fr/software/dc-motor-driver-hat

## Build and Install using cargo

Will need a host with rust installed (ie ubuntu), and an ST-LINK programmer for initial setup. See the last page of the schematics on the dfrobot website for SWD pad locations. No need for reset pin.

See the ```INSTALL.md``` at above repository

Note: ```cargo install probe-rs-tools``` should be replaced with ```cargo install probe-rs-tools --locked``` to avoid errors with cargo install. 

My build had troubles loading the firmware due to a miss-matched CPUAPID. My dfrobot board uses an STM32 clone, Geehy APM32F103C8T6, this is possibly the cause. After building the elf files are located at ```[repository-dir]/target/thumbv7m-none-eabi/production/``` directory, so you can flash using openocd:

```openocd -f interface/stlink.cfg -c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "init" -c "reset halt" -c "stm32f1x unlock 0" -c "reset halt" -c "program [repository-dir]/target/thumbv7m-none-eabi/production/bootloader verify reset exit"```
