# sr1pcb
An ESP32 Motor Control board for the TMAx SR1.
Integrates USB Power Delivery at 100W (use appropriate cabling)

:warning: **This device can use dangerous amounts of power**: Device may become very hot in use and may present a fire hazard. Operate at own risk.
Device can draw up to 100W from a compatible power supply. Ensure all cabling and any power devices are rated for this much power dissipation.
Device may present a shock hazard.

## Hardware V1
![hardware v1 pcb](HWv1.png "Hardware Version 1")
### USB PD
USB power delivery port can sink up to 5A at 20V, meeting the maximum 100W usb-c PD standard.
A cable rated for 100W draw should be used. A power supply rated for 100W draw should be used.
Device does not operate as a source.
### USB Data
A second usb port is used for data and low voltage programming.
When using this port without the PD port, any motors or other high draw loads should be disconnected from the board.
PD operation will energize the 5V power domain.
### External power
The power regulators are exposed through user accessable pins.
A 12V, 5V, 3.3V and PD rail (20v) are exposed.
20V total draw should not exceed 5A. Note that pin headers may not be rated for more than 1A draw. 20V connections should not draw more than 1A.
12V total draw should not exceed 3A. Not that pin headers may not be rated for more than 1A draw, 12V connections should not draw more than 1A.
5V total draw should not exceed 1A. Note that pin headers may not be rated for more than 1A draw. 5V connections should not draw more than 1A.
3.3V total draw should not exceed 1A. Note that pin headers may not be rated for more than 1A draw. 3.3 connections should not draw more than 500mA.
Using the 3.3V power rail may adversely effect the operation of the ESP32.

## Pinout
![hwv1 pinouts](SSR [1] PCB.png "Pinouts")
1) ESP32 uses multiplexed peripherals, hardware assignments are recomended
2) I2C Pins required to access encoder

## Peripherals
Onboard peripherals include 
* TMP236A4DBZR temperature sensor
* MT6701QT-STD rotary encoder
* DRV8313PWPR FOC controller
* CP2102-GMR USB-UART transcoder/controller

### I2C Addresses (default)
Rotary Encoder: 0x06 - 7 bit mode

## Software
TODO
