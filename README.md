# SSR1PCB
An ESP32 Motor Control board for the TMAx SR1.
Integrates USB Power Delivery at 100W (use appropriate cabling)

:warning: **This device can use dangerous amounts of power**: Device may become very hot in use and may present a fire hazard. Operate at own risk.
Device can draw up to 100W from a compatible power supply. Ensure all cabling and any power devices are rated for this much power dissipation.
Device may present a shock hazard.

## Hardware V1.3
![hardware v1.3 pcb](HW_Front_1.3.jpg "Hardware Version 1.3")
### USB PD
USB power delivery port can sink up to 5A at 20V, meeting the maximum 100W USB PD standard.
When operated at maximum power, a cable rated for 100W should be used and power supply rated for 100W should be used.
Device does not operate as a power source.

Device should require less than 45W during operation, however it is recommended to use at least a 65W rated power supply.
### USB Data
A second usb port is used for data and low voltage programming.
When using this port without the PD port, any motors or other high draw loads should be disconnected from the board.
PD operation will energize the 5V power domain, but is disconnected from the 5V rail of the port.
### External power
The power regulators are exposed through user accessable pins.
A 12V, 5V, 3.3V and PD rail (20V) are exposed.
20V total draw should not exceed 5A.

Note: most 2.54mm headers are not rated for more than 1A per pin. Excercise caution when drawing current from the power rails.
:warning: The 12V power supply is limited to 3A internally, but may be limited by the maximum power of the PD interface during operation.
:warning: The 5V power supply is limited to 3A internally, but may be limited by the maximum power of the PD interface during operation. The 3V3 supply depends on the 5V rail, and may be adversely effected by excessive current draw.
:warning: The 3.3V total draw should not exceed 500mA. Using the 3.3V power rail may adversely effect the operation of the ESP32. Activating WiFi and/or bluetooth connections may increase the load on the 3.3V rail.

## Pinout
![hwv1.3 pinouts](SSR1PCB_1.3.png "Pinouts")
1) ESP32 uses multiplexed peripherals, hardware assignments are recomended
2) SPI (vspi) Pins required to access encoder

| Pin | Arduino GPIO | Function/Description |
| --- | ------------ | -------------------- |
|   9 |       GPIO33 | User IO 4            |
|  10 |       GPIO25 | User IO 3            |
|  11 |       GPIO26 | User IO 2            |
|  12 |       GPIO27 | User IO 1            |
|  13 |       GPIO14 | User / Hall Effect   |
|  14 |       GPIO12 | User IO 0            |
|  13 |       GPIO13 | User IO 5            |
|  23 |       GPIO15 | FOC N_FAULT          |
|  24 |        GPIO2 | FOC IN1              |
|  25 |         BOOT | BOOT                 |
|  26 |        GPIO4 | FOC Enable           |
|  27 |       GPIO16 | FOC IN2              |
|  28 |       GPIO17 | FOC IN3              |
|  29 |        GPIO5 | SSI/SPI CS           |
|  30 |       GPIO18 | SSI/SPI CLK          |
|  31 |       GPIO19 | SSI DATA/SPI MISO    |
|  33 |       GPIO21 | I2C SCL              |
|  34 |       RXD0   | UART (usb reserved)  |
|  35 |       TXD0   | UART (usb reserved)  |
|  36 |       GPIO22 | I2C SDA              |


## Peripherals
Onboard peripherals include 
* MT6701QT-STD rotary encoder in SSI mode, wired to vpsi interface (GPIO5,18,19)
* DRV8313PWPR FOC controller (GPIO2,4,16,17)
* CP2102-GMR USB-UART transcoder/controller (TX,RX)

## Software
See [firmware](firmware/) folder for basic TCode implementation.
Built with Platformio.

### Troubleshooting/FAQ
:warning:
#### My motor wont spin
Check that the 20V rail is powered (USB PC) with a source that is 100W certified.
Ensure cable used is 100W certified.
#### My motor wont spin/vibrates
Check that you have not screwed too far into the motor and damaged the coils.
Replace motor if so.
#### My motor spins initially, then just vibrates
Check encoder magnet orientation. Incorrect orientation will cause unpredictable motor behavior. N/S poles must be orthogonal to encoder.
