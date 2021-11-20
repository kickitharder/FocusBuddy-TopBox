# FocusBuddy-TopBox

FocusBuddy TopBox is a box fixed to the top off my 10" Meade LX200 SCT Classic Telescope.  It's primary purpose is to cut down the number of cables being fed to the equipment attached to the telescope, eleven in all (2x CCD power, CCD USB, guide camera USB, guide ST-4 cable, 2 x dew heater power, primary focuser motor, focuser stepper motor, filter wheel usb cable, temperature sensor).

The ToxBox only requires two - a power and USB cable.  Incorporated within it is a 7 port USB hub, 2x SBIG 5-pin DIN sockets, 3.5mm mono socket for primary focuser, 6 pin RJ11 socket for focuser stepper motor, two phono sockets for dew heaters, 12V 2.5mm power socket, 4 pin mini DIN for temperature sensor, 3 pin DIN socket for an electroluminescence panel for flat fields.

Inside, 3 motor controller modules are used to control the primary focuser and the two dew heaters.  They allow the power to them to be controlled by the TopBox Arduino Nano.  The same Arduino also can turn on/off the power to the SBIG CCD camera, the 12V power socket, the focuser stepper motor, and the power to the inverter for power an electroluminescence panel for flat fields.

Also on board is an Arduino Nano based myFocuserPro2 PCB (with Bluetooth) for controlling the stepper motor attached to my MoonLite focuser.  The temperature sensor is attached to it.

Both Arduinos are connected to the USB hub and are controlled by PC apps and ASCOM drivers.
