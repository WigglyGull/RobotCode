# Arduino Robot Control Code
Software to control an Arduino robot, making use of a custom shield based on the [ZXBM5210 motor driver](https://www.diodes.com/assets/Datasheets/products_inactive_data/ZXBM5210.pdf), and a chassis to hold two ["TT" geared DC motors](https://www.adafruit.com/product/3777). 

## If you're marking this
Source code can be found under [/src](https://github.com/WigglyGull/RobotCode/tree/main/src). A custom library was also written to extend the provided UcTTDcMotor library, which can be found under [/lib/DualMotorControl/src](https://github.com/WigglyGull/RobotCode/tree/main/lib/DualMotorControl/src)