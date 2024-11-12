# BNO085 Bosch Sensortec IMU with integrated sensor fusion for AHRS (STM32G474RE)
The [BNO055](https://github.com/ufnalski/ahrs_bno055_g474re) is marked by the manufacturer as not recommended for new designs. Therefore, let's play with its successor - the mighty BNO085. My exemplary code just configures the sensor system to output AR/VR[^1]-stabilized rotation vector and then periodically reads a report with a unit quaternion. This is to test the device and to give but a flavour of CEVA's SHTP (Sensor Hub Transport Protocol). Bon appetit!

[^1]: Augmented reality / virtual reality.

![BMO085 in action](/Assets/Images/bno085_in_action.jpg)

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Libraries (not yet tested by me)
* [STM32 I2C Library for BNO08x 9 axis IMU](https://www.grozeaion.com/electronics/stm32/stm32-i2c-library-for-bno08x-9-axis-imu) (Ion Grozea)
* [SH2 Sensorhub driver for MCU application](https://github.com/ceva-dsp/sh2) (CEVA)
* [Example Application for CEVA SH2 sensor modules](https://github.com/ceva-dsp/sh2-demo-nucleo) (CEVA)
* [Adafruit BNO08x](https://github.com/adafruit/Adafruit_BNO08x) (Adafruit)
* [SparkFun VR IMU BNO08X Arduino Library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library) (SparkFun)

# Readings
* [SH-2 Reference Manual](https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf) (CEVA)
* [BNO08X Data Sheet](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf) (CEVA)
* [Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085) (Adafruit)
* [CEVA BNO085 Absolute Orientation Sensor System User Guide](https://device.report/manual/12339730)
* [Q-Point](https://in.element14.com/q-point-definition) (element14)
* [Q (number format)](https://en.wikipedia.org/wiki/Q_(number_format)) (Wikipedia)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_popularyzacji_matematyki/Dzien_Popularyzacji_Matematyki_2024.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), [Nikodem Bartnik](https://www.youtube.com/@nikodembartnik), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control engineering - do try this at home :sunglasses:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
