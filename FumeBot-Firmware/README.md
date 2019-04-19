## **Additional Information about the Firmware**

#### Firmware Usage
The `FumeBOT` folder needs to be opened in Arduino 1.8.7 or higher and the `FumeBOT.ino` file needs to be flashed to
an Arduino MEGA 2560 only. This is because the timers of the microcontroller are now used for generating the square wave required to drive the stepper motor and registers specific to the Atmel ATmega2560 microcontroller are changed.

#### Modification to Servo Libraries
The above modification to the timer register meant some changes needed to be applied to the `Servo.cpp`, `Servo.h` and `ServoTimers.h` files. The changes to these files are necessary because all the timers in the microcontroller are initially setup to generate PWM outputs and the above register modification to generate a required frequency square wave interferers with servo library timer setup. The library is also required to control the pan and tilt servos for the camera fork mount.

To fix the above-mentioned problems, a copy of the servo library was made and altered to not make use of the timers required to generate the square wave for the stepper drivers. The libraries were then renamed to `ServoMod.cpp`, `ServoMod.h` and `ServoTimersMod.h` so that it does not interfere with the original servo libraries provided by Arduino. This was done instead of modifying the original servo libraries because the changes were only needed for this application.