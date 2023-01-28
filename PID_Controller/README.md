## PID Controller

- namespace ***pid***

## ***include folder***

**Following classes are stand-alone**
- **Timer.h**          
    - (The Timer.h file is located in the Numerics Repository)
- **RandomNumber.h**   
    - (The RandomNumber.h file is located in the Numerics Repository)
- **Controller.h**         
    - (PID Controller)

***Control Manager class require the following dependencies***
+ ***ControlManager.h***     
    - (depended on *Controller.h*, *LinearMap.h* and *RandomNumber.h*)

## ***src folder***

- main.cpp  
    - (depended on *Timer.h*)

- Note: 20230127 - Continuing testing Kp from zero to 1. Ki and Kd set to zero.

## ***root folder***

- platformio.ini        
    - (Please review this document for platform configuration while testing the code.)

## ***Notes***

- 20230127 Code changes... Removed LinearMap.h because there are too many variables for testing...

