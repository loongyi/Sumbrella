# Sumbrella
Functional Fashion IROS 2023 UoB Softlab team

raspberry pi-openrb-150 arduino for dynamixel control-stm32 pneumatic ciruit
dynamixel motor is xl330. pneumatic circuit uses 12v pumps and solenoid valves with PID pressure regulation. We just need to tell the arduino motor controller the motor states, and the pneumatic valve the channel and pressure.

Rpi sends serial to arduino and pneumatic circuit to change their states.

both the arduino and pneumatic circuit have a finite state machine
that services the motors and pumps/valves

to use arduino without raspberry pi:
open up serial monitor- send the numbers below:
serial commands:
0-9 - digits to send in serial monitors
states
0 - reset/stop
1 - pull left up / release right if it is up
2 - pull right up / release left if it is up
3 - release left
4 - release right
5 - pull back uo
6 - release back
7 - pull all up
8 - release all down
9 - wave (with a frequency as an input)

pull - pulling until current limit
release - remembers its home position - counts the rotations needed (stroke) when pulls up - returns to pre-stroke home position.

to change the wave:
AMPLITUDE: no. of rotations/angle
FREQUENCY (function input): effectively speed
blimit and ulimit: like upper and lower limit of pwm for the motor

the wave uses velocity control and a proportional controller to emulate a sine wave motion in the tentacles, 
you can change the phase shift of the tentacles for offset wavey motions or set them equal to have all tentacles rotate in sync.


