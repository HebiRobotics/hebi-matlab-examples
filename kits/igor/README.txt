Igor Balancing Robot Matlab Demo
HEBI Robotics
June 2018

This demo assumes you have:
Matlab 2013b (or later)
Sony Playstation Dualshock 4 USB gamepad.

========================

To run the main demo code for Igor from Matlab:

- Open Matlab
- Navigate to this folder hebi-matlab-examples/kits/igor/
- Run startup.m
- Run igor2StartupIntegrated.m

This will launch the a Matlab function that assumes a joystick is connected to the computer.  The joystick is assumed to be a Sony Playstation Dualshock 4 USB gamepad.

==========================

Almost all of control code for Igor is contained within igor2DemoIntegrated.m.  

The function will look for modules, look for a gamepad, make the user wiggle all the axes of the gamepad, and then start the demo on a left-joystick-click of the gamepad.

After starting the demo with left-joystick-click the robot will standup partially and start balancing.  Press the lower-left trigger to stand up the rest way.  

Controls:
Right Joystick - Robot fwd/back and turn
Left Joystick - Arms fwd/back and in/out
Lower Triggers - Stand up/down
Upper Bumpers - Arms up/down
D-pad Up/Down - Wrist spin in/out

HOLD DOWN OPTION TO END THE DEMO.  The robot will squat down and exit the main loop when it hits the bottom of the leg travel.  The robot will then wait for a left-joystick-click to restart the demo.

=========================

To run the main demo automatically, there are scripts provided:
- igorStart.sh - A shell script for Linux
- igorStart.bat - A batch file for Windows

Both of these scripts may need to be modified so that the paths to Matlab or the Igor code directory match the path on your machine.








