# Florence - Biped Kit

## Unpacking Florence
The robot ships folded and packed in a single Pelican Case.  

Included in the case is:
- Florence Robot
- Gas Springs (2x)
- Batteries (4x)
- Chargers (2x)
- WiFi Antennas (2x)

The gas springs and WiFi antennas will need to be attached to the robot after unpacking.  There are photos of the packing process at the link below: 
https://www.dropbox.com/sh/liuyvpv9i1d8ft1/AABBnn8OZRLyZwI8TJBr-LHsa?dl=0

Follow the photos in order for packing the robot, follow the photos in reverse to unpack the robot.


## Starting Florence

#### Powering up the robot:
- Mount and plug in at least 1 of the batteries (preferably both).  
- [Turn on the batteries](https://www.ebikes.ca/product-info/ligo-batteries.html) by pressing and holding the button on the batteries until the lights flash green.
- After the batteries turn on, you will hear 2 beeps, followed by 2 double-beeps as the router in the robot powers up.
- Press and hold the button labeled 'Modules'.  When the power to the modules turns on the button will turn blue.
- Turn on the power to the onboard computer.  When the power to the computer turns on the button will turn white.  *The computer does not need to be on for the modules in the robot to be seen on the network.*
- Make sure the M-Stop button is off (not pressed in).

#### Getting the robot ready to run:
- **Make sure all of the legs are within their joint limits.** The [status LEDs](http://docs.hebi.us/core_concepts.html#led-status-codes) on the actuators will blink orange if the module is outside the joint limit.  When the actuators rotate to within their joint limits, the LEDs will change to a slow green fade.  The joints that will have to be rotated the most will be the ankle joints.  
- You can also use [Scope](http://docs.hebi.us/tools.html#scope-gui) to check the position feedback and limits on each joint.
- You can connect to the modules on the robot directly from an external computer by accessing the robot's WiFi network (default password `hebi1234`) or by plugging into the ethernet port on the back of the robot.  **If you are controlling the robot from an external computer, you should always use the wired ethernet connection.**

#### Connecting a Mobile Device for Control:
- The code to control the robot assumes that a mobile device is on the network running the [HEBI Mobile I/O App](http://docs.hebi.us/tools.html#mobile-io).  The device will need to be on the same network as the robot (usually connected to the robot's WiFi network), and it will need to be set to the Family `Florence` and the Name `_Controller`.  You can set the Family/Name of the mobile device in [Scope](http://docs.hebi.us/tools.html#scope-gui), or in the settings for the app on the mobile device.


## MATLAB Example Code

The main scripts that you run to control the robot are listed below.  Other functions are helper functions that get called by these scripts.

#### floBalancing.m
Commands the robot to balnce on two legs and control stance parameters based on inputs from the controller.

#### captureStepWalking.m
Implementation of the walking algorithm described in ["Balanced Walking with Capture Steps"](https://www.ais.uni-bonn.de/papers/RoboCup_2014_Missura_Capture_Steps.pdf) by Missura and Behnke.  It connects to a mobile device running the [HEBI Mobile I/O App](http://docs.hebi.us/tools.html#mobile-io) and allows you to drive a simulated point-mass inverted pendulum robot around.  This algorithm will get implemetented in `floWalking.m` to control the actual robot.

#### floWalking.m
*Not yet completed.* Will command the robot to walk based on velocity inputs from the controller and the walking algorithm implemented in `captureStepWalking.m`. 

#### floVisualize.m
Visualizes the robot's kinematics in 3D based on feedback from the robot.  This is designed to run in a second instance of Matlab, or on another computer from the control computer so that visualization does not slow down the control of the robot.

#### floLegKinematics.m
Visualizes the kinematics of the robot based on desired commands, and provides basic kinematics and quasi-dynamic simulation.  This was an early test function to verify that the kinematics of the robot were configured correctly and that the joints of the robot could provide the appropriate torques and velocities need to stand and walk.
