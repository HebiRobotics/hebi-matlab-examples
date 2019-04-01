# Rosie Demo Code

This folder contains everything needed to run the Daisy hexapod kit.  It assumes that you have:
* Daisy Kit
* iOS or Android mobile device (phone or tablet)

## Unpacking Florence
The robot ships folded and packed in a single Pelican Case.  

Included in the case is:
- Florence Robot
- Gas Springs (6x)
- Batteries (4x)
- Chargers (2x)
- WiFi Antennas (2x)

The gas springs and WiFi antennas will need to be attached to the robot after unpacking.  

## Starting Daisy

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


## To get started:
* Download this folder
* Start Matlab (2013b or later)
* run `startup.m`
* run `daisyDemo.m`

The demo will start up looking for the actuators in the robot, followed by the mobile device for input.  


## Coordinate frame convention

* `X` = forward (from driver's perspective)
* `Y` = left (from driver's perspective)
* `Theta` = counter-clockwise rotation


## Input Controls

| Input      | Command   | Note  |
| ----------------- | ----------------- | ----------- |
| Right Joystick (`A8` and `A7`)  | Base X-Y Velocity |  |
| Left Joystick (`A1`) | Base Theta Velocity |  |
| Slider (`A6`) | Gipper force | Drag down to close. Drag up to open |
| Phone Pose | Arm end-effector pose |  |
| Slider (`A3`) | Scale of end-effector translation  | Should normally be all the way up. Drag this to all the way down, to reset the pose and hold it place. |
| Button (`B1`) | Reset Pose of the gripper| When you press this, make sure the phone is oriented face down, matching the gripper. |


## Automatic startup instructions for Linux

This process was tested using Ubuntu 16.04.

The folder also contains scripts designed to automatically launch the demo. `rosieStart.sh` is used to launch Matlab and start the script with the correct path (by default it assumes the `omni` type, but it can be edited). The `install.sh` script will create a symbolic link to `rosieStart.sh` so that the user can simply type `rosieStart` into a terminal window and the Matlab script will execute.

To install:
* Open a command prompt and navigate to the top level `rosie` kit directory
* Make sure that the `install.sh` script has execute permissions: `chmod +x install.sh`
* Run `install.sh`: `./install.sh`

If you would like to have the script start on boot, add the `rosieStart` command under `Startup Applications` if using Ubuntu or use another standard method for automatically launching the program.

Note that if you move the location of `rosieStart.sh` you will have to delete the symbolic link from `/usr/bin` and run `install.sh` again.

