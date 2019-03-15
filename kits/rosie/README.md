# Rosie Demo Code

This folder contains everything needed to run the Rosie mobile manipulator kit.  It assumes that you have:
* Rosie Kit (Diff-drive, Omni-drive, or Mecanum-drive)
* Matlab (2013b or newer)
* An mobile device capable runing the [HEBI Mobile I/O](http://docs.hebi.us/tools.html#mobile-io) application for [iOS](https://itunes.apple.com/gb/app/hebi-virtual-io/id1385180764?mt=8&ign-mpt=uo%3D2) / [Android](https://play.google.com/store/apps/details?id=us.hebi.android.node).  The device will need to be capable of running ARKit (iPhone 6S and later, iPad 2017 or later / iPad Pro), or ARCore (Google Pixel phones, other recent Android phones).

If you are running code on the computer inside the robot, there are instructions below for installing a script to have the demo launch automatically on startup.

## To get started:
* Download this folder
* Start Matlab 
* run `startup.m`
* run `rosieDemo( 'mobileBaseType' )`
  - where `'mobileBaseType'` is: `'diff-drive'`, `'omni'`, or `'mecanum'`

The demo will start up looking for the actuators in the robot, followed by the mobile device for input. 
* You will need to change the *Family* of the device running Mobile IO to *Rosie*.  This can be done the in the app settings on the mobile device, or through [Scope](http://docs.hebi.us/tools.html#scope-gui)
* If the mobile device is not found, check to make sure it is on the same network as the robot.

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

Note that if you move the location of `rosieStart.sh` you will have to delete the symbolic link from `/usr/bin` and run `install.sh` again.    You can do this by typing: `sudo rm /usr/bin/rosieStart.sh` and then `./install.sh` again.

