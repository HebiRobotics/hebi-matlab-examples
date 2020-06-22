# Rosie Demo Code

This folder contains everything needed to run the Rosie mobile manipulator kit.  It assumes that you have:
* Rosie Kit (Diff-drive, Omni-drive, or Mecanum-drive)
* iOS device capable of running ARKit (iPhone 6S or later, iPad 2017 or later, iPad Pro)
   or
* Android device capable of running ARCore (Google Pixel or recent Samsung Galaxy phones)

## To get started:
* Download this folder
* Start Matlab (2013b or later)
* run `startup.m`
* run `rosieDemo( 'mobileBaseType' )`
  - where `'mobileBaseType'` is: `'diff-drive'`, `'omni'`, or `'mecanum'`

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
| Slider (`A6`) | Gripper force | Drag down to close. Drag up to open |
| Phone Pose | Arm end-effector pose |  |
| Slider (`A3`) | Scale of end-effector translation  | Should normally be all the way up. Drag this to all the way down, to reset the pose and hold it place. |
| Button (`B1`) | Reset Pose of the gripper| When you press this, make sure the phone is oriented face down, matching the gripper. |
| Button (`B2`) | Compliant Mode| The arm stops commanding position and velocities, and will remain in a compliant gravity-compensation mode for the duration of the press. |
| Button (`B8`) | Exit Script| |

## Automatic startup instructions for Linux

This process was tested using Ubuntu 16.04.

The folder also contains scripts designed to automatically launch the demo. `rosieStart.sh` is used to launch Matlab and start the script with the correct path (by default it assumes the `omni` type, but it can be edited). The `install.sh` script will create a symbolic link to `rosieStart.sh` so that the user can simply type `rosieStart` into a terminal window and the Matlab script will execute.

To install:
* Open a command prompt and navigate to the top level `rosie` kit directory
* Make sure that the `install.sh` script has execute permissions: `chmod +x install.sh`
* Run `install.sh`: `./install.sh`

If you would like to have the script start on boot, add the `rosieStart` command under `Startup Applications` if using Ubuntu or use another standard method for automatically launching the program.

Note that if you move the location of `rosieStart.sh` you will have to delete the symbolic link from `/usr/bin` and run `install.sh` again.

