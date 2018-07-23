# Rosie Demo Code

This folder contains everything needed to run the Rosie mobile manipulator kit.  It assumes that you have:
* Rosie Kit (Diff-drive, Omni-drive, or Mecanum-drive)
* iOS device capable of running ARKit (iPhone 6S or later, iPad Air 2 or later)

To get started:
* Download this folder
* Start Matlab (2013b or later)
* run `startup.m`
* run `rosieDemo( mobileBaseType )`
** where `mobileBaseType` is: `diff-drive`, `omni`, or `mecanum`

The demo will start up looking for the actuators in the robot, followed by the mobile device for input.  

The frame convention for the robot is:
X = forward (from driver's perspective)
Y = left (from driver's perspective)
Theta = counter-clockwise rotation

The controls are as follows:
| **Input**       | **Command**     | **Note**  |
| ----------------- | ----------------- | ----------- |
| Right Joystick (`A8` and `A7`)  | Base X-Y Velocity |  |
| Left Joystick (`A1`) | Base Theta Velocity |  |
| Slider `A6` | Gipper force | Drag down to close. Drag up to open |
| Phone Pose | Arm end-effector pose |  |
| Slider `A3` | Scale of end-effector translation  | Should normally be all the way up. Drag this to all the way down, to reset the pose and hold it place. |
| Button `B1` | Reset Pose of the gripper| When you press this, make sure the phone is oriented face down, matching the gripper. |
