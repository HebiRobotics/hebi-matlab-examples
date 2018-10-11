
# Scanning Arm Demo Code

This folder contains everything needed to run the NDT scanning arm. It assumes that you have:

* 4-DOF Arm that matches the HRDF file in this folder.
* I/O Board running the custom firmware for talking to an Olympus Omniscan MX2
* iOS or Android device running the HEBI Mobile I/O App.


## To get started:

Download the `hebi-matlab-examples` folder (make sure you get this branch).
Start Matlab (2013b or later)
run `startup.m`
run `scanningArmRaster.m`
The demo will start up looking for the actuators in the robot, followed by the mobile device for input.  

To run a version of the demo that wraps try-catch block so that it never crashes, run `scanningArmDemoRunner.m`.


## Input Controls

| Input      | Command   | Note  |
| ----------------- | ----------------- | ----------- |
| Button (`B1`)  | Hold down to allow hand-positioning of probe | Arm will do IK to keep the probe aligned with Z-axis of the base frame. |
| Button (`B4`)  | Reset Omniscan display and coordinates | This is a little janky.  Might need to press twice. |
| Button (`B8`)  | Start/stop raster scanning |  |
| Left Joystick (`A1` and `A2`)  | Jog probe in X-Y plane |  |
| Slider (`A3`) | Raster speed | Top position is top speed.  Bottom position will completely pause rastering. |
| Slider (`A4`) | Probe angle adjust | Middle is no adjustment. Up and down slightly adjusts the angle of the wrist module for better probe contact. |
| Slider (`A6`) | Downward force | Top position is full down-force.  Bottom position is no additional down-force (arm is 'weightless'). 


## Automatic startup instructions for Linux

This process was tested using Ubuntu 16.04.

The folder also contains scripts designed to automatically launch the demo. `robotStart.sh` is used to launch Matlab and start the script with the correct path. The `install.sh` script will create a symbolic link to `robotStart.sh` so that the user can simply type `robotStart` into a terminal window and the Matlab script will execute.

To install:
* Open a command prompt and navigate to the top level `scanningArm` directory (the one with this README).
* Make sure that the `install.sh` script has execute permissions: `chmod +x install.sh`
* Run `install.sh`: `./install.sh`

If you would like to have the script start on boot, add the `robotStart` command under `Startup Applications` if using Ubuntu or use another standard method for automatically launching the program.

Note that if you replace or move the location of `robotStart.sh`, you will have to delete the symbolic link from `/usr/bin/` and run `install.sh` again.
