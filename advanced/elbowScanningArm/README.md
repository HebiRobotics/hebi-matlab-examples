# Pipe Elbow Scanning Arm

The code here is for an 8-DoF arm designed to investigate the feasibility of automatically scanning a pipe elbow for NDT inspection.  There are scripts that register to and raster a pipe, as well as scripts that do a general purpose teach-repeat and visualize feedback from the arm.


## Getting Started
Run the `startup.m` script one level up from this folder.  This will load the API and set up all the directories that need to be in Matlab's path.  You can then run the following scripts from this folder.


## ex_kinematicsVisualization.m
This script will put the arm in a gravity-compensated mode and visualize the frames of each rigid body in a Matlab figure.  Press `ctrl-C` to stop this script.


## ex_teachRepeat.m
This script will put the arm in a gravity-compensated mode and allow you to move it to different poses to train them as waypoints.  Move the arm into a position and press `ALT` for each new waypoint.  When you are done, press `ESC` and the arm will start moving between the trained waypoints.  Press and hold `ESC` to end the demo and plot end effector error and other logged data.

*NOTE: This demo uses a blocking API, so you will need to hold ESC down until the arm reaches a waypoing in order for the keyboard to be read and end the demo.


## pipeMapping.m
This script is used to train the arm's position on a new pipe elbow.  Follow the instructions in script to move the arm along the oustide of the elbow, and then radially around the pipe at different points.  A video of the process is also shown below.  Once the pipe has been 'mapped' by the arm's end effector, the pipe parameters are stored in a `.mat` file, `pipeData_latest.mat`.

Video Links:
* Training - Pipe Elbow Measurement / Registration
  * https://youtu.be/TnMrLfVjM5A

* Data Plot - Pipe Registration
  * https://youtu.be/9AUYly1QvpE
  

## pipeRaster.m 
This script moves the arm up and down the pipe in a raster pattern.  It loads previously saved pipe data from `pipeMapping.m`.  Follow the on-screen instructions in script.  When you first start the script you have the option of putting the arm in positions around the pipe for the IK solver to use for initial positions.  These positions are important for the arm to find a good solution for IK that doesn't collide with the pipe.  If you skip this step a default set of initial IK positions will be loaded.
  
Video Links:
* Inspection
  * https://youtu.be/YhsyXQfFDv4
  * https://youtu.be/AfGZ6Vq50YI
  * https://youtu.be/XgTT_Cyjc08

* Data Plot Pipe Coverage Plot
  * https://youtu.be/tKntWUpHhmw
