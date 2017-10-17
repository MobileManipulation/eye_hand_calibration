# eye\_hand\_calibration

WARNING - this code requires several other ROS nodes to run - currently UNDER CONSTRUCTION

This repository contains tools for performing eye-hand calibration on the KUKA arm
with PTU/Kinect head. It consists of two tools:

## Data Collection Tool

This tool is a control script, eyehand.py, that commands the KUKA arm in a grid with
an Aruco tag calibration plate as its end effector. In all target poses, the calibration
plate remains parallel to the camera.

This control script delegates the data collection task to a C++ observation node. This
node collects as much state as possible about the position of the arm at every position,
to be used for later calibration processing.

## Data Baking Tool

This tool performs several post-processing tasks on the raw data collected above. Namely, it
computes the average location of the arm at each point in time, as measured by both the AR
tags and forward kinematics, and cleverly crops the point cloud down to include only the
points from the calibration plate. It also computes a combined point cloud from all frames
of a given datapoint, and performs and ICP fit of a plane to the data.

## Generating the Calibration URDF
Running "go.sh calibration" in momap_descriptions will generate the correct URDF for calibration

Collection script information
-----------------------------
The eye_hand_calibration repo (ROS package) is in archive.tgz.

To run the data collection again, use these commands after initalizing
the robot drivers and the kinect perception stack:
	roslaunch eyehand_calib calib_stack.launch
	rosrun eyehand_calib eyehand.py

Saved IK solutions: full_calib_ik.save
Saved trajectory solutions: full_calib_traj.save (Count: 61)
Saved URDF: 20170817T183837_kinect2_calib_ir.urdf

Data format
-----------

Each folder and raw data CSV file corresponds to a unique head/arm pose.
This corresponds to the "datapoints" referenced later in this document.

At each datapoint, several frames were collected. Details of frame colletions
are shown below. An approximate time filter was used to gather all data
simulataneously to within a small margin of error.

Merged data
-----------

* 20170816T160945_kinect2_calib-merged.csv contains all the collected data except point clouds and images,
organized by datapoint and frame.

It was generated using the command:
	find . -name "*_raw_data.csv" | cut -b 3-5 | xargs -I{} sed 's/^/{},/' {}_raw_data.csv > 20170817T183837_kinect2_calib_ir-merged.csv

The columns are:
0	: Datapoint
1	: Frame
2	: Timestamp
3       : Desired Pan of head joint
4	: Desired Tilt of head joint
5-11	: Desired IIWA joint angles, ordered 0-6
12	: Actual Pan of head joint
13	: Actual Tilt of head joint
14-20	: Actual IIWA joint angles, ordered 0-6
21-27	: AR tag location, in camera frame
28-35	: AR tag corner pixel locations (CW from top right)
36-42	: AR tag top left corner location, in camera frame
43-49	: AR tag top right corner location, in camera frame
50-56	: AR tag bottom left corner location, in camera frame
57-63	: AR tag bottom right corner location, in camera frame
