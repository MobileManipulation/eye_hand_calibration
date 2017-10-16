# eyehand\_calib

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

