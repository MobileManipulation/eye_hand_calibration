#!/bin/sh
# This script takes as an argument the merged csv file and outputs
# a new file that contains only the relevant information for Lou's
# calibration script.
# That is, it produces a file with these contents:
# 1       : Datapoint
# 2       : Frame
# 3       : Actual Pan of head joint
# 4       : Actual Tilt of head joint
# 5-11    : Actual IIWA joint angles, ordered 0-6
# 12-18   : AR tag location, in camera frame
awk -F, '{OFS=" ";print $1,$2,$27,$28,$29,$30,$31,$32,$33,$34,$35,$13,$14,$15,$16,$17,$18,$19}' $@
