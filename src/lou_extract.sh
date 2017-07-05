#!/bin/sh
# This script takes as an argument the merged csv file and outputs
# a new file that contains only the relevant information for Lou's
# calibration script.
# That is, it produces a file with these contents:
# 1       : Cluster
# 2       : Datapoint
# 3       : Frame
# 4       : X component of plane normal
# 5       : Y component of plane normal
# 6       : Z component of plane normal
# 7       : Distance (along normal direction) from origin to plane
awk -F, '{OFS=" ";print $1,$2,$3,$42,$43,$44,$45,$46,$47,$48,$49,$50,$28,$29,$30,$31,$32,$33,$34}' $@
