#!/usr/bin/env python

import lxml.etree as ET
import os
from itertools import islice

import numpy as np
import tf.transformations as transf

path = "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/xacro"
urdf_in = "kinect.xacro"
urdf_out = "kinect.xacro"
shouldWrite = True

data_path = "/home/momap/momap_data/log_robot/20170801/20170801T205210_eyehand_ir"
data_in = "Calibration-IR_2017-08-02T152501_UNIX.txt"

# Joint names
joint_names = [
    "kinect1_link_to_depth",
]

# Read the URDF
urdf = ET.parse(os.path.join(path, urdf_in))

# Read the data file
with open(os.path.join(data_path, data_in)) as fp:
    for joint in joint_names:
        xyz = tuple(map(float, islice(fp, 3)))
        quat = tuple(map(float, islice(fp, 4)))

        # Reorder quaternion...
        quat =  quat[1:] + (quat[0],)

        # Construct perturbation matrix
        T = transf.translation_matrix(xyz)
        R = transf.quaternion_matrix(quat)
        Tp = np.dot(T, R)

        print "Perturbation for {}:".format(joint)
        print "trans: (x: {:>8.5f}, y: {:>8.5f}, z: {:>8.5f})".format(*xyz)
        print "rot:   (x: {:>8.5f}, y: {:>8.5f}, z: {:>8.5f}, w: {:>8.5f})".format(*quat)
        print Tp

        # Modify the URDF according to the data
        origin = urdf.xpath("joint[@name = '{}']/origin".format(joint))
        if (len(origin) != 1):
            print "WARNING: Joint {} had {} origin nodes".format(joint, len(origin))
            continue
        origin = origin[0]

        # Read old transformation
        xyz = tuple(map(float, origin.get('xyz', "0 0 0").split()))
        rpy = tuple(map(float, origin.get('rpy', "0 0 0").split()))
        T = transf.translation_matrix(xyz)
        R = transf.euler_matrix(*rpy, axes='sxyz')
        To = np.dot(T, R)

        # Compute new transformation
        Tn = np.dot(To, Tp)

        # Update URDF
        nxyz = transf.translation_from_matrix(Tn)
        nrpy = transf.euler_from_matrix(Tn, axes='sxyz')
        origin.set("xyz", "{} {} {}".format(*nxyz))
        origin.set("rpy", "{} {} {}".format(*nrpy))

        print "Updated {}:".format(joint)
        print "\txyz: ({:>8.5f}, {:>8.5f}, {:>8.5f})".format(*xyz)
        print "\t  -> ({:>8.5f}, {:>8.5f}, {:>8.5f})".format(*nxyz)
        print "\trpy: ({:>8.5f}, {:>8.5f}, {:>8.5f})".format(*rpy)
        print "\t  -> ({:>8.5f}, {:>8.5f}, {:>8.5f})".format(*nrpy)
        print ""

if shouldWrite:
    # Write the updated URDF
    write_path = os.path.join(path, urdf_out)
    urdf.write(write_path,
        encoding='utf-8',
        xml_declaration=True,
        pretty_print=True)
    print "Wrote updated URDF to:", write_path
