import xml.etree.ElementTree as ET
import os
from itertools import islice

import numpy as np
import tf.transformations as transf

path = "/home/momap/momap/src/robot_core/robot_descriptions/robot_description/urdf"
urdf_in = "draper_ptu_gripper-rviz2.urdf"
urdf_out = "draper_ptu_gripper-eyehand1.urdf"
shouldWrite = True

data_path = path
data_in = "Calibration_2017-06-28T1301_UNIX.txt"

# Joint names
joint_names = [
    "head_mount_joint",
    "ptu_base",
    "ptu_pan",
    "ptu_tilt",
    "ptu_mount",
    "camera_joint",
    "kinect1_optical_joint",
    "jnt_kinect1_base_link"
]

# Read the URDF
urdf = ET.parse(os.path.join(data_path, urdf_in))

# Read the data file
joint_data = dict()
with open(os.path.join(path, data_in)) as fp:
    for joint in joint_names:
        xyz = tuple(map(float, islice(fp, 3)))
        quat = tuple(map(float, islice(fp, 4)))

        print joint, xyz, quat

        # Construct perturbation matrix
        T = transf.translation_matrix(xyz)
        R = transf.quaternion_matrix(quat)
        joint_data[joint] = np.dot(T, R)

        print joint_data[joint]


# Modify the URDF according to the data
for joint in urdf.findall('joint'):
    # Filter to joints we care about
    joint_name = joint.get("name")
    if joint_name in joint_data:
        origin = joint.find('origin')

        # Read old transformation
        xyz = tuple(map(float, origin.get('xyz', "0 0 0").split()))
        rpy = tuple(map(float, origin.get('rpy', "0 0 0").split()))
        T = transf.translation_matrix(xyz)
        R = transf.euler_matrix(*rpy, axes='sxyz')
        To = np.dot(T, R)

        # Compute new transformation
        Tp = joint_data[joint_name]
        Tn = np.dot(To, Tp)

        # Update URDF
        nxyz = transf.translation_from_matrix(Tn)
        nrpy = transf.euler_from_matrix(Tn, axes='sxyz')
        origin.set("xyz", "{} {} {}".format(*nxyz))
        origin.set("rpy", "{} {} {}".format(*nrpy))

        print "Updated {}: {}, {} -> {} {}".format(joint_name, xyz, rpy, nxyz, nrpy)

# Write the updated URDF
if shouldWrite:
    urdf.write(os.path.join(path, urdf_out))
    print "Wrote update URDF to:", urdf_out
