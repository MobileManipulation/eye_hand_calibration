#!/usr/bin/env python

"""
Automated data collection script for Kuka eye-hand calibration with the Kinect.

Prereqs:
* iiwa_calibration.yaml:
    Set robotRightGripper -> targetConstraintFrame to calibration_target so the IK
    solver will get it right
"""

from __future__ import print_function

from robot_prims.primitive_utils import *
from perception_interfaces.utils.transform_helper import *

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped, do_transform_pose

import numpy as np
import math

class EyeHandCalibrator(object):
    def __init__(self, config):
        self.config = config

        # Initialize robot
        self.robot_handle = create_robot_handle()

        # Initialize TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to necessary data channels
        # Raw camera data: /kinect1/rgb/image_rect_color
        # Camera pointcloud: /kinect1/depth_registered/points
        # Robot JointState: /joint_states
        # AR Tags: UNKNOWN! How to launch?

        rospy.sleep(1) # cheating

    def run_test(self):
        # Face the camera
        angle = [0, -math.pi / 2.0, 0]
        for head_pose in self.head_poses():
            self.move_head(head_pose)
            for pos in self.viewspace_samples():
                print("Moving to pose:", pos)
                self.move_to_pose(pos, angle)

    def fuzz_target_pose(self, target_rpy):
        result = list()
        for i in np.linspace(0, 2*math.pi):
            fuzzed = list(target_rpy)
            fuzzed[2] += i
            result.append(fuzzed)
        return result

    def move_to_pose(self, target_xyz, target_rpy):
        """
        Move calibration plate to specified position. target given in camera coordinates.
        """
        optical_frame = self.config["optical_frame"]

        # Generate fuzzed positions
        fuzz = self.fuzz_target_pose(target_rpy)
        print(fuzz)

        attempts = 0
        for fuzz_rpy in fuzz:
            attempts += 1
            print("Attempt", attempts)

            # Convert to quaternion
            target_quat = transf.quaternion_from_euler(*fuzz_rpy, axes='szyx')
            cam_pose = create_pose_stamp_msg("cam_pose", np.concatenate([target_xyz, target_quat]))
            broadcast_pose(self.broadcaster, optical_frame, "eyehand_1", cam_pose)

            # Transform camera frame pose to IK frame (world, according to Jay)
            xform = self.tfBuffer.lookup_transform("world", optical_frame, rospy.Time.now(), rospy.Duration(1.0))
            target_pose = do_transform_pose(cam_pose, xform)

            # print("Cam pose:", cam_pose, sep='\n')
            # print("Target pose:", target_pose, sep='\n')
            broadcast_pose(self.broadcaster, "world", "eyehand_2", target_pose)

            target_mat = pose_stamp_to_tf(target_pose)
            # print ("Target mat", target_mat, sep='\n')

            # Now, get a matrix from the pose
            conf, err = obtainConfViaIK(self.robot_handle, target_mat)
            if err != 1:
                continue

            print("IK Solved!", conf, sep='\n')
            sendRobotToConf(conf, 'move')
            return
        print("IK failed")

    def move_head(self, head_pose):
        sendRobotToConf({'robotHead': head_pose}, 'look', duration=5.0)

    def viewspace_samples(self):
        """
        Expected ranges:
            pan: -0.5 -> 0.5 (right to left)
            tilt: 0 -> -0.7 (straight to down)
        """
        # params
        kuka_reach = self.config["kuka_reach"]
        near_fov = self.config["near_fov"]
        far_fov = self.config["far_fov"]
        horiz_angle = self.config["horiz_angle"]
        vert_angle = self.config["vert_angle"]
        grid_size = self.config["grid_size"]

        # Compute corners of bounding rect

        # Near-top-left
        p1z = near_fov*math.tan(horiz_angle / 2.0)
        p1y = -near_fov*math.tan(vert_angle / 2.0)
        p1x = near_fov

        # Far-bottom-right
        p2z = -p1z
        p2y = -p1y
        p2x = far_fov

        # Get Kuka base link in camera coords
        xform = self.tfBuffer.lookup_transform("kinect1_optical_link", "iiwa_link_0", rospy.Time.now(), rospy.Duration(1.0))
        t = xform.transform.translation
        kuka_base = np.array([t.x, t.y, t.z])
        print("Kuka base pose:", kuka_base)

        for x in np.linspace(p1x, p2x, num=grid_size):
            for y in np.linspace(p1y, p2y, num=grid_size):
                for z in np.linspace(p1z, p2z, num=grid_size):
                    p = np.array([x, y, z])
                    dist = np.linalg.norm(p - kuka_base)
                    if dist < kuka_reach:
                        yield p

    def head_poses(self):
        pans = np.linspace(self.config["pan_low"], self.config["pan_high"], num=self.config["pan_count"])
        tilts = np.linspace(self.config["tilt_low"], self.config["tilt_high"], num=self.config["tilt_count"])
        for pan in pans:
            for tilt in tilts:
                yield [pan, tilt]


def broadcast(broadcaster, parent, child, xyz, angle, euler=True):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = xyz[0]
    t.transform.translation.y = xyz[1]
    t.transform.translation.z = xyz[2]
    q = transf.quaternion_from_euler(*angle, axes="sxyz") if euler else angle
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    broadcaster.sendTransform(t)

def broadcast_pose(broadcaster, parent, child, pose):
    p = pose.pose.position
    q = pose.pose.orientation
    broadcast(broadcaster, parent, child, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], euler=False)

# Random notes from earlier days...
# Camera at [0, 0]
# Near-top-left: [0.6, -0.15, 0.45]
# Far-bottom-right: [0.90, 0.45, -0.35]
#
# !! Self collision: [1.00, 0.50, -0.25]

def main():
    # Initialize ROS node
    rospy.init_node('eyehand_calib')

    config = {
        # Optical frame
        "optical_frame": "kinect1_optical_link",
        "kuka_reach_frame": "iiwa_link_0",

        # Head pose selection
        "pan_low": -0.5,
        "pan_high": 0.5,
        "pan_count": 3,
        "tilt_low": 0.0,
        "tilt_high": -0.7,
        "tilt_count":   3,

        # Target pattern
        "kuka_reach": 1.2,
        "near_fov": 0.5,
        "far_fov": 0.9,
        "horiz_angle": 55.0 * math.pi / 180.0,
        "vert_angle": 40.0 * math.pi / 180.0,
        "grid_size": 2
    }

    tester = EyeHandCalibrator(config)
    angle = [0, -math.pi / 2.0, 0]
    pos =  [1.0, 0.60, -0.00]
    tester.move_head([0, 0])
    tester.move_to_pose(pos, angle)
    #tester.run_test()

if __name__ == "__main__":
    main()
