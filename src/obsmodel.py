#!/usr/bin/env python

"""
Automated data collection script for observation model. Script moves the KUKA
around to build up a dense pointcloud of a target object with the wrist camera.

Prereqs:
* iiwa_calibration.yaml:
    Set robotRightGripper -> targetConstraintFrame to iiwa_tool_frame so the IK
    solver will get it right
"""
from __future__ import print_function

from robot_prims.primitive_utils import *
from perception_interfaces.utils.transform_helper import *

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped, do_transform_pose
from eyehand_calib.srv import *

import numpy as np
import math
import yaml

from datetime import datetime
import os

class TargetSweepExperiment(object):
    def __init__(self, config):
        self.config = config

        # Initialize robot
        self.robot_handle = create_robot_handle()

        # Initialize TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Read motion config
        self.config['motion'] = yaml.load_all(open(config["motion_config"])).next()

        rospy.sleep(1) # cheating

    def prepare_data_collection(self):
        if self.config["collect_data"]:
            # create experiment directory
            date = datetime.utcnow()
            day_dir = date.strftime("%Y%m%d")
            exp_dir = date.strftime("%Y%m%dT%H%M%S_targetsweep")
            data_path = '/'.join([self.config["data_root"], day_dir, exp_dir])

            # Make sure the directory exists
            os.makedirs(data_path)

    def collect_data(self):
        if not self.config["collect_data"]: return

        # Not yet sure what this will need to do. Data collection is happening
        # in LCM instead of ROS, so it may just do nothing
        pass

    def run_test(self):
        # Set up data collection directory
        self.prepare_data_collection()

        confs = list()
        direction = 1
        for frame in self.config["motion"]["frames"]:
            rot_x = frame["rotateX"] * math.pi / 180.0
            y = frame["rotateY"]
            tran_z = frame["translateZ"]
            step_count = frame["numFrames"]

            # All candidate z rotations.
            # This is a shim for not constraining the IK properly
            all_z = np.linspace(0, 2*math.pi, endpoint=False)

            # Change direction with each pass
            y_range = direction*np.linspace(y["min"], y["max"], num=step_count) * math.pi / 180.0
            direction *= -1

            for rot_y in y_range:
                rospy.sleep(0.2)
                for rot_z in all_z:
                    pose_mat = self.computeCameraPose(rot_x, rot_y, rot_z, tran_z)

                    # Now, get a matrix from the pose
                    conf, err = obtainConfViaIK(self.robot_handle, pose_mat)
                    if err != 1:
                        print("IK Solved!", conf, sep='\n')
                        confs.append(conf)
                        break

        if self.config['move_robot']:
            for conf in confs:
                sendRobotToConf(conf, 'move')
                self.collect_data()

    def fuzz_target_pose(self, target_rpy):
        if not self.config['should_fuzz']: return [target_rpy]
        result = list()
        for i in np.linspace(0, 2*math.pi):
            fuzzed = list(target_rpy)
            fuzzed[2] += i
            result.append(fuzzed)
        return result

    def computeCameraPose(self, rot_x, rot_y, rot_z, tran_z):
        """
        Returns the target pose of the camera for a specified view angle and distance

        The strategy
        ------------
        1. Assumes look_at frame is known beforehand
           * look_at is specified according to REP 103
        2. Define intermediate frame: look_at_rot
           * Rotation only
           * Composition of two rotations:
              1. Body to optical frame
              2. View angle
        3. Specify desired camera pose in the rotated frame
        4. Transform camera pose into world frame

        rot_x: tilt angle in radians
        rot_y: pan angle in radians
        rot_z: roll angle in radians
        tran_z: distance of camera from object origin

        Returns a PoseStamped in "world" frame with the current time stamp.
        """
        body_to_camera = transf.quaternion_from_euler(-math.pi / 2.0, math.pi / 2.0, 0, axes="rxyz")
        view_angle = transf.quaternion_from_euler(rot_y, rot_x, rot_z, axes='ryxz')
        quat = transf.quaternion_multiply(body_to_camera, view_angle)

        # Define rotated frame (intermediate)
        inter = create_pose_stamp_msg("inter", np.concatenate([[0,0,0], quat]))
        broadcast_pose(self.tfBuffer, "look_at", "look_at_rot", inter, private=True)

        # Define target camera pose
        target = geometry_msgs.msg.PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "look_at_rot"
        target.pose.position.z = tran_z
        target.pose.orientation.w = 1

        # Transform camera pose into world frame for IK
        xform = self.tfBuffer.lookup_transform("world", "look_at_rot", rospy.Time(0), rospy.Duration(0.1))
        target_pose = do_transform_pose(target, xform)

        # Show the resulting frame for debug
        broadcast_pose(self.broadcaster, "world", "eyehand_1", target_pose)
        target_mat = pose_stamp_to_tf(target_pose)

        return target_mat

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
            target_quat = transf.quaternion_from_euler(*fuzz_rpy, axes='sxyz')
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

            if self.config['move_robot']:
                sendRobotToConf(conf, 'move')

            return True, conf, cam_pose, target_pose
        print("IK failed")
        return False, {}, None, None

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
        optical_frame = self.config["optical_frame"]

        # Compute corners of bounding rect

        # Near-top-right
        p1x = near_fov*math.tan(horiz_angle / 2.0)
        p1y = -near_fov*math.tan(vert_angle / 2.0)
        p1z = near_fov
        print ("Bounds:", p1x, p1y, p1z)

        # Far-bottom-left
        p2x = -p1x
        p2y = -p1y
        p2z = far_fov

        # Get Kuka base link in camera coords
        xform = self.tfBuffer.lookup_transform(optical_frame, "iiwa_link_0", rospy.Time.now(), rospy.Duration(1.0))
        t = xform.transform.translation
        kuka_base = np.array([t.x, t.y, t.z])
        print("Kuka base pose:", kuka_base)

        for x in np.linspace(p1x, p2x, num=grid_size):
            for y in np.linspace(p1y, p2y, num=grid_size):
                for z in np.linspace(p1z, p2z, num=grid_size):
                    p = np.array([x, y, z])
                    dist = np.linalg.norm(p - kuka_base)
                    if dist < kuka_reach:
                        rospy.sleep(1)
                        yield p

    def head_poses(self):
        pans = np.linspace(self.config["pan_low"], self.config["pan_high"], num=self.config["pan_count"])
        tilts = np.linspace(self.config["tilt_low"], self.config["tilt_high"], num=self.config["tilt_count"])
        for pan in pans:
            for tilt in tilts:
                yield [pan, tilt]


def broadcast(b, parent, child, xyz, angle, euler=True, private=False):
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

    if private:
        print("Sending transform {} -> {}: Private".format(parent, child))
        b.set_transform(t, "PRIVATE")
    else:
        print("Sending transform {} -> {}: Public".format(parent, child))
        b.sendTransform(t)

def broadcast_pose(broadcaster, parent, child, pose, private=False):
    p = pose.pose.position
    q = pose.pose.orientation
    broadcast(broadcaster, parent, child, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], euler=False, private=private)

# Random notes from earlier days...
# Camera at [0, 0]
# Near-top-left: [0.6, -0.15, 0.45]
# Far-bottom-right: [0.90, 0.45, -0.35]
#
# !! Self collision: [1.00, 0.50, -0.25]

def main():
    # Initialize ROS node
    rospy.init_node('eyehand_calib')

    # Generate full directory name

    config = {
        # Test parametes
        "data_root": "/home/momap/momap_data",
        "frame_count": 10,
        "collect_data": False,
        "move_robot": True,
        "should_fuzz": True,
        "motion_config": "/home/momap/spartan/src/CorlDev/config/data_collection.yaml",

        # Optical frame
        "look_at_frame": "target_sweep_look_at_frame",

        # Sweep pattern
        "kuka_reach": 1.2,
        "near_fov": 0.6,
        "far_fov": 0.9,
        "horiz_angle": 40.0 * math.pi / 180.0,
        "vert_angle": 25.0 * math.pi / 180.0,
        "grid_size": 4
    }

    tester = TargetSweepExperiment(config)
    # angle = [0, -math.pi / 2.0, 0]
    # pos =  [1.0, 0.60, -0.00]
    # tester.move_head([0, 0])
    # tester.move_to_pose(pos, angle)
    tester.run_test()

if __name__ == "__main__":
    main()
