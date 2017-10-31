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

#from robot_prims.primitive_utils import *
from perception_interfaces.utils.transform_helper import *

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped, do_transform_pose
from eye_hand_calibration.srv import *
from robot_msgs.srv import *

import pydrake
from pydrake.solvers import ik

import numpy as np
import math
import yaml

from datetime import datetime
import os

class TargetSweepExperiment(object):
    def __init__(self, config):
        self.config = config
        self.count = 0

        # Initialize robot
        # This is a hack to get named access to URDF parts
        robot = pydrake.rbtree.RigidBodyTree(config["urdf"])
        bodies = dict()
        for body in range(robot.get_num_bodies()):
            body_name = robot.getBodyOrFrameName(body)
            bodies[body_name] = body
        robot.bodies = bodies
        self.robot = robot

        # Initialize TF2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Set up services to move robot
        rospy.wait_for_service('/robot_server/getRobotConf')
        rospy.wait_for_service('/robot_control/GoToConf')
        self.conf_service = rospy.ServiceProxy('/robot_server/getRobotConf', GetRobotConf)
        self.go_to_service = rospy.ServiceProxy('/robot_control/GoToConf', GoToConf)


        # Read motion config
        self.config['motion'] = yaml.load_all(open(config["motion_config"])).next()

        rospy.sleep(1) # cheating

    def moveArm(self,conf, duration=7.0):
        # Get current robot configuration
        robotConf = self.conf_service().conf

        # Update to desired pose
        robotConf.robotRightArm = conf

        # Send robot to pose
        resp = self.go_to_service(GoToConfRequest(robotConf, duration))
        return np.array(resp.conf.robotRightArm)

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

            # Change direction with each pass
            y_range = direction*np.linspace(y["min"], y["max"], num=step_count) * math.pi / 180.0
            direction *= -1

            # Generate list of target poses
            pose_mats = list()
            for rot_y in y_range:
                rot_z = 0.0
                pose_mats.append(self.computeCameraPose(rot_x, rot_y, rot_z, tran_z))

                # Slow down for debugging
                rospy.sleep(0.2)

            # Solve IK
            for pose_mat in pose_mats:
                conf, success = self.runIK(pose_mat)
                if success:
                    print("IK Solved!", conf, sep='\n')
                    confs.append(conf)
                else:
                    print ("Infeasible pose: ({}, {}, {}, {})".format(rot_x, rot_y, rot_z, tran_z))

        if self.config['move_robot']:
            for conf in confs:
                self.moveArm(conf)
                self.collect_data()

    def runIK(self, pose_mat):
        # Set up constraints
        constraints = list()

        xyz = transf.translation_from_matrix(pose_mat)
        pos_delta = self.config["pose_delta"]
        constraints.append( ik.WorldPositionConstraint( self.robot,
            self.robot.bodies["iiwa_tool_frame"], # Frame of reference
            np.array([0.0, 0.0, 0.0]), # Point to constrain (in frame of reference)
            xyz - pos_delta,  # Lower bound
            xyz + pos_delta   # Upper bound
        ))

        rpy = np.array(transf.euler_from_matrix(pose_mat))
        quat_tol = self.config["quat_tol"]
        constraints.append( ik.WorldEulerConstraint( self.robot,
            self.robot.bodies["iiwa_tool_frame"], # Frame of reference
            rpy - quat_tol, # Lower bound
            rpy + quat_tol  # Upper bound
        ))

        # Run the IK
        q_seed = self.robot.getZeroConfiguration()
        options = ik.IKoptions(self.robot)

        result = ik.InverseKin(self.robot, q_seed, q_seed, constraints, options)

        if result.info[0] == 1:
            return result.q_sol[0][2:], True
        else:
            return [], False

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
        # Define frame to look at
        look_at = np.concatenate([
            self.config["look_at_frame"][:3],
            transf.quaternion_from_euler(*self.config["look_at_frame"][3:])
        ])
        time = rospy.Time.now()
        temp = create_pose_stamp_msg("look_at", look_at)
        broadcast_pose(self.tfBuffer, "world", "look_at", temp, time, private=True)

        # Define rotated frame (intermediate)
        body_to_camera = transf.quaternion_from_euler(-math.pi / 2.0, math.pi / 2.0, 0, axes="rxyz")
        #view_angle = transf.quaternion_from_euler(rot_y, rot_x, rot_z, axes='ryxz')
        #quat = transf.quaternion_multiply(body_to_camera, view_angle)

        # Testing something here
        view_angle = transf.quaternion_from_euler(rot_y, -rot_x, rot_z, axes='rzyx')

        inter = create_pose_stamp_msg("inter", np.concatenate([[0,0,0], view_angle]))
        broadcast_pose(self.tfBuffer, "look_at", "look_at_rot", inter, time, private=True)

        # Define target camera pose
        target = geometry_msgs.msg.PoseStamped()
        target.header.stamp = time
        target.header.frame_id = "look_at_rot"
        #target.pose.position.z = tran_z
        target.pose.position.x = tran_z
        target.pose.orientation.w = 1

        # Transform camera pose into world frame for IK
        xform = self.tfBuffer.lookup_transform("world", "look_at_rot", rospy.Time(0), rospy.Duration(0.1))
        target_pose = do_transform_pose(target, xform)

        # Show the resulting frame for debug
        broadcast_pose(self.broadcaster, "world", "eyehand_{}".format(self.count), target_pose, rospy.Time.now())
        self.count += 1
        target_mat = pose_stamp_to_tf(target_pose)

        return target_mat

def broadcast(b, parent, child, xyz, angle, time, euler=True, private=False, ):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = time
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
        #print("Sending transform {} -> {}: Private".format(parent, child))
        b.set_transform(t, "PRIVATE")
    else:
        #print("Sending transform {} -> {}: Public".format(parent, child))
        b.sendTransform(t)

def broadcast_pose(broadcaster, parent, child, pose, time, private=False):
    p = pose.pose.position
    q = pose.pose.orientation
    broadcast(broadcaster, parent, child, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], time, euler=False, private=private)

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
        "urdf": "/home/momap/momap/src/robot_core/robot_descriptions/robot_description/urdf/draper_ptu_gripper.urdf",

        # Where are we circling?
        "look_at_frame": [1.0,0,0.75,0,0,0],     # [x,y,z,r,p,y] from world

        # IK parameters
        "pose_delta": 0.001,
        "quat_tol": 0.001,

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
