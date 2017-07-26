#!/usr/bin/env python

"""
Automated data collection script for Kuka eye-hand calibration with the Kinect.

This script requires Spartan Drake.

Prereqs:
* iiwa_calibration.yaml:
    Set robotRightGripper -> targetConstraintFrame to calibration_target so the IK
    solver will get it right
"""

# For robot motion
import rospy
from eyehand_calib.srv import *
from robot_msgs.srv import *

# For IK
import pydrake
from pydrake.solvers import ik

import numpy as np
import math

from datetime import datetime
import os

class EyeHandCalibrator(object):
    def __init__(self, config):
        self.config = config

        # Initialize robot
        print "Loading URDF..."
        self.robot = pydrake.rbtree.RigidBodyTree(self.config["ik_urdf"])
        print

        print "Bodies"
        print "------"
        self.bodies = dict()
        for body in range(self.robot.get_num_bodies()):
            body_name = self.robot.getBodyOrFrameName(body)
            print body_name, body
            self.bodies[body_name] = body
        # if robot.get_num_bodies() == 0: print "No bodies"
        print

        self.positions = dict()
        print "Positions"
        print "---------"
        for position in range(self.robot.get_num_positions()):
            position_name = self.robot.get_position_name(position)
            print position_name, position
            self.positions[position_name] = position
        # if robot.get_num_positions() == 0: print "No positions"
        print

        print "Waiting for getRobotConf service..."
        rospy.wait_for_service('/robot_server/getRobotConf')
        print "Waiting for GoToConf service..."
        rospy.wait_for_service('/robot_control/GoToConf')

        self.conf_service = rospy.ServiceProxy('/robot_server/getRobotConf', GetRobotConf)
        self.go_to_service = rospy.ServiceProxy('/robot_control/GoToConf', GoToConf)

        rospy.sleep(1) # cheating

    def collect_data(self, data_point, conf, cam_pose, world_pose):
        # Nasty hack here:
        conf.robotBase = conf["robotBase"]
        conf.robotRightArm = conf["robotRightArm"]
        conf.robotRightGripper = conf["robotRightGripper"]

        proxy = rospy.ServiceProxy('/capture_data', Capture)
        req = CaptureRequest(self.current_dir, data_point,
            self.config['frame_count'], conf, cam_pose, world_pose)
        resp = proxy(req)

        if resp.captured_frames != self.config['frame_count']:
            print "WARNING: Wrong number of frames captured!"

    def computePoses(self):
        # Precompute poses
        head_poses = self.computeHeadPoses()
        plate_poses = self.computePlatePoses()
        print "Total head poses:", len(head_poses)
        print "Total plate poses:", len(plate_poses)
        print

        print "Performing IK..."
        # Solve all the IK
        all_poses = list()
        for i, head_pose in enumerate(head_poses):
            for plate_pose in plate_poses:
                # Generate constraints and solve IK
                constraints = list()

                # Constrain the head pose
                head_constraint = ik.PostureConstraint( self.robot )
                head_constraint.setJointLimits(
                    np.array([self.positions['ptu_pan'], self.positions['ptu_tilt']], dtype=np.int32).reshape(2, 1),
                    head_pose - self.config['head_pose_tol'],
                    head_pose + self.config['head_pose_tol']
                )
                constraints.append(head_constraint)

                # Set the calibration plate position
                constraints.append( ik.RelativePositionConstraint ( self.robot,
                    np.zeros([3,1]),                                                    # Point relative to Body A (calibration target)
                    plate_pose - self.config['arm_pos_tol'], # Lower bound relative to Body B (optical frame)
                    plate_pose + self.config['arm_pos_tol'], # Upper bound relative to Body B (optical frame)
                    self.bodies['calibration_target'],                            # Body A
                    self.bodies[self.config['optical_frame']],                    # Body B
                    np.concatenate([                                                    # Transform from B to reference point
                        np.zeros(3),                                                    #   Translation (identity)
                        np.array([1, 0, 0, 0])                                          #   Rotation (identity)
                    ]).reshape(7, 1)
                ))

                # Set the calibration plate orientation
                constraints.append( ik.RelativeGazeDirConstraint ( self.robot,
                    self.bodies['calibration_target'],                   # Body A
                    self.bodies[self.config['optical_frame']],           # Body B
                    np.array([0, 0,  1], dtype=np.float64).reshape(3,1), # Body A axis
                    np.array([0, 0, -1], dtype=np.float64).reshape(3,1), # Body B axis
                    self.config['gaze_dir_tol']                          # Cone tolerance in radians
                ))

                # Perhaps add collision constraints here?
                # TODO: MinDistanceConstraint needed for sure
                constraints.append( ik.MinDistanceConstraint ( self.robot,
                    self.config['collision_min_distance'],   # Minimum distance between bodies
                    list(),                                  # Active bodies (empty set means all bodies)
                    set()                                    # Active collision groups (not filter groups! Empty set means all)
                ))

                # NOTE: Possibly a global position constraint to keep the arm off the ground
                constraints.append( ik.WorldPositionConstraint ( self.robot,
                    self.bodies["calibration_target"],                      # Body to constrain
                    np.array([0, 0, 0]),                                    # Point on body
                    np.array([np.nan, np.nan, self.config['min_height']]),  # Lower bound. Nan is don't care
                    np.array([np.nan, np.nan, np.nan])                      # Upper bound. Nan is don't care
                ))

                # Actually solve the IK!
                # Since we're solving ahead of time, just use the zero as seed
                # NOTE: Could possibly track last solution as seed...
                q_seed = self.robot.getZeroConfiguration()
                options = ik.IKoptions(self.robot)
                result = ik.InverseKin(self.robot, q_seed, q_seed, constraints, options)

                if result.info[0] != 1:
                    pass
                    # print "Bad result! info = {}".format(result.info[0])
                    # print "head_pose:",head_pose
                    # print "plate_pose:", plate_pose
                else:
                    # Tuple: head_pose, arm_pose
                    all_poses.append((result.q_sol[0][0:2],result.q_sol[0][2:]))
                    # print "Success!"

            # Show status info after every head pose
            attempted = (i+1) * len(plate_poses)
            success = len(all_poses)
            total = len(head_poses)*len(plate_poses)
            completion = attempted*100 / total
            print "[{:>3d}%] {}/{} solutions found".format(completion, success, attempted)

        return all_poses

    def run_test(self):
        """
        Run the data collection procedure.

        This method systematically changes the head orientaiton of the robot
        through its full range of motion. At every head pose, the arm is used to
        move a calibration plate in a cube pattern within the camera field of
        view, always keeping the plate perpendicular to the face of the camera.

        Procedure:
        1. Set up for data collection.
        2. Create list of head orientations.
        3. Create list of calibration target poses in camera frame.
        4. Solve arm IK across all locations.
        5. For each IK solution:
            a. Move robot to solution.
            b. Collect data.
        """

        # Prepare for data collection
        if self.config["collect_data"]:
            # create experiment directory
            date = datetime.utcnow()
            day_dir = date.strftime("%Y%m%d")
            exp_dir = date.strftime("%Y%m%dT%H%M%S_"+self.config["experiment_suffix"])
            data_path = '/'.join([self.config["data_root"], day_dir, exp_dir])

            print "Preparing for data collection!"
            print "Making output directory:", data_path

            # Make sure the directory exists
            os.makedirs(data_path)

        # Determine robot positions
        all_poses = self.computePoses()

        # Loop over valid configurations
        for i, (head_pose, arm_pose) in enumerate(all_poses):
            # Move robot to configuration
            print "Moving to pose {}/{}".format(i+1, len(all_poses))
            print "\thead:", head_pose
            print "\tarm: ", arm_pose
            if self.config['move_robot']: self.moveRobot(head_pose, arm_pose)

            # Collect data
            if self.config['collect_data']:
                try:
                    print "Collecting data..."
                    conf.robotHead = head_pose
                    self.collect_data(dp, conf, cam_pose, target_pose)
                    print "Success!"
                    dp += 1
                except:
                    print "Capture failed! Continuing anyway..."

    def moveRobot(self, head_pose, arm_pose):
        # Get current robot configuration
        robotConf = self.conf_service().conf

        # Update to desired pose
        robotConf.robotHead = head_pose
        robotConf.robotRightArm = arm_pose

        # Send robot to pose
        resp = self.go_to_service(GoToConfRequest(robotConf, 0.0))
        return np.concatenate([resp.conf.robotHead, resp.conf.robotRightArm])

    def computePlatePoses(self):
        # params
        near_fov = self.config["near_fov"]
        far_fov = self.config["far_fov"]
        horiz_angle = self.config["horiz_angle"]
        vert_angle = self.config["vert_angle"]
        grid_size = self.config["grid_size"]

        # Compute corners of bounding rect
        # Near-top-left
        p1x = -near_fov*math.tan(horiz_angle / 2.0)
        p1y = -near_fov*math.tan(vert_angle / 2.0)
        p1z = near_fov
        print  "Inner bounds:", p1x, p1y, p1z

        # Far-bottom-right
        p2x = -p1x
        p2y = -p1y
        p2z = far_fov
        print  "Outer bounds:", p2x, p2y, p2z

        plate_poses = list()
        for x in np.linspace(p1x, p2x, num=grid_size):
            for y in np.linspace(p1y, p2y, num=grid_size):
                for z in np.linspace(p1z, p2z, num=grid_size):
                    plate_poses.append(np.array([x, y, z], dtype=np.float64).reshape(3, 1))
        return plate_poses

    def computeHeadPoses(self):
        pans = np.linspace(self.config["pan_low"], self.config["pan_high"], num=self.config["pan_count"])
        tilts = np.linspace(self.config["tilt_low"], self.config["tilt_high"], num=self.config["tilt_count"])

        head_poses = list()
        for pan in pans:
            for tilt in tilts:
                head_poses.append(np.array([pan, tilt], dtype=np.float64).reshape(2, 1))
        return head_poses


def main():
    # Initialize ROS node
    rospy.init_node('eyehand_calib')

    config = {
        # Test control
        "collect_data": False,
        "move_robot": True,
        "save_ik": True,
        "load_ik": False,

        # Data collection
        "data_root": "/home/momap/momap_data",
        "experiment_suffix": "eyehand",
        "frame_count": 10,

        # Robot description
        "ik_urdf": "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/urdf/.full_momap_left_arm-drake.urdf",
        "optical_frame": "kinect1_rgb_optical_frame",

        # IK persistence
        "ik_save_path": "",
        "ik_load_path": "",

        # Inverse kinematics
        "head_pose_tol": 0.001,
        "arm_pos_tol": 0.01,
        "gaze_dir_tol": 0.01,
        "collision_min_distance": 0.01,
        "min_height": 0.30,

        # Head pose selection
        "pan_low": 0.0,
        "pan_high": 0.5,
        "pan_count": 3,
        "tilt_low": 0.0,
        "tilt_high": -0.5,
        "tilt_count":   3,

        # Plate pose selection
        "near_fov": 0.6,
        "far_fov": 1.2,
        "horiz_angle": 40.0 * math.pi / 180.0,
        "vert_angle": 30.0 * math.pi / 180.0,
        "grid_size": 5
    }

    tester = EyeHandCalibrator(config)
    tester.run_test()

if __name__ == "__main__":
    main()
