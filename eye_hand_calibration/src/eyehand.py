#!/usr/bin/env python

"""
Automated data collection script for Kuka eye-hand calibration with the Kinect.

This script requires Spartan / Draper Drake.
"""

# For robot motion
import rospy
from eyehand_calib.srv import *
from robot_msgs.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint

# For IK
import pydrake
from pydrake.solvers import ik

import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import math

from datetime import datetime
from itertools import izip, tee
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
        self.joint_names = list()
        print "Positions"
        print "---------"
        for position in range(self.robot.get_num_positions()):
            position_name = self.robot.get_position_name(position)
            print position_name, position
            self.positions[position_name] = position
            self.joint_names.append(position_name)
        # if robot.get_num_positions() == 0: print "No positions"
        print

        print "Waiting for getRobotConf service..."
        rospy.wait_for_service('/robot_server/getRobotConf')
        print "Waiting for GoToConf service..."
        rospy.wait_for_service('/robot_control/GoToConf')
        print "Waiting for SendJointTrajectory service..."
        rospy.wait_for_service('/robot_control/SendJointTrajectory')
        print "Waiting for MoveHead service..."
        rospy.wait_for_service('/robot_control/MoveHead')
        print "Waiting for Capture service..."
        rospy.wait_for_service('/capture_data')

        print "ROS stack ready!"
        print

        self.conf_service = rospy.ServiceProxy('/robot_server/getRobotConf', GetRobotConf)
        self.go_to_service = rospy.ServiceProxy('/robot_control/GoToConf', GoToConf)
        self.traj_service = rospy.ServiceProxy('/robot_control/SendJointTrajectory', SendJointTrajectory)
        self.move_head_service = rospy.ServiceProxy('/robot_control/MoveHead', MoveHead)
        self.collect_data_service = rospy.ServiceProxy('/capture_data', Capture)

        rospy.sleep(1) # cheating

    def collect_data(self, out_dir, arm_pose_idx, target_conf):
        # Nasty hack here:
        req = CaptureRequest(
            out_dir,
            arm_pose_idx,
            self.config['frame_count'],
            target_conf
        )
        resp = self.collect_data_service(req)

        if resp.captured_frames != self.config['frame_count']:
            print "WARNING: Wrong number of frames captured!"

    def getAllPoses(self):
        if self.config["load_ik"]:
            try:
                return self.loadPoses()
            except:
                print "Loading poses failed!"
                print
        return self.computePoses()

    def loadPoses(self):
        filename = self.config["ik_load_path"]
        print "Loading poses from {}".format(filename)

        all_poses = list()
        with open(filename, "r") as fp:
            data = np.loadtxt(fp, delimiter=",")
            for pose in data:
                all_poses.append((pose[0:2], pose[2:]))
            print "Loaded {} poses!".format(len(all_poses))
            print
        return all_poses

    def computePoses(self):
        # Precompute poses
        head_poses = self.computeHeadPoses()
        plate_poses = self.computePlatePoses()
        print "Total head poses:", len(head_poses)
        print "Total plate poses:", len(plate_poses)
        print

        if self.config["save_ik"]:
            filename = self.config["ik_save_path"]
            print "IK save path: {}".format(filename)
            if os.path.isfile(filename):
                self.config["save_ik"] = False
                print "WARNING! IK save path already exists. IK saving disabled"
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

                # Avoid self-collision
                constraints.append( ik.MinDistanceConstraint ( self.robot,
                    self.config['collision_min_distance'],   # Minimum distance between bodies
                    list(),                                  # Active bodies (empty set means all bodies)
                    set()                                    # Active collision groups (not filter groups! Empty set means all)
                ))

                # Actually solve the IK!
                # Since we're solving ahead of time, just use the zero as seed
                # NOTE: Could possibly track last solution as seed...
                q_seed = self.robot.getZeroConfiguration()
                options = ik.IKoptions(self.robot)
                result = ik.InverseKin(self.robot, q_seed, q_seed, constraints, options)

                if result.info[0] != 1:
                    pass
                else:
                    # Tuple: head_pose, arm_pose
                    all_poses.append((result.q_sol[0][0:2],result.q_sol[0][2:]))
                    if self.config["save_ik"]:
                        with open(self.config["ik_save_path"],'a') as fp:
                            np.savetxt(fp, result.q_sol[0][None, :], delimiter=',')

            # Show status info after every head pose
            attempted = (i+1) * len(plate_poses)
            success = len(all_poses)
            total = len(head_poses)*len(plate_poses)
            completion = attempted*100 / total
            print "[{:>3d}%] {}/{} solutions found".format(completion, success, attempted)

        return all_poses

    def computeHeadPoses(self):
        pans = np.linspace(self.config["pan_low"], self.config["pan_high"], num=self.config["pan_count"])
        tilts = np.linspace(self.config["tilt_low"], self.config["tilt_high"], num=self.config["tilt_count"])

        head_poses = list()
        for pan in pans:
            for tilt in tilts:
                head_poses.append(np.array([pan, tilt], dtype=np.float64).reshape(2, 1))
        return head_poses

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

    def getTrajectories(self):
        if self.config["load_traj"]:
            try:
                return self.loadTrajectories()
            except Exception as e:
                print "Loading trajectories failed!"
                print e
                print

        all_poses = self.getAllPoses()
        return self.computeTrajectories(all_poses)

    def loadTrajectories(self):
        filename = self.config["traj_load_path"]
        print "Loading trajectories from {}".format(filename)

        # FIXME: Proper data format
        all_traj = list()
        with open(filename, 'rb') as fp:
            for i in range(self.config["load_traj_count"]):
                all_traj.append(np.load(fp))

        # Throw out the poses we don't care about
        all_traj = all_traj[self.config["start_pose"]:]

        # NOTE: At this point, all_traj[0] is invalid, because we don't know
        # if the current pose of the robot is the same as the inital pose for
        # the saved trajectories. To fix this, we read the current pose and
        # solve for the first trajectory. From that point, we can use the rest
        # of the solutions without re-solving.

        # Get current pose and solve for for initial trajectory
        init_conf = self.conf_service().conf
        init_head = np.array(init_conf.robotHead)
        init_arm = np.array(init_conf.robotRightArm)
        start = (init_head, init_arm)
        # print "start:", start

        # Grab the start position of the second loaded trajectory
        # This will be the end position of the new initial trajectory
        end = (all_traj[1][0, 1:],)
        # print "end:", end

        # Solve for the new initial trajectory
        print "Computing inital trajectory..."
        q_sol = self.trajectoryIK(start, end)
        if q_sol is None:
            print "Could not find safe initial trajectory! Abort."
            return []
        else:
            all_traj[0] = q_sol
            return all_traj

    def computeTrajectories(self, all_poses):
        def pairwise(it):
            "s -> (s0, s1), (s1, s2), ..."
            a, b = tee(it)
            next(b, None)
            return izip(a, b)

        # First, get current pose and prepend all_poses
        init_conf = self.conf_service().conf
        init_head = np.array(init_conf.robotHead)
        init_arm = np.array(init_conf.robotRightArm)
        print "Current robot configuration:"
        print "\thead:", init_head
        print "\tarm: ", init_arm
        print

        if self.config["save_traj"]:
            filename = self.config["traj_save_path"]
            print "Trajectory save path: {}".format(filename)
            if os.path.isfile(filename):
                self.config["save_traj"] = False
                print "WARNING! Trajectory save path already exists. Saving disabled"
                print

        print "Computing safe trajectories..."
        poses = [(init_head, init_arm)] + all_poses # Intentional copy
        all_traj = list()
        last_good_start = None
        for i, (start, end) in enumerate(pairwise(poses)):
            print "Trajectory {}/{}".format(i+1, len(poses)-1)
            # print start
            # print end

            # If the last one failed, use the previous start pose
            if last_good_start is not None:
                start = last_good_start
                last_good_start = None

            q_sol = self.trajectoryIK(start, end)

            if q_sol is None:
                # Hold on to the start position if we fail
                last_good_start = start
                pass
            else:
                all_traj.append(q_sol)

                if self.config["save_traj"]:
                    with open(self.config["traj_save_path"],'ab') as fp:
                        np.save(fp, q_sol)
        print
        return all_traj

    def trajectoryIK(self, start, end):

        dur = self.config["trajectory_duration"]

        joints = np.array(
            [self.positions[joint] for joint in self.joint_names],
            dtype=np.int32
        ).reshape(9, 1)

        constraints = list()
        # Constrain the start position
        start_pose = np.concatenate(start).reshape(9, 1)
        start_constraint = ik.PostureConstraint( self.robot,
            np.array([0.0, 0.0]).reshape([2,1])    # Active time
        )
        start_constraint.setJointLimits(
            joints,
            start_pose - self.config['pose_tol'],
            start_pose + self.config['pose_tol']
        )
        constraints.append(start_constraint)

        # Constrain the end position
        end_pose = np.concatenate(end).reshape(9, 1)
        end_constraint = ik.PostureConstraint( self.robot,
            np.array([dur, dur]).reshape([2,1])    # Active time
        )
        end_constraint.setJointLimits(
            joints,
            end_pose - self.config['pose_tol'],
            end_pose + self.config['pose_tol']
        )
        constraints.append(end_constraint)

        # Constrain against collisions
        constraints.append( ik.MinDistanceConstraint ( self.robot,
            self.config['collision_min_distance'],   # Minimum distance between bodies
            list(),                                  # Active bodies (empty set means all bodies)
            set()                                    # Active collision groups (not filter groups! Empty set means all)
        ))

        # Prepare times of interest for trajectory solve
        times = np.linspace(0, dur, num=self.config['trajectory_count'], endpoint=True)

        # Compute cubic interpolation as seed trajectory
        x = np.array([0, dur])
        y = np.hstack([start_pose, end_pose])
        f = CubicSpline(x, y, axis=1, bc_type='clamped')
        q_seed = f(times)
        # print "q_seed:", q_seed

        # Solve the IK problem
        options = ik.IKoptions(self.robot)
        options.setMajorIterationsLimit(400)
        options.setFixInitialState(True)
        # print "Iterations limit:", options.getMajorIterationsLimit()
        result = ik.InverseKinTraj(self.robot, times, q_seed, q_seed, constraints, options)

        if result.info[0] != 1:
            print "Could not find safe trajectory!"
            print "Start pose:", start_pose.flatten()
            print "End pose:  ", end_pose.flatten()
            print "Result:", result.info
            return None
        else:
            # Clean up: Pack solutions as a matrix, with time as first column
            q_sol = np.vstack(result.q_sol)
            q_sol = np.insert(q_sol, [0], times[:, None], axis=1)

            return q_sol

    def followTrajectory(self, traj):
        # Generate ROS trajectory message
        # Don't send head pose! plan_runner freaks out
        msg = SendJointTrajectoryRequest()
        msg.trajectory.header.stamp = rospy.Time.now()
        msg.trajectory.header.seq = 0
        msg.trajectory.joint_names = self.joint_names[2:]
        for sol in traj:
            # print "sol:", sol
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(sol[0])
            point.positions = sol[3:]
            point.velocities = [0.0]*len(point.positions)
            point.accelerations = [0.0]*len(point.positions)
            point.effort = [0.0]*len(point.positions)
            msg.trajectory.points.append(point)

        # Generate ROS head message
        # All the head poses are the same -- just use the last one
        # print "Moving head to pose:", traj[-1, 1:3]
        head_msg = MoveHeadRequest(traj[-1, 1:3])
        self.move_head_service(head_msg)

        # Call SendTrajectory service
        self.traj_service(msg)

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
            self.data_path = '/'.join([self.config["data_root"], day_dir, exp_dir])

            print "Preparing for data collection!"
            print "Output directory:", self.data_path

            # Make sure the directory exists
            os.makedirs(self.data_path)

        # Trajectory shape: (knots, joints+1)
        # Each row is a knot along the trajectory
        # Columns:
        #   0: Time
        #   1-2: Head Pan/Tilt
        #   3-8: IIWA Joints 1-7
        trajectories = self.getTrajectories()

        if not self.config["move_robot"]: return

        for i, traj in enumerate(trajectories):
            print "Following trajectory {}/{}!".format(i+1, len(trajectories))
            self.followTrajectory(traj)

            rospy.sleep(self.config["pause_duration"])

            # Collect data
            if self.config['collect_data']:
                try:
                    print "Collecting data..."
                    # collect_data(self, out_dir, pose_idx, target_conf):
                    self.collect_data(self.data_path, i, traj[-1, 1:])
                    print "Success!"
                except Exception as e:
                    print "Capture failed! Continuing anyway..."
                    print e

        return


    def moveRobot(self, head_pose, arm_pose):
        # Get current robot configuration
        robotConf = self.conf_service().conf

        # Update to desired pose
        robotConf.robotHead = head_pose
        robotConf.robotRightArm = arm_pose

        # Send robot to pose
        resp = self.go_to_service(GoToConfRequest(robotConf, 0.0))
        return np.concatenate([resp.conf.robotHead, resp.conf.robotRightArm])


def main():
    # Initialize ROS node
    rospy.init_node('eyehand_calib')

    config = {
        # Test control
        "collect_data": True,
        "move_robot": True,
        "save_ik": False,
        "load_ik": True,
        "save_traj": False,
        "load_traj": True,
        "load_traj_count": 61,
        "start_pose": 0,
        "pause_duration": 1.0,

        # Data collection
        "data_root": "/home/momap/momap_data/log_robot",
        "experiment_suffix": "kinect1_calib_rgb",
        "frame_count": 10,

        # Robot description
        "ik_urdf": "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/urdf/.momap_left_arm-drake.urdf",
        "optical_frame": "kinect1_rgb_optical_frame",

        # IK persistence
        "ik_save_path": "full_calib_ik.save",
        "ik_load_path": "full_calib_ik.save",
        "traj_save_path": "full_calib_traj.save",
        "traj_load_path": "full_calib_traj.save",

        # Inverse kinematics
        "head_pose_tol": 0.001,
        "arm_pos_tol": 0.01,
        "gaze_dir_tol": 0.01,
        "pose_tol": 0.001,
        "collision_min_distance": 0.01,
        "min_height": 0.30,
        "trajectory_count": 60,
        "trajectory_duration": 30,

        # Head pose selection
        "pan_low": 1.57,
        "pan_high": 1.57,
        "pan_count": 1,
        "tilt_low": -0.3,
        "tilt_high": -0.7,
        "tilt_count":   1,

        # Plate pose selection
        "near_fov": 0.6,
        "far_fov": 1.2,
        "horiz_angle": 35.0 * math.pi / 180.0,
        "vert_angle": 25.0 * math.pi / 180.0,
        "grid_size": 5
    }

    tester = EyeHandCalibrator(config)
    tester.run_test()

if __name__ == "__main__":
    main()
