#!/usr/bin/env python

"""
This node will select random locations for the arm, such that the wrist camera
is always looking at the Aruco tag but from new points of view.

Ideally, this node should prioritize views we haven't been to yet, but for a
first pass random selection will do.

Requirements:
* momap_left_arm configuration
* Draper Drake / Spartan
"""

# For robot motion
import rospy
from robot_msgs.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# For Aruco processing
import tf2_ros
from tf2_geometry_msgs import PoseStamped, do_transform_pose

# For IK
import pydrake
from pydrake.solvers import ik

import numpy as np
from scipy.interpolate import interp1d, CubicSpline
import math
from eyehand_utils import query_yes_no
from track_tag import TagTracker

class Orbiter(object):
    def __init__(self, config, target):
        self.config = config
        self.target = target

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

        # Set up ROS
        print "Waiting for getRobotConf service..."
        rospy.wait_for_service('/robot_server/getRobotConf')
        self.conf_service = rospy.ServiceProxy('/robot_server/getRobotConf', GetRobotConf)

        print "Waiting for GoToConf service..."
        rospy.wait_for_service('/robot_control/GoToConf')
        self.go_to_service = rospy.ServiceProxy('/robot_control/GoToConf', GoToConf)

        print "Waiting for SendJointTrajectory service..."
        rospy.wait_for_service('/robot_control/SendJointTrajectory')
        self.traj_service = rospy.ServiceProxy('/robot_control/SendJointTrajectory', SendJointTrajectory)

        print "Waiting for MoveHead service..."
        rospy.wait_for_service('/robot_control/MoveHead')
        self.move_head_service = rospy.ServiceProxy('/robot_control/MoveHead', MoveHead)

        print "ROS stack ready!"
        print

    def generatePoses(self, max_poses):
        """
        Generates poses relative to the target to move to.

        These have 4 degrees of freedom, but we'll only constrain 3:
         * Inclination [0, pi/2)
         * Rotation (-theta, theta)
         * Distance [0, d)
         * Roll (-pi, pi) / unconstrained

        We don't constrain roll in order to find IK solutions more easily.
        """
        r = self.config["pose_ub"] - self.config["pose_lb"]
        poses =  r * np.random.random_sample([self.config["max_poses"], 3]) + self.config["pose_lb"]
        return poses

    def solveIK(self, pose, target):
        """
        pose: np.array([latitude, longitude, depth])
        target: np.array([x, y, z, quat[4]]) of view target in "world" frame
        """

        # Duration of motion for solver
        dur = self.config["trajectory_duration"]

        constraints = list()

        # Constrain the gaze
        constraints.append ( ik.WorldGazeTargetConstraint( self.robot,
            self.bodies[self.config["optical_frame"]],  # Body to constrain
            np.array([0.0, 0.0, 1.0]).reshape(3, 1),    # Gaze axis
            target[0:3, None],                          # Gaze target (world frame)
            np.zeros([3,1]),                            # Gaze origin (body frame)
            self.config["cone_threshold"]               # Cone threshold
            # np.array([dur, dur]).reshape([2,1])       # Valid times
        ))

        # Constrain the position
        # NOTE: Due to weirdness with the Aruco tag, Y is up and -Z is front
        # So, longitude rotates around Y
        #     latitude rotates around -X
        #     depth is in positive Z
        # ... This is easier just doing trig by hand
        camera_pose = np.array([
            pose[2]*math.cos(pose[0])*math.sin(pose[1]), # X
            pose[2]*math.sin(pose[0]),                   # Y
            pose[2]*math.cos(pose[0])*math.cos(pose[1])  # Z
        ])

        print "Pose:       ", pose
        print "Camera:     ", camera_pose

        constraints.append ( ik.RelativePositionConstraint( self.robot,
            np.zeros([3,1]),                                # Point relative to Body A (optical frame)
            camera_pose - self.config['pose_tol'],           # Lower bound relative to Body B (target)
            camera_pose + self.config['pose_tol'],           # Upper bound relative to Body B (target)
            self.bodies[self.config['optical_frame']],      # Body A
            self.bodies["world"],                           # Body B
            target.reshape(7, 1)                            # Transform from B to reference point
        ))

        # Avoid collisions
        # NOTE: How do we avoid colliding with the table?
        #       * Table needs to be in the RBT
        constraints.append( ik.MinDistanceConstraint ( self.robot,
            self.config['collision_min_distance'],   # Minimum distance between bodies
            list(),                                  # Active bodies (empty set means all bodies)
            set()                                    # Active collision groups (not filter groups! Empty set means all)
        ))

        # Seed with current configuration (eventually!)
        q_seed = self.robot.getZeroConfiguration()

        options = ik.IKoptions(self.robot)
        result = ik.InverseKin(self.robot, q_seed, q_seed, constraints, options)

        if result.info[0] != 1:
            print "IK Failed! Result:",result.info
            return None

        print
        return result.q_sol[0]

    def trajectoryIK(self, start, end):
        print "Moving along trajectory!"
        print "Start:", start
        print "End:",end

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
        # options.setMajorIterationsLimit(400)
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

    def moveRobot(self, head_pose, arm_pose):
        # Get current robot configuration
        robotConf = self.conf_service().conf

        # Update to desired pose
        robotConf.robotHead = head_pose
        robotConf.robotRightArm = arm_pose

        # Send robot to pose
        resp = self.go_to_service(GoToConfRequest(robotConf, 0.0))
        return np.concatenate([resp.conf.robotHead, resp.conf.robotRightArm])

    def moveRobotSafe(self, goal):
        # Get current robot configuration
        robotConf = self.conf_service().conf
        start = (robotConf.robotHead, robotConf.robotRightArm)
        end = (robotConf.robotHead, goal[2:])

        traj = self.trajectoryIK(start, end)
        if traj is None:
            print "Could not find a safe trajectory!"
        else:
            self.followTrajectory(traj)

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

    def orbit(self):
        poses = self.generatePoses(self.config["max_poses"])

        # Generate IK solutions
        for pose in poses:
            # Quat order is WXYZ
            q_sol = self.solveIK(pose, self.target)

            # Move robot between solutions
            if q_sol is not None:
                self.moveRobotSafe(q_sol)


def main():
    # Initialize ROS node
    rospy.init_node('orbiter')

    tracker_config = {
        # Tracking config
        "tag": 50,
        "tag_topic": "/aruco_tags/markers",
        "move_head": False,

        # Robot config
        "ik_urdf": "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/urdf/.momap_left_arm-drake.urdf",
        "optical_frame": "kinect1_rgb_optical_frame",

        # IK options
        "cone_threshold": 0.001,

        # Motion options
        "ptu_velocity": 2.0,
        "dead_band": 0.1
    }

    orbit_config = {
        # Robot config
        "ik_urdf": "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/urdf/.momap_left_arm-drake.urdf",
        "optical_frame": "realsense_rgb_optical_frame",

        # Pose generation
        "max_poses": 100,
        "pose_lb": np.array([0, -3*math.pi/4, 0.4]),
        "pose_ub": np.array([math.pi/2, 3*math.pi/4, 0.7]),

        # IK options
        "pose_tol": 0.001,
        "cone_threshold": 1.0 * math.pi / 180.0,
        "collision_min_distance": 0.01,
        "trajectory_count": 40,
        "trajectory_duration": 20,
    }

    # Use tracker to get initial location
    print "Using TagTracker to select initial position"
    tracker = TagTracker(tracker_config)

    yep = True
    target = None
    while yep:
        target = tracker.track_for(1)
        print "Target:", target
        yep = query_yes_no("Track again?")

    orbiter = Orbiter(orbit_config, target)
    raw_input("Press [Enter] when ready to orbit!")
    orbiter.orbit()

if __name__=="__main__":
    main()
