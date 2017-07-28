#!/usr/bin/env python

"""
Script for tracking an Aruco tag using the head camera.

Requirements:
* momap_left_arm configuration
* Draper Drake / Spartan
"""

# For robot motion
import rospy
from robot_msgs.srv import *
from sensor_msgs.msg import JointState

# For Aruco processing
from aruco_msgs.msg import MarkerArray
import tf2_ros
from tf2_geometry_msgs import PoseStamped, do_transform_pose

# For IK
import pydrake
from pydrake.solvers import ik

import numpy as np

class TagTracker(object):
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

        # Set up ROS
        print "Waiting for getRobotConf service..."
        rospy.wait_for_service('/robot_server/getRobotConf')
        self.conf_service = rospy.ServiceProxy('/robot_server/getRobotConf', GetRobotConf)

        print "Waiting for MoveHead service..."
        rospy.wait_for_service('/robot_control/MoveHead')
        self.move_head_service = rospy.ServiceProxy('/robot_control/MoveHead', MoveHead)

        print "Initializing publishers and subscribers..."
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        print "ROS stack ready!"
        print

    def markerToWorldFrame(self, marker):
        # Massage the data into a reasonable format
        p = PoseStamped()
        p.header = marker.header
        p.pose = marker.pose.pose

        # Transform pose to world frame (ROS)
        transform = self.tf_buffer.lookup_transform("world",
                        p.header.frame_id, #source frame
                        rospy.Time(0),
                        rospy.Duration(1.0)) #wait for 1 second
        pose_transformed = do_transform_pose(p, transform)

        # Grab pose as a vector
        target = np.array([
            pose_transformed.pose.position.x,
            pose_transformed.pose.position.y,
            pose_transformed.pose.position.z
        ])

        return target

    def solveIK(self, target):
        constraints = list()

        constraints.append ( ik.WorldGazeTargetConstraint( self.robot,
            self.bodies[self.config["optical_frame"]],  # Body to constrain
            np.array([0.0, 0.0, 0.1]).reshape(3, 1),    # Gaze axis
            target.reshape(3,1),                        # Gaze target (world frame)
            np.zeros([3,1]),                            # Gaze origin (body frame)
            self.config["cone_threshold"]               # Cone threshold
        ))

        q_seed = self.robot.getZeroConfiguration()
        options = ik.IKoptions(self.robot)
        result = ik.InverseKin(self.robot, q_seed, q_seed, constraints, options)

        if result.info[0] != 1:
            print "Outside joint limits! (Result = {})".format(result.info[0])

        # If we're out of bounds, result is the best-effort solution
        return result.q_sol[0][:2] # Only return head pose

    def movePtu(self,q_sol):
        # Get robot conf
        start = np.array(self.conf_service().conf.robotHead)
        diff = np.abs(q_sol - start)

        # Only send if we're outside of the dead_band
        if np.max(diff) > self.config["dead_band"]:
            ptu_cmd = JointState()
            ptu_cmd.name = ['ptu_pan_joint', 'ptu_tilt_joint']
            ptu_cmd.position = q_sol
            ptu_cmd.velocity = [self.config["ptu_velocity"]]*2
            self.pub_ptu_cmd.publish(ptu_cmd)

    def tag_callback(self, msg):
        # Filter for tags that we care about
        for marker in msg.markers:
            if marker.id != self.config["tag"]: continue

            # Use ROS to translate from camera to world frame
            target = self.markerToWorldFrame(marker)

            # Solve for best head position
            q_sol = self.solveIK(target)
            print q_sol

            # Send command to the PTU
            self.movePtu(q_sol)

            break

    def track(self):
        print "Subscribing to {}!".format(self.config["tag_topic"])
        self.sub_tag_topic = rospy.Subscriber(self.config["tag_topic"], MarkerArray, self.tag_callback)
        self.pub_ptu_cmd = rospy.Publisher('/ptu/cmd', JointState, queue_size=10)
        rospy.spin()

def main():
    # Initialize ROS node
    rospy.init_node('tag_tracker')

    config = {
        # Tracking config
        "tag": 50,
        "tag_topic": "/aruco_tags/markers",

        # Robot config
        "ik_urdf": "/home/momap/momap/src/robot_core/robot_descriptions/momap_description/urdf/.momap_left_arm-drake.urdf",
        "optical_frame": "kinect1_rgb_optical_frame",

        # IK options
        "cone_threshold": 0.001,

        # Motion options
        "ptu_velocity": 2.0,
        "dead_band": 0.1
    }

    tracker = TagTracker(config)
    tracker.track()

if __name__=="__main__":
    main()
