#!/usr/bin/env python

"""
Script for tracking an Aruco tag with a camera.

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
from geometry_msgs.msg import TransformStamped, Transform

# For IK
import pydrake
from pydrake.solvers import ik

import numpy as np
from eyehand_utils import query_yes_no

from visp_hand2eye_calibration.msg import TransformArray

class TagTracker(object):
    def __init__(self, config):
        self.config = config
        self.target = None # Initialize tracked target pose
        self.cam_T_target = None
        self.w_T_hand = None

        # Msgs for visp_hand2eye_calibration
        self.cam_T_target_array = TransformArray()
        self.w_T_hand_array = TransformArray()

        # Publishers for visp_hand2eye_calibration
        self.w_T_hand_pub = rospy.Publisher("/world_effector", Transform, queue_size=1000)
        self.cam_T_target_pub = rospy.Publisher("/camera_object", Transform, queue_size=1000)

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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        print "ROS stack ready!"
        print

    # Publish the aruco detection as a tf in cam frame and store the transform
    def publish_aruco_as_tf(self,marker):
        t = TransformStamped()
        t.header = marker.header
        t.child_frame_id = "aruco_track"
        t.transform.translation.x = marker.pose.pose.position.x
        t.transform.translation.y = marker.pose.pose.position.y
        t.transform.translation.z = marker.pose.pose.position.z
        t.transform.rotation.x = marker.pose.pose.orientation.x
        t.transform.rotation.y = marker.pose.pose.orientation.y
        t.transform.rotation.z = marker.pose.pose.orientation.z
        t.transform.rotation.w = marker.pose.pose.orientation.w
        self.cam_T_target = t.transform
        self.tf_broadcaster.sendTransform(t)
        print 'cam_T_target:\n', self.cam_T_target.translation

    # Get the camera pose in world frame
    def get_w_T_cam(self, marker):
        # Transform pose to world frame (ROS)
        w_T_cam = self.tf_buffer.lookup_transform("world",
                        marker.header.frame_id, #source frame
                        rospy.Time(0),
                        rospy.Duration(1.0)) #wait for 1 second

        # Grab pose as a vector
        w_T_cam_vec = np.array([
            w_T_cam.transform.translation.x,
            w_T_cam.transform.translation.y,
            w_T_cam.transform.translation.z,
            w_T_cam.transform.rotation.w,
            w_T_cam.transform.rotation.x,
            w_T_cam.transform.rotation.y,
            w_T_cam.transform.rotation.z
        ])

        print 'w_T_cam:\n', w_T_cam.transform.translation

        return w_T_cam_vec

    # Store the w_T_hand transform
    def store_w_T_hand(self, marker):
        # Transform pose to world frame (ROS)
        w_T_hand = self.tf_buffer.lookup_transform("world",
                        "interface_plate", #source frame
                        rospy.Time(0),
                        rospy.Duration(1.0)) #wait for 1 second
        self.w_T_hand = w_T_hand.transform
        print 'w_T_hand:\n', self.w_T_hand.translation

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
            self.target = self.get_w_T_cam(marker)
            self.publish_aruco_as_tf(marker)
            self.store_w_T_hand(marker)

            # Solve for best head position
            q_sol = self.solveIK(self.target[0:3])
            # print q_sol

            if self.config["move_head"]:
                # Send command to the PTU
                self.movePtu(q_sol)
            break

    def start_track(self):
        # print "Subscribing to {}!".format(self.config["tag_topic"])
        print "Start tracking!"
        self.sub_tag_topic = rospy.Subscriber(self.config["tag_topic"], MarkerArray, self.tag_callback)
        self.pub_ptu_cmd = rospy.Publisher('/ptu/cmd', JointState, queue_size=10)

    def stop_track(self):
        print "Stop tracking!"
        # Unsubscribe
        self.sub_tag_topic.unregister()

    def track(self):
        self.start_track()
        rospy.spin()

    def track_for(self, dur):
        self.start_track()
        rospy.sleep(dur)
        self.stop_track()
        return self.target

    def track_tag_in_optical(self, dur):
        self.start_track()
        rospy.sleep(dur)
        self.stop_track()
        return self.cam_T_target

    # Publish w_T_hand and cam_T_target for visp hand to eye
    def publish_transforms(self):
        self.w_T_hand_pub.publish(self.w_T_hand)
        self.cam_T_target_pub.publish(self.cam_T_target)

def main():
    # Initialize ROS node
    rospy.init_node('tag_tracker')

    config = {
        # Tracking config
        "tag": 50,
        "tag_topic": "/aruco_tags/markers",
        "move_head": True,

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
#    yep = True
#    while yep:
#        pos = tracker.track_for(10)
#        print "Final position:", pos
#        yep = query_yes_no("Track again?")


if __name__=="__main__":
    main()
