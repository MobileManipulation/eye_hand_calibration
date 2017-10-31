#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


rospy.init_node("ptu_sim")

pub_ptu_states = rospy.Publisher('/ptu_joint_states', JointState, queue_size=10)

def ptu_callback(data):
    # Data is a JointState... Just publish it
    print data
    print 'inside ptu_sim'
    pub_ptu_states.publish(data)

sub_ptu_states = rospy.Subscriber('/ptu/cmd', JointState, ptu_callback)

rospy.spin()
