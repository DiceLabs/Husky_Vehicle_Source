#!/usr/bin/env python

import rospy
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperActionGoal
from std_msgs.msg import Header

rospy.init_node('rostopic_pub_example')

pub = rospy.Publisher('/left_gripper/command_robotiq_action/goal', CommandRobotiqGripperActionGoal, queue_size=10)
rospy.sleep(1)  # Delay to ensure the publisher is set up

msg = CommandRobotiqGripperActionGoal()
msg.header = Header()
msg.header.seq = 0
msg.header.stamp = rospy.Time(0)
msg.header.frame_id = ''

msg.goal_id = CommandRobotiqGripperActionGoal().goal_id
msg.goal_id.stamp = rospy.Time(0)
msg.goal_id.id = ''

msg.goal.emergency_release = False
msg.goal.emergency_release_dir = 0
msg.goal.stop = False
msg.goal.position = 0.045	#position value limits from 0.00 to 0.085
msg.goal.speed = 0.01		#speed calue limits from 0.01 to 0.1
msg.goal.force = 5.0		#good default force 5.0

pub.publish(msg)

rospy.sleep(1)  # Delay to allow the message to be published

# Exit the script after sending the message
rospy.signal_shutdown('Message sent')
