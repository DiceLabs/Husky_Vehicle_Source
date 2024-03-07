#!/usr/bin/python3
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_msgs.msg import Header
import moveit_commander
import geometry_msgs.msg
import sys
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi

from std_msgs.msg import Float64MultiArray
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperActionGoal
import math

rospy.init_node("arm_take")
class robot_arm:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator_right"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.current_pose = self.move_group.get_current_pose().pose
        self.current_quaternion = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                                   self.current_pose.orientation.z, self.current_pose.orientation.w]
        self.current_euler = euler_from_quaternion(self.current_quaternion)
        self.current_position = [self.current_pose.position.x, self.current_pose.position.y,
                                 self.current_pose.position.z]
        self.target_pose = geometry_msgs.msg.Pose()

        self.target_pose.position.x = 0.0
        self.target_pose.position.y = 0.0
        self.target_pose.position.z = 0.0
        self.target_pose.orientation.x = None
        self.target_pose.orientation.y = None
        self.target_pose.orientation.z = None
        self.target_pose.orientation.w = None

        self.pub1 = rospy.Publisher('/right_gripper/command_robotiq_action/goal', CommandRobotiqGripperActionGoal,
                              queue_size=10)

        self.gripper = CommandRobotiqGripperActionGoal()
        self.gripper.header = Header()
        self.gripper.header.seq = 0
        self.gripper.header.stamp = rospy.Time(0)
        self.gripper.header.frame_id = ''

        self.gripper.goal_id = CommandRobotiqGripperActionGoal().goal_id
        self.gripper.goal_id.stamp = rospy.Time(0)
        self.gripper.goal_id.id = ''

        self.gripper.goal.emergency_release = False
        self.gripper.goal.emergency_release_dir = 0
        self.gripper.goal.stop = False
        self.gripper.goal.position = 0.045  # position value limits from 0.00 to 0.085
        self.gripper.goal.speed = 0.01  # speed calue limits from 0.01 to 0.1
        self.gripper.goal.force = 1.0  # good default force 5.0

        self.roll=None
        self.pitch=None
        self.yaw=None

    def transform(self, coord, mat44):
        print("MAT44: ", mat44)
        print("POINT ", coord)
        xyz = tuple(np.dot(mat44, np.array([coord[0], coord[1], coord[2], 1.0])))[:3]
        r = geometry_msgs.msg.PointStamped()
        r.point = geometry_msgs.msg.Point(*xyz)
        return [r.point.x, r.point.y, r.point.z]

    def find_yaw(self,x,y):
        # Relate quadrants to the map frame axes
        if x > 0 and y > 0:
            self.yaw = math.atan2(abs(y), abs(x));
        elif x < 0 and y > 0:
            self.yaw = (math.pi / 2) + math.atan2(abs(x), abs(y))
        elif x < 0 and y < 0:
            self.yaw = -(math.pi / 2) - math.atan2(abs(x), abs(y))
        elif x > 0 and y < 0:
            self.yaw = -math.atan2(abs(y), abs(x))
        elif x != 0 and y == 0:
            if x > 0:
                self.yaw = 0.0
            if x < 0:
                self.yaw = math.pi
        elif x == 0 and y != 0:
            if y > 0:
                self.yaw = math.pi / 2

            if y < 0:
                self.yaw = -math.pi / 2
        else:
            self.yaw = 0.0

    def find_pitch(self, z, x):
        if z > 0 and x < 0:
            self.pitch = -math.atan2(abs(x), abs(z));
        elif z < 0 and x < 0:
            self.pitch = -(math.pi / 2) - math.atan2(abs(z), abs(x))
        elif z < 0 and x > 0:
            self.pitch = (math.pi / 2) + math.atan2(abs(z), abs(x))
        elif z > 0 and x > 0:
            self.pitch = math.atan2(abs(x), abs(z))

        elif z != 0 and x == 0:
            if z > 0:
                self.pitch = 0.0
            if z < 0:
                self.pitch = math.pi
        elif z == 0 and x != 0:
            if x > 0:
                self.pitch = math.pi / 2

            if x < 0:
                self.pitch = -math.pi / 2
        else:
            self.pitch = 0.0

    def find_roll(self, y, z):
        if y > 0 and z < 0:
            self.roll = -math.atan2(abs(z), abs(y));
        elif y < 0 and z < 0:
            self.roll = -(math.pi / 2) - math.atan2(abs(y), abs(z))
        elif y < 0 and z > 0:
            self.roll = (math.pi / 2) + math.atan2(abs(y), abs(z))
        elif y > 0 and z > 0:
            self.roll = math.atan2(abs(z), abs(y))

        elif y != 0 and z == 0:
            if y > 0:
                self.roll = 0.0
            if y < 0:
                self.roll = math.pi
        elif y == 0 and z != 0:
            if z > 0:
                self.roll = math.pi / 2

            if z < 0:
                self.roll = -math.pi / 2
        else:
            self.roll = 0.0

    def move_it(self,msg):
        if msg.data[2]>=1.0:
            print(msg.data[2])
            print("NOT GOOD")
            #pass

        if  abs(msg.data[2])==0.0 and abs(msg.data[1])==0.0 and abs(msg.data[0])==0.0:
            print("NOT GOOD")
            #pass

        else:
            listener = tf.TransformListener()
            """Get transform matrix to convert Point in 3D space to from camera_realsense_link_gazebo to base_link"""
            listener.waitForTransform('base_link', 'secondary_camera_realsense_depth_optical_frame', rospy.Time(0), rospy.Duration(20.0))
            (trans, rot) = listener.lookupTransform('base_link', 'secondary_camera_realsense_depth_optical_frame', rospy.Time(0))
            TF_c_b = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            print("PRE POINT ", msg.data)
            xyz_point = self.transform(msg.data, TF_c_b)
            x = xyz_point[0]
            y = xyz_point[1]
            z = xyz_point[2]
            self.find_yaw(x,y)
            self.find_pitch(z,x)
            self.find_roll(y,z)

            """# Determine the quadrant
            quadrant = None
            if x > 0 and y > 0:
                quadrant = "Quadrant I"
            elif x < 0 and y > 0:
                quadrant = "Quadrant II"
            elif x < 0 and y < 0:
                quadrant = "Quadrant III"
            elif x > 0 and y < 0:
                quadrant = "Quadrant IV"
            elif x == 0 and y != 0:
                quadrant = "On the y-axis"
            elif x != 0 and y == 0:
                quadrant = "On the x-axis"
            else:
                quadrant = "At the origin"""

            print("angle: ", self.roll,",",self.pitch,",",self.yaw)
            print("GRIPPER POINT: ",xyz_point)
            self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z = xyz_point
            self.target_pose.orientation.x,\
                self.target_pose.orientation.y,\
                self.target_pose.orientation.z,\
                self.target_pose.orientation.w = quaternion_from_euler(self.roll, self.pitch, self.yaw)

            #self.move_group.set_goal_orientation_tolerance(2*np.pi)

            # Set a new planning timeout (in seconds)
            new_timeout = 3.0  # Adjust the timeout value as needed

            # Update the planning timeout
            self.move_group.set_planning_time(new_timeout)

            self.move_group.set_pose_target(self.target_pose)
            self.move_group.set_max_velocity_scaling_factor(0.1)
            plan = self.move_group.go(wait=True)
            if plan:
                rospy.loginfo("Motion planning and execution succeeded!")
            else:
                rospy.logerr("Motion planning and execution failed!")

            self.pub1.publish(self.gripper)

            rospy.sleep(1)  # Delay to allow the message to be published

            self.gripper.goal.position = 0.085

            self.pub1.publish(self.gripper)

            rospy.sleep(1)  # Delay to allow the message to be published

            # Exit the script after sending the message
            #rospy.signal_shutdown('Message sent')



def main():

    tester = robot_arm()
    rospy.sleep(1)

    rospy.Subscriber("/the_points", Float64MultiArray, tester.move_it, queue_size=1, buff_size=2_000_000)

    rospy.spin()


if __name__ == "__main__":
    main()
