#! /usr/bin/env python
##@package assignment2_part1
# \file assignment.py
# \brief This node implements an action client, allowing the user to set a target goal (x, y), to be reached by the robot using the action server.
# \author Marco Lovecchio
# \version 1.0.0
# \date 15/03/2025
#
# \details
#
# **Subscribes to:** <BR>
# 	 /odom
#
# **Publishes to:** <BR>
# 	 /robotposvel <BR>
#	 /user_pose
#
# **Service:** <BR>
#	[None]
#
# Description:
# Launching the code, the target is set to the values written in the parameter server, setted by the launch file. The target values are published on a new topic /user_pose, to be used by the second node.
# During the movement of the robot, the user can type 'x' to cancel the goal and stop the robot, then set new target coordinates. If the robot reach the goal, the user can set other coordinates. Also, a 
# custom message, with position and velocity of the robot (x, y, vel_l_x, vel_a_z) has been created and updated by relying on the values published on the topic /odom, then published on a new topic 
# /robotposvel.
# 

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment2_part1.msg
from actionlib_msgs.msg import GoalStatus
from assignment2_part1.msg import robotposvel
import sys
import select

## \var des_pose
# Desired pose target for the robot
des_pose = PoseStamped()

## \var pose_
# Current pose of the robot
pose_ = Pose()

## \var twist_
# Current velocity of the robot
twist_ = Twist()

## \var msg
# Custom message to publish robot position and velocity
msg =  robotposvel()

## \var pub
# Publisher for the user target pose
pub = rospy.Publisher('/robotposvel', robotposvel, queue_size = 10)

## \var pub2
# Publisher for the user target pose
pub2 = rospy.Publisher('/user_pose', PoseStamped, queue_size = 10)

##
# \brief planning_client function handle the action client implementation
# \param des_pose = desired pose typed by the user
# \return client.get_result() = result of the action client (success of failure)
#
# This function creates an action client that communicates with the action server. It sends the goal to the server, monitors for user cancellation request, and continuously publishes updated robot position
# and velocity information. While waiting for the goal to be reached, it allows the user to cancel the goal typing 'x' and handles the appropriate callbacks.
#
def planning_client(des_pose):
	global msg, pub, pub2, pose_, twist_
	client = actionlib.SimpleActionClient('/reaching_goal', assignment2_part1.msg.PlanningAction)
	client.wait_for_server()
	goal = assignment2_part1.msg.PlanningGoal(target_pose = des_pose)
	client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)
	
	print("'x' to cancel the target ")
	while not client.get_result():
		msg.x = pose_.position.x
		msg.y = pose_.position.y
		msg.vel_x = twist_.linear.x
		msg.vel_z = twist_.angular.z
		
		i, o, e = select.select([sys.stdin], [], [], 1.0)
		if(i):
			cancel = sys.stdin.readline().strip()
			if cancel == 'x':
				client.cancel_goal()
				print("Goal cancelled")
				break
			else:
				print("Input not valid")
				print("'x' to cancel the target ")

		pub.publish(msg)
		pub2.publish(des_pose)
		
	return client.get_result()


##
# \brief feedback_callback for the action client
# \param feedback = feedback message from the action server
#
# This callback processes the feedback messages received from the action server. It calculates the Euclidean distance between the current robot position and the target position.
# This information can be used to monitor the robot's towards reaching its goal.
#
def feedback_callback(feedback):
	actual_position = feedback.actual_pose.position
	target_position = des_pose.pose.position
	
	distance = math.sqrt(
		(actual_position.x - target_position.x)**2 +
		(actual_position.y - target_position.y)**2 +
		(actual_position.z - target_position.z)**2)
        
##
# \brief done_callback for the action client
# \param state = final state of the action
# \param result = result message from the action server
#
# This callback is called when the action is completed (either successfully or not). It logs the message "Goal reached" when the SUCCEEDED state as been reached, when the robot successfully reaches the
# target position.
#
def done_callback(state, result):
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("[STATE] Goal reached")

##
# \brief callback_odom for odometry data
# \param msg = Odometry message received from the /odom topic
#
# This callback processes the odometry data published by the robot. It updates the global pose_ and  twist_ variables with the current pose and velocity information.
# This data is then used to update the custom robotposvel message that is published.
#
def callback_odom(msg):
	global pose_
	pose_ = msg.pose.pose
	global twist_
	twist_ = msg.twist.twist

## 
# \brief is_float function to check if a value can be converted to a float.
# \param value = input value to check
#
# This function attempts to convert the input value to a float. If the conversion is successful, it returns True, otherwise it returns False.
# This is used to validate user input for target coordinates to ensure they are valid numerical values.
#
def is_float(value):
	try:
		float(value)
		return True
	except ValueError:
		return False

##
# \brief main function that initializes the node and handles the main loop
#
# The main initializes the ROS node, sets up the initial target position from parameters, subscribes to the odometry topic, and handles the main loop for user input.
# It repeatedly propts the user for new target positions after each goal is completer or cancelled. It validates the input to ensure it consists of valid numerical values before setting them as new target.
#
def main():
	rospy.init_node("Assignment_node", anonymous = True)
	global des_pose
	des_pose.pose.position.x = rospy.get_param('des_pos_x')  
	des_pose.pose.position.y = rospy.get_param('des_pos_y')
	
	rospy.Subscriber('/odom', Odometry, callback_odom)
	
	planning_client(des_pose)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		des_pose.pose.position.x = input('X Goal: ')
		while not is_float(des_pose.pose.position.x):
			print("not a number")
			des_pose.pose.position.x = input('X Goal: ')
		des_pose.pose.position.y = input('Y Goal: ')
		while not is_float(des_pose.pose.position.y):
			print("not a number")
			des_pose.pose.position.y = input('Y Goal: ')
		
		des_pose.pose.position.x = float(des_pose.pose.position.x)
		des_pose.pose.position.y = float(des_pose.pose.position.y)
		planning_client(des_pose)
		rate.sleep()


if __name__ == "__main__":
	main()
