#! /usr/bin/env python

import keyboard
import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
import math
import actionlib
import actionlib.msg
import assignment2_part1.msg

pose = PoseStamped()

def planning_client(pose):
	client = actionlib.SimpleActionClient('/reaching_goal', assignment2_part1.msg.PlanningAction)
	client.wait_for_server()
	goal = assignment2_part1.msg.PlanningGoal(target_pose = pose)
	client.send_goal(goal)
	
	while not client.get_result():
		userin = input('x to cancel the goal: ')
		if userin == 'x':
			client.cancel_goal()
			print('Goal cancelled')
			return 0
			
		client.wait_for_result()
		
	return client.get_result()



def main():
	rospy.init_node("Assignment_node", anonymous = True)
	global pose
	
	rate = rospy.Rate(20)
	rate.sleep()
	while not rospy.is_shutdown():
		pose.pose.position.x = float(input('X Goal: '))
		pose.pose.position.y = float(input('Y Goal: '))
		
		planning_client(pose)
		
		rate.sleep()


if __name__ == "__main__":
    main()
