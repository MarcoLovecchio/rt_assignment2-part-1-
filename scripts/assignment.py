#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment2_part1.msg
from assignment2_part1.msg import robotposvel

des_pose = PoseStamped()
pose_ = Pose()
twist_ = Twist()
msg =  robotposvel()
pub = rospy.Publisher('/robotposvel', robotposvel, queue_size = 10)

def planning_client(des_pose):
	global msg, pub, pose_, twist_
	client = actionlib.SimpleActionClient('/reaching_goal', assignment2_part1.msg.PlanningAction)
	client.wait_for_server()
	goal = assignment2_part1.msg.PlanningGoal(target_pose = des_pose)
	client.send_goal(goal)
	
	while not client.get_result():
		userin = input('x to cancel the goal: ')
		if userin == 'x':
			client.cancel_goal()
			print('Goal cancelled')
			return 0
			
		while not client.get_result():
			msg.x = pose_.position.x
			msg.y = pose_.position.y
			msg.vel_x = twist_.linear.x
			msg.vel_z = twist_.angular.z
			pub.publish(msg)
		
		#client.wait_for_result()
		
	return client.get_result()

def clbk_odom(msg):
	global pose_
	pose_ = msg.pose.pose
	global twist_
	twist_ = msg.twist.twist

def main():
	rospy.init_node("Assignment_node", anonymous = True)
	global des_pose, pose_, twist_
	
	rospy.Subscriber('/odom', Odometry, clbk_odom)
	#pub = rospy.Publisher('robotposvel', robotposvel, queue_size = 10)
	
	rate = rospy.Rate(10)
	rate.sleep()
	while not rospy.is_shutdown():
		des_pose.pose.position.x = float(input('X Goal: '))
		des_pose.pose.position.y = float(input('Y Goal: '))
		
		planning_client(des_pose)
		
		
		
		rate.sleep()


if __name__ == "__main__":
    main()
