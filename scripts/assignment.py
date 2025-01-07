#! /usr/bin/env python
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

des_pose = PoseStamped()
pose_ = Pose()
twist_ = Twist()
msg =  robotposvel()
pub = rospy.Publisher('/robotposvel', robotposvel, queue_size = 10)
pub2 = rospy.Publisher('/user_pose', PoseStamped, queue_size = 10)

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

def feedback_callback(feedback):
	actual_position = feedback.actual_pose.position
	target_position = des_pose.pose.position
	
	distance = math.sqrt(
		(actual_position.x - target_position.x)**2 +
		(actual_position.y - target_position.y)**2 +
		(actual_position.z - target_position.z)**2)
        
def done_callback(state, result):
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("[STATE] Goal reached")

def callback_odom(msg):
	global pose_
	pose_ = msg.pose.pose
	global twist_
	twist_ = msg.twist.twist

def is_float(value):
	try:
		float(value)
		return True
	except ValueError:
		return False

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
