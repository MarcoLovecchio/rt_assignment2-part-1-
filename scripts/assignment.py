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
			pub2.publish(des_pose)
		
		#client.wait_for_result()
		
	return client.get_result()

def feedback_callback(feedback):
	actual_position = feedback.actual_pose.position
	target_position = des_pose.pose.position
	
	distance = math.sqrt(
		(actual_position.x - target_position.x)**2 +
		(actual_position.y - target_position.y)**2 +
		(actual_position.z - target_position.z)**2)
    
	#rospy.loginfo("[FEEDBACK] Position: (%f, %f, %f); Distance: %f", actual_position.x, actual_position.y, actual_position.z, distance)

        
def done_callback(state, result):
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("[STATE] Goal reached")

def callback_odom(msg):
	global pose_
	pose_ = msg.pose.pose
	global twist_
	twist_ = msg.twist.twist

def main():
	rospy.init_node("Assignment_node", anonymous = True)
	global des_pose
	
	rospy.Subscriber('/odom', Odometry, callback_odom)
	
	rate = rospy.Rate(10)
	rate.sleep()
	while not rospy.is_shutdown():
		des_pose.pose.position.x = float(input('X Goal: '))
		des_pose.pose.position.y = float(input('Y Goal: '))
		
		planning_client(des_pose)
		
		rate.sleep()


if __name__ == "__main__":
	main()
