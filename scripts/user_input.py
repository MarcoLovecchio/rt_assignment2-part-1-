#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def callback_UP(msg):
	x = msg.pose.position.x
	y = msg.pose.position.y
	
	print(' Last X position: ', x)
	print(' Last Y position: ', y)

def main():
	rospy.init_node("UserInput_node", anonymous = True)
	rate = rospy.Rate(10)
	rate.sleep()
	while not rospy.is_shutdown():
		rospy.Subscriber('/user_pose', PoseStamped, callback_UP)
		rate.sleep()

if __name__ == "__main__":
	main()
