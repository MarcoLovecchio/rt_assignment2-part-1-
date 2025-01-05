#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from assignment2_part1.srv import user_input, user_inputResponse

x = 0
y = 0

def callback_UP(msg):
	global x, y
	x = msg.pose.position.x
	y = msg.pose.position.y

def return_xy(req):
	global x, y
	return user_inputResponse(x, y)

def main():
	global x, y
	rospy.init_node("UserInput_node", anonymous = True)
	rospy.Subscriber('/user_pose', PoseStamped, callback_UP)
	rospy.Service('user_input', user_input, return_xy)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == "__main__":
	main()
