#! /usr/bin/env python
##@package assignment2_part1
# \file user_input.py
# \brief A ROS node that provides a service to return the current user-defined target position coordinates
# \author Marco Lovecchio
# \version 1.0.0
# \date 15/03/2025
#
# \details
# **Subscribes to:** <BR>
# 	 /user_pose
#
# **Publshes to:** <BR>
# 	[None]
# 
# **Service:** <BR>
#	user_input
#
# Description:
# This node subscribes to the /user_pose topic to retrive the current target position set by the user in the main node. It provides a service called 'user_input' that returns the current x and y coordinates
# of the target position when requested. This allows other nodes to access the current target position without needing to subscribe to the /usep_pose topic directly.
#

import rospy
from geometry_msgs.msg import PoseStamped
from assignment2_part1.srv import user_input, user_inputResponse

## \var x
# Current x-coordinate of the target position
x = 0

## \var y
# Current y-coordinate of the target position
y = 0

##
# \brief callback_UP for the /user_pose topic subscription
# \param msg = PoseStamped message received from the /user_pose topic
#
# This callback is called whenever a new message is received on the /user_pose topic. It extracts the x and y coordinates from the received message and updates the global x and y variables accordingly.
# These updated values will be used by the service to respond to requests for the current target position.
#
def callback_UP(msg):
	global x, y
	x = msg.pose.position.x
	y = msg.pose.position.y

##
# \brief return_xy service handler function for the user_input service
# \param req = service request
# \return user_inputResponse(x, y) = response containing the current x and y coordinates
#
# This function is called whenever a request is made to the 'user_input' service. It returns a responde containing the current values of the global x and y variables, which represent the current target
# position coordinates. These coordinates are continuously updated by the callback_UP function as new messages are received on the /user_pose topic.
#
def return_xy(req):
	global x, y
	return user_inputResponse(x, y)

##
# \brief main function that initializes the node and sets up subscriptions and services
#
# The main inizializes the ROS node, sets up the subscription to the /user_pose topic, and creates the 'user_input' service. It then enters a loop that keeps the node running and handling callbacks and
# service requests until the node is shut down. The node operates at a rate of 10Hz, which determines how frequently the loop iterates.
#
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
