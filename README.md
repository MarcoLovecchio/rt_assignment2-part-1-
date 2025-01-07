# ResearchTrack1_Assignment2-part-1-
The first node 'assignment.py' implements an action client, allowing the user to set a target goal (x, y), to be reached by the robot using the action server.
Launching the code, the target is set to the values written in the parameter server, setted by the launch file.
The target values are published on a new topic /user_pose, to be used by the second node.
During the movement of the robot, the user can type 'x' to cancel the goal and stop the robot, then set new target coordinates.
If the robot reach the goal, the user can set other coordinates.
Also, a custom message, with position and velocity of the robot (x, y, vel_l_x, vel_a_z) has been created and updated by relying on the values published on the topic /odom, then published on a new topic /robotposvel.
The second node 'user_input.py' is a service node, that simply read the last target setted by the user on the topic /user_pose and return them on the service user_input.
The launch file 'assignment1.launch' has been modified, in order to launch the new nodes and display the first one in a new terminal.

In order to run the code you need to do the following steps:

  Clone the repository inside the src folder of a ros workspace:

    git clone https://github.com/MarcoLovecchio/rt_assignment2-part-1-.git

  Build the workspace:
    
    cd ..
    catkin_make

  Go inside the launch folder of the package:

    cd /assignment2_part1/launch

  Launch the launch file assignment1:

    roslaunch assignment1.launch

In order to call the service:

    rosservice call user_input
