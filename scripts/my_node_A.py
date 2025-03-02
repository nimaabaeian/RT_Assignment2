#! /usr/bin/env python3

"""
.. module:: my_node_A
   :platform: Unix
   :synopsis: Python module for my_node_A

.. moduleauthor:: Nima Abaeian

A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. 
The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), 
by relying on the values published on the topic /odom. 
"""

# Import necessary libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from actionlib_msgs.msg import GoalStatus

# Initialize the publisher to publish position and velocity
pub = rospy.Publisher("/kinematics", Vel, queue_size=1)

def publish_kinematics(msg):
    """
    Callback function to publish the robot's kinematics.

    This function is called whenever a new Odometry message is received.
    It extracts the position and velocity of the robot and publishes them 
    as a custom `Vel` message.

    Parameters
    ----------
    msg : nav_msgs.msg.Odometry
        The Odometry message containing the robot's current state.
    """
    global pub

    # Get the position and velocity of the robot from Odometry
    linear_vel_now = msg.twist.twist.linear
    angular_vel_now = msg.twist.twist.angular

    # Create a new position and velocity message
    pos_vel = Vel()
    pos_vel.x = msg.pose.pose.position.x
    pos_vel.y = msg.pose.pose.position.y
    pos_vel.vel_x = linear_vel_now.x
    pos_vel.vel_z = angular_vel_now.z

    # Publish the custom message
    pub.publish(pos_vel)

def main():
    """
    Main function to run the ROS node.

    This function initializes the ROS node, sets up the action client,
    and handles user input for changing or canceling goals.

    The user is prompted to either change the goal or cancel the current goal.
    The new goal coordinates are taken from the user input and updated accordingly.
    """
    global pub

    # Initialize the node
    rospy.init_node('node_a_client')
    rospy.loginfo("Node A started successfully.")

    # Initialize action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()
    rospy.loginfo("Action successfully initialized.")

    have_goal = False

    while not rospy.is_shutdown():
        # Subscribe to odom topic and publish the obtained position and velocity
        rospy.Subscriber("/odom", Odometry, publish_kinematics)

        # Get the current target from the parameters in the launch file
        x_t = rospy.get_param('/des_pos_x')
        y_t = rospy.get_param('/des_pos_y')

        rospy.loginfo("Current goal is: target_x = %f, target_y = %f", x_t, y_t)

        # Define the new goal based on the launch targets
        goal_new = assignment_2_2023.msg.PlanningGoal()
        goal_new.target_pose.pose.position.x = x_t
        goal_new.target_pose.pose.position.y = y_t

        print("###################################################################\n")
        print("## Write down your choice based on one of the following options: ##\n")
        print("###################################################################\n")
        print("########### -------> Write Change: Change Target        ###########\n")
        print("########### -------> Write Cancel: Cancel Target        ###########\n")
        print("###################################################################\n")
        print("## Note: Keep in mind that you should cancel the movement before ##\n")
        print("######### setting a new target or you will get a warning. #########\n")
        print("###################################################################\n")

        # Ask user if they want to cancel the goal or move towards a new goal
        Input1 = input("What's your choice now:\n")

        if Input1 == 'Change':
            if have_goal:
                rospy.logwarn("You should first stop the current target")
            else:
                # Get the value for x and y targets
                input_x = float(input('Enter a value for the new x_coordinate:'))
                input_y = float(input('Enter a value for the new y_coordinate:'))

                # Change the value of the goal
                rospy.set_param('/des_pos_x', input_x)
                rospy.set_param('/des_pos_y', input_y)
                goal_new.target_pose.pose.position.x = input_x
                goal_new.target_pose.pose.position.y = input_y

                # Send the new goal
                client.send_goal(goal_new)
                have_goal = True

        elif Input1 == 'Cancel':
            if have_goal:
                # Cancel the goal
                have_goal = False
                client.cancel_goal()
                rospy.loginfo("The goal is cancelled successfully")
            else:
                rospy.logwarn("There are no goals to be cancelled. First determine a goal!!!")
        else:
            rospy.logwarn("Warning!!! The command should either be 'Change' or 'Cancel'.")

        rospy.loginfo("The chosen goal is: target_x = %f, target_y = %f", goal_new.target_pose.pose.position.x, goal_new.target_pose.pose.position.y)

if __name__ == '__main__':
    main()

