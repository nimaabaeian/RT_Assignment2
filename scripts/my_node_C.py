#! /usr/bin/env python3

"""
Node C: Average Velocity Calculator
-----------------------------------

This module implements a ROS node responsible for calculating the average velocity and distance to the last target position entered by the user. It subscribes to the `/kinematics` topic to receive the current position and velocity of the robot, and it utilizes the `last_target` service to retrieve the last target position (x, y).

Services:
    - `average`: Calculates the average velocity and distance to the last target position.

Subscribers:
    - `/kinematics`: Receives the current position and velocity of the robot.

Author: Nima Abaeian
"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import LastTarget, LastTargetRequest
from assignment_2_2023.srv import Average, AverageResponse
import math

# Wait for the last_target service to be available and create a service proxy
rospy.wait_for_service("last_target")
client = rospy.ServiceProxy('last_target', LastTarget)
request = LastTargetRequest()  # Create a LastTargetRequest (empty request in this case)

velocity_list = []
distance = 0
average_vel_x = 0

def give_avg(req):
    """
    Callback function for the 'average' service.

    This function calculates the average velocity and distance to the last target position.

    Parameters
    ----------
    req : assignment_2_2023.srv.AverageRequest
        The request message for the service (unused).

    Returns
    -------
    assignment_2_2023.srv.AverageResponse
        The response message containing the calculated distance and average velocity.
    """
    global distance, average_vel_x

    res = AverageResponse()
    res.dist = distance
    res.velocity_mean = average_vel_x
    return res

def get_average(msg):
    """
    Callback function for the /kinematics topic subscriber.

    This function calculates the distance to the last target position and the average velocity.

    Parameters
    ----------
    msg : assignment_2_2023.msg.Vel
        The custom Vel message containing the current position and velocity.
    """
    global response, velocity_list, distance, average_vel_x

    # Call the LastTarget service and get the response
    response = client(request)
    target_x = response.target_x
    target_y = response.target_y

    Window = rospy.get_param('average_window')

    x_now = msg.x
    y_now = msg.y
    vel_x_now = msg.vel_x

    # Calculate the distance to the target
    distance = math.sqrt((target_x - x_now)**2 + (target_y - y_now)**2)

    # Add the current velocity to the list
    velocity_list.append(vel_x_now)

    # Calculate the average velocity over the specified window
    if len(velocity_list) < Window:
        average_vel_x = sum(velocity_list) / len(velocity_list)
    else:
        average_vel_x = sum(velocity_list[-Window:]) / Window

def main():
    """
    Main function to initialize the average calculator node.

    This function initializes the ROS node, sets up the 'average' service,
    subscribes to the /kinematics topic, and keeps the node running.
    """
    # Initialize the node
    rospy.init_node("node_c_service")
    rospy.loginfo("Node started and ready to calculate the average")

    # Initialize the service
    rospy.Service('average', Average, give_avg)

    # Subscribe to the /kinematics topic
    rospy.Subscriber("/kinematics", Vel, get_average)

    rospy.spin()

if __name__ == "__main__":
    main()

