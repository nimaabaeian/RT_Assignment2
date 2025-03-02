#! /usr/bin/env python3

"""
.. module:: my_node_B
   :platform: Unix
   :synopsis: Python module for the last_target service node

.. moduleauthor:: Nima Abaeian

A node that implements a service to provide the last target position entered by the user.

Services:
    last_target
"""

# Import necessary libraries
import rospy
from assignment_2_2023.srv import LastTarget, LastTargetResponse

def put_target(req):
    """
    Callback function for the 'last_target' service.

    This function retrieves the last target position (x, y) from ROS parameters
    and returns them in a LastTargetResponse message.

    Parameters
    ----------
    req : assignment_2_2023.srv.LastTargetRequest
        The request message for the service (unused).

    Returns
    -------
    assignment_2_2023.srv.LastTargetResponse
        The response message containing the last target position (x, y).
    """
    res = LastTargetResponse()
    res.target_x = rospy.get_param('/des_pos_x', default=0.0)
    res.target_y = rospy.get_param('/des_pos_y', default=0.0)
    return res

def main():
    """
    Main function to initialize the last_target service node.

    This function initializes the ROS node, sets up the 'last_target' service,
    and keeps the node running.
    """
    # Initialize the node 
    rospy.init_node("node_b_service")
    rospy.loginfo("Target node started and ready to provide the last target the user entered")
    
    # Initialize the service
    rospy.Service('last_target', LastTarget, put_target)
    
    rospy.spin()

if __name__ == "__main__":
    main()

