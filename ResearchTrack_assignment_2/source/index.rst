.. RT1-Assignment2 documentation master file, created by
   sphinx-quickstart on Fri May 31 18:32:21 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to rt1_assignment2's documentation!
==========================================
This documentation provides a comprehensive guide to the `rt_assignment2` package. The package includes multiple nodes implemented in Python, each performing specific tasks essential for the assignment. Below you will find detailed documentation for each node, including their functionality, usage, and integration within the overall system.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


my_node_A
===========================
Node A orchestrates target setting and motion control. It integrates an action client for setting navigation targets and publishes real-time kinematic data, including position and velocity, derived from the robot's odometry.

rt1_assignmrnt2 Module
===========================

.. automodule:: scripts.my_node_A
  :members:

.. _scripts.my_node_B:

my_node_B
===========================
Node B serves as a memory bank for the system, offering a service to recall the last target position set by the user. It efficiently retrieves and delivers this historical data upon request, ensuring seamless continuity in navigation tasks.

.. module:: scripts.my_node_B

.. automodule:: scripts.my_node_B
   :members:
   :undoc-members:
   :show-inheritance:
   
.. _scripts.my_node_C:

my_node_C
===========================
Node C serves as an analytical hub, dynamically computing the average velocity and tracking the distance to the last target position. By processing real-time velocity data and leveraging historical target positions, it provides valuable insights for fine-tuning robot navigation strategies. 

.. module:: scripts.my_node_C

.. automodule:: scripts.my_node_C
   :members:
   :undoc-members:
   :show-inheritance:
