Piklet Description ROS2 Package
===============================

This repository contains the ROS2 package ``piklet_description`` for the Piklet pick-and-place robot.
It includes a modular robot description (URDF/Xacro), RViz configurations, Gazebo simulation launch files, and sensor setups.

Directory Structure
-------------------

.. code-block:: text

    piklet_description/
    ├── urdf/
    │   ├── piklet_robot.xacro       # Main modular robot description
    │   ├── body.xacro               # Body plates and poles
    │   ├── wheel.xacro              # Differential drive wheels
    │   ├── caster.xacro             # Passive support wheels
    │   ├── lidar.xacro              # Lidar sensor link and joint
    │   ├── depth_camera.xacro       # Depth camera link and joint
    │   └── gazebo.xacro             # Gazebo plugin definitions
    ├── rviz/
    │   ├── display.rviz             # URDF-only RViz configuration
    │   └── display_gazebo.rviz      # RViz configuration for Gazebo simulation
    ├── launch/
    │   ├── display.launch.py        # Main RViz visualization launch file
    │   ├── view_model.launch.py     # Convenience launcher: URDF-only view
    │   ├── view_gazebo.launch.py    # Convenience launcher: simulation-ready RViz
    │   └── sim.launch.py            # All-in-one launch: Gazebo + robot + RViz
    └── README.rst                    # This documentation

URDF/Xacro Files
----------------

+-----------------------+--------------------------------------------------------+
| File                  | Description                                            |
+=======================+========================================================+
| piklet_robot.xacro    | Main robot description file that includes all         |
|                       | components. Modular structure allows easy maintenance |
|                       | and code reuse.                                       |
+-----------------------+--------------------------------------------------------+
| body.xacro            | Defines the container-like robot body (two plates    |
|                       | supported by poles) that holds picked objects.       |
+-----------------------+--------------------------------------------------------+
| wheel.xacro           | Defines differential drive wheels (left and right)   |
|                       | with geometry, collision, and inertial properties.   |
+-----------------------+--------------------------------------------------------+
| caster.xacro          | Defines two passive support wheels with joints and   |
|                       | links.                                                |
+-----------------------+--------------------------------------------------------+
| lidar.xacro           | Adds a lidar link and fixed joint for the robot.     |
+-----------------------+--------------------------------------------------------+
| depth_camera.xacro    | Adds depth camera link and fixed joint for the robot.|
+-----------------------+--------------------------------------------------------+
| gazebo.xacro          | Includes Gazebo plugin definitions for diff-drive    |
|                       | controller, lidar, and depth camera simulation.      |
+-----------------------+--------------------------------------------------------+

RViz Configurations
------------------

+-----------------------+--------------------------------------------------------+
| File                  | Purpose                                                |
+=======================+========================================================+
| display.rviz          | RViz configuration for visualizing the URDF, TF tree,|
|                       | and interactive joint sliders (no simulation).       |
+-----------------------+--------------------------------------------------------+
| display_gazebo.rviz   | RViz configuration for Gazebo simulation with robot, |
|                       | TF, lidar, and depth camera topics preloaded.        |
+-----------------------+--------------------------------------------------------+

Launch Files
------------

+-----------------------+--------------------------------------------------------+
| File                  | Purpose                                                |
+=======================+========================================================+
| display.launch.py     | Main launch file to view the robot in RViz. Supports |
|                       | `sim` and `use_sim_time` arguments.                  |
+-----------------------+--------------------------------------------------------+
| view_model.launch.py  | Convenience launcher for URDF-only visualization in  |
|                       | RViz.                                                 |
+-----------------------+--------------------------------------------------------+
| view_gazebo.launch.py | Convenience launcher for simulation-ready RViz with  |
|                       | sensor topics.                                        |
+-----------------------+--------------------------------------------------------+
| sim.launch.py         | All-in-one launcher: starts Gazebo Harmonic, spawns  |
|                       | robot from Xacro, and launches RViz with Gazebo-ready|
|                       | config.                                               |
+-----------------------+--------------------------------------------------------+

Usage Examples
--------------

- URDF-only visualization::

    ros2 launch piklet_description view_model.launch.py

- Simulation-ready RViz::

    ros2 launch piklet_description view_gazebo.launch.py

- Gazebo + robot + RViz all-in-one::

    ros2 launch piklet_description sim.launch.py

Notes
-----

- All URDF/Xacro files are modular for easy extension.
- Sensors (lidar and depth camera) are configured to work in both RViz and Gazebo.
- Launch files support ROS2 simulation time (`use_sim_time`) for proper integration with Gazebo.
- The package is designed for ROS2 Jazzy + Gazebo Harmonic.

Robot Structure Diagram
-----------------------

To visualize the robot links and joints:

1. Using ``urdf_to_graphiz``::

    # Convert xacro to DOT
    ros2 run urdf_tutorial xacro_to_graph.py $(ros2 pkg prefix piklet_description)/share/piklet_description/urdf/piklet_robot.xacro > piklet_robot.dot

    # Convert DOT to PNG
    dot -Tpng piklet_robot.dot -o piklet_robot.png

2. Using RViz (interactive)::

    ros2 launch piklet_description view_model.launch.py

    - Add **TF** display to show all link axes.
    - Rotate/zoom and take a screenshot for ``piklet_robot_structure.png``.

Example README inclusion::

    .. image:: rviz/piklet_robot_structure.png
       :alt: Piklet Robot Structure
