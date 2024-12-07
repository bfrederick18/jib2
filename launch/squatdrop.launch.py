"""Launch the squatting and ball dropping demo

   ros2 launch jib2 squatdrop.launch.py

   This should start
     1) RVIZ, ready to view the robot and the ball
     2) The robot_state_publisher to broadcast the robot model
     3) The squat demo
     4) The ball drop demo

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('demos'), 'rviz/viewmarkers.rviz')
    
    # Locate the URDF file.
    urdf = os.path.join(pkgdir('atlas_description'), 'urdf/atlas_v5.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())
    
    node_squatdrop = Node(
        name       = 'squatdrop',
        package    = 'jib2',
        executable = 'squatdrop',
        output     = 'screen',
        on_exit    = Shutdown())

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the robot_state_publisher, RVIZ, and the demo.
        node_robot_state_publisher,
        node_rviz,
        node_squatdrop
    ])
