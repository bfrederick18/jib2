"""
ros2 launch jib2 cannonball.launch.py
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

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())
    
    node_cannonball = Node(
        name       = 'cannonball',
        package    = 'jib2',
        executable = 'cannonball',
        output     = 'screen',
        on_exit    = Shutdown())
    
    node_subscriber = Node(
        name       = 'subscriber',
        package    = 'jib2',
        executable = 'subscriber',
        output     = 'screen',
        on_exit    = Shutdown())

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the robot_state_publisher, RVIZ, and the demo.
        node_rviz,
        node_cannonball,
        node_subscriber
    ])
