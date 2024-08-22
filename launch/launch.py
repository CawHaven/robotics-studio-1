#!/usr/bin/env python3

#

# Copyright 2019 ROBOTIS CO., LTD.

#

# Licensed under the Apache License, Version 2.0 (the "License");

# you may not use this file except in compliance with the License.

# You may obtain a copy of the License at

#

#     http://www.apache.org/licenses/LICENSE-2.0

#

# Unless required by applicable law or agreed to in writing, software

# distributed under the License is distributed on an "AS IS" BASIS,

# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# See the License for the specific language governing permissions and

# limitations under the License.

#

# Authors: Joep Tool
 
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():

    try:

        # Set the environment variable
        os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

        # Directories for package shares
        launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        pkg_turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')

        # Ensure the directories exist
        if not os.path.isdir(launch_file_dir):
            raise FileNotFoundError(f"Launch directory not found: {launch_file_dir}")
        if not os.path.isdir(pkg_gazebo_ros):
            raise FileNotFoundError(f"Gazebo ROS package directory not found: {pkg_gazebo_ros}")
        if not os.path.isdir(pkg_turtlebot3_bringup):
            raise FileNotFoundError(f"TurtleBot3 Bringup package directory not found: {pkg_turtlebot3_bringup}")
 
        # Launch configurations
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        x_pose = LaunchConfiguration('x_pose', default='-2.0')
        y_pose = LaunchConfiguration('y_pose', default='-0.5')
 
        # World file path
        world = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'worlds',
            'small_house.world'
        )
 
        # Commands for launching Gazebo server and client
        gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        )
 
        gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
 
        # Commands for launching robot state publisher and spawning TurtleBot3
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
 
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        )
 
        # Additional launch files and commands
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_bringup, 'launch', 'rviz2.launch.py')
            )
        )
 
        # Commands for rqt_image_view and rqt_plot
        rqt_image_view_cmd = ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        )
 
        rqt_plot_cmd = ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'],
            output='screen'
        )
 
        # Command to start teleop_keyboard in a new terminal
        teleop_keyboard_cmd = ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen'
        )
 
        # Create LaunchDescription and add actions
        ld = LaunchDescription()
        ld.add_action(gzserver_cmd)
        ld.add_action(gzclient_cmd)
        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)
        ld.add_action(rviz_cmd)
        ld.add_action(rqt_image_view_cmd)
        ld.add_action(rqt_plot_cmd)
        ld.add_action(teleop_keyboard_cmd)
 
        return ld
 
    except Exception as e:
        print(f"Error in generating launch description: {e}")
        raise
