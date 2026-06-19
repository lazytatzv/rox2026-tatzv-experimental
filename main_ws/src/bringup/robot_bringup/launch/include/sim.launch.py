# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = LaunchConfiguration('gazebo')
    headless = LaunchConfiguration('headless', default='false')

    display_env = os.environ.get('DISPLAY', ':0')
    set_display_cmd = SetEnvironmentVariable(name='DISPLAY', value=display_env)
    
    # Qtの描画プラットフォームとして xcb (X11) を指定
    set_qt_cmd = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb')

    models_path = os.path.join(pkg_robot_bringup, 'models')
    
    # Set GZ_SIM_RESOURCE_PATH to include our models
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':',
            models_path
        ]
    )

    world_path = os.path.join(pkg_robot_bringup, 'world', 'rox2026_field_cad.sdf')

    # Construct gz_args: -r (run), -v 4 (debug), and optionally -s (server/headless)
    gz_args = PythonExpression([
        "'-r -v 4 ' + '", world_path, "' + (' -s' if '", headless, "' == 'true' else '')"
    ])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': gz_args
        }.items(),
        condition=IfCondition(gazebo)
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rox2026', '-topic', 'robot_description'],
        condition=IfCondition(gazebo)
    )

    return LaunchDescription([
        set_display_cmd,
        set_qt_cmd,
        set_gz_resource_path,
        gazebo_sim,
        spawn_robot
    ])
