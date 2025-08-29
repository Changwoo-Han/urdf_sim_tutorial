# 10-head.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 패키지와 URDF 경로 인자
    package_arg = DeclareLaunchArgument(
        'urdf_package',
        default_value='urdf_sim_tutorial',
        description='Package where the robot description is located'
    )

    model_arg = DeclareLaunchArgument(
        'urdf_package_path',
        default_value='urdf/10-firsttransmission.urdf.xacro',
        description='Path to the robot description relative to package root'
    )

    # RViz 설정
    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('urdf_sim_tutorial'), 'rviz', 'urdf.rviz'])
    )

    # Gazebo launch 포함
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('urdf_sim_tutorial'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')
        }.items()
    )

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    # joint_state_broadcaster 자동 실행
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # head_controller 자동 실행
    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'head_controller'],
        output='screen'
    )

    # diff_drive_base_controller 자동 실행
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    load_left_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_gripper_joint_controller'],
        output='screen'
    )

    load_right_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_gripper_joint_controller'],
        output='screen'
    )

    load_gripper_extension = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_extension_controller'],
        output='screen'
    )
    return LaunchDescription([
        package_arg,
        model_arg,
        rvizconfig_arg,
        gazebo_launch,
        rviz_node,
        load_joint_state_controller,
        load_head_controller,
        load_diff_drive_controller,
        load_left_gripper,
        load_right_gripper,
        load_gripper_extension,
    ])
