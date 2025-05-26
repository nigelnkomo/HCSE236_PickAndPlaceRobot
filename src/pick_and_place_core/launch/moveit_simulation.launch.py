import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro')
        .trajectory_execution(file_path='config/gripper_moveit_controllers.yaml')
        .to_moveit_configs()
    )

    move_group_capabilities = {
        'capabilities': 'move_group/ExecuteTaskSolutionCapability'
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('pick_and_place_core'),
        'launch',
        'mtc.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'world', 'panda_link0',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output='both',
    )

    load_controllers = []
    for controller in [
        'panda_arm_controller',
        'panda_hand_controller',
        'joint_state_broadcaster',
    ]:
        load_controllers.append(
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ] + load_controllers
    )

