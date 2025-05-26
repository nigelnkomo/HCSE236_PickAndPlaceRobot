from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder('moveit_resources_panda').to_dict()

    pick_and_place_node = Node(
        package='pick_and_place_core',
        executable='pick_and_place_node',
        output='screen',
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_and_place_node])

