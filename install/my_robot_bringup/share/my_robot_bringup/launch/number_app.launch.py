from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_config = os.path.join(get_package_share_directory('my_robot_bringup'),
                                "config",
                                "number_app.yaml")

    number_publisher_node = Node(
        package='my_py_pkg',
        executable='number_publisher',
        namespace="/robot1",
        name='my_number_publisher',
        remappings=[('number', 'my_number')],
        # parameters=[
        #     {'number': 6},
        #     {'time_period': 0.5}
        # ]
        parameters=[param_config]
    )

    number_counter_node = Node(
        package='my_cpp_pkg',
        executable='number_counter',
        namespace="/robot1",
        name='my_number_counter',
        remappings=[('number', 'my_number')]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld