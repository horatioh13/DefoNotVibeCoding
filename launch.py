from rclpy.node import Node
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def generate_launch_description():
    node1 = Node(package="mission_control", executable="control_gui")
    node2 = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"autorepeat_rate": 0.0, "coalesce_interval_ms": 5, "deadzone": 0.1}],
    )
    return LaunchDescription([node1, node2])

if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()

