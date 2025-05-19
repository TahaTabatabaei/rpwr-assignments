from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    turtles = [
        ('turtle1', None),       # manually spawned by turtlesim_node
        ('turtle2', 'turtle1'),
        ('turtle3', 'turtle2'),
        ('turtle4', 'turtle3'),
    ]

    nodes = [
        # Launch turtlesim GUI
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
    ]

    # Add broadcasters and followers
    for turtle_name, target in turtles:
        # TF2 broadcaster node
        nodes.append(
            Node(
                package='turtle_party',
                executable='turtle_tf2_broadcaster',
                name=f'{turtle_name}_broadcaster',
                parameters=[{'turtlename': turtle_name}]
            )
        )

        # Skip follower node for turtle1 (it's manually controlled)
        if target is not None:
            nodes.append(
                Node(
                    package='turtle_party',
                    executable='follow',
                    name=f'{turtle_name}_follower',
                    parameters=[
                        {'turtle_name': turtle_name},
                        {'target_frame': target}
                    ],
                    output='screen'
                )
            )

    return LaunchDescription(nodes)
