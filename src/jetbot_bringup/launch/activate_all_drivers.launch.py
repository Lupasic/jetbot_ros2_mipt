from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare robot ID argument
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_id",
            default_value="1",
            description="Unique robot ID (1-5)"
        )
    )
    
    # Initialize Arguments
    robot_id = LaunchConfiguration("robot_id")
    robot_namespace = [TextSubstitution(text="robot_"), robot_id]

    # Publish a dummy message to create the topic for twist_stamped_to_twist
    topic_name = ['/robot_', robot_id, '/cmd_vel_robot_steering_stamped']
    message_type = 'geometry_msgs/msg/TwistStamped'
    message_content = '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"'
    
    create_topic_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', topic_name, message_type, message_content],
        shell=True
    )

# Needed to activate robot steering app before activation
    twist_stamped_to_twist = Node(
        package='topic_tools',
        executable='relay_field',
        name='twist_stamped_to_twist',
        # namespace=robot_namespace,
        arguments=[
            ['/robot_', robot_id, '/cmd_vel_robot_steering_stamped'],
            ['/robot_', robot_id, '/cmd_vel_robot_steering'],
            'geometry_msgs/msg/Twist',
            '{linear: m.twist.linear, angular: m.twist.angular}'
        ]
    )

    # Event handler to launch twist_stamped_to_twist after create_topic_cmd finishes
    launch_relay_after_topic_creation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_topic_cmd,
            on_exit=[twist_stamped_to_twist],
        )
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        namespace=robot_namespace,
        parameters=[{
            'dev': "/dev/input/js0"
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=robot_namespace,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "joy.yaml"
            ]),
            {'publish_stamped_twist': False}
        ],
        remappings=[('cmd_vel', 'cmd_vel_joy')]
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            namespace=robot_namespace,
            parameters=[PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "twist_mux.yaml"
            ])],
            remappings=[('cmd_vel_out', 'diffbot_base_controller/cmd_vel_unstamped')],
            # arguments=['--ros-args', '--log-level', 'debug']
        )

    # Lidar launch include with robot_id
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "launch", "rplidar.launch.py"
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items()
    )

    # Diffbot launch include with robot_id
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "launch", "diffbot.launch.py"
            ])
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        create_topic_cmd,
        launch_relay_after_topic_creation,
        #joy_node,
        #teleop_twist_joy_node,
        twist_mux,
        lidar_launch,
        diffbot_launch,
    ])
