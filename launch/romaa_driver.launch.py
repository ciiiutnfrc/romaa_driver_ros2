from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    frequency_arg = DeclareLaunchArgument(
        'frequency', default_value='10.0',
        description='Publisher frequency')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port device')

    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='Serial port baud-rate')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Odometry frame')

    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot frame')

    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor', default_value='False',
        description='Enable/disable motors at node startup')

    reset_odom_arg = DeclareLaunchArgument(
        'reset_odom', default_value='False',
        description='Reset odometry at node startup')

    romaa_driver_node = Node(
        package = 'romaa_driver',
        name = 'romaa_driver',
        executable = 'romaa_driver',
        parameters = [
            {"frequency": LaunchConfiguration('frequency')},
            {"port": LaunchConfiguration('port')},
            {"baudrate": LaunchConfiguration('baudrate')},
            {"odom_frame": LaunchConfiguration('odom_frame')},
            {"base_frame": LaunchConfiguration('base_frame')},
            {"enable_motor": LaunchConfiguration('enable_motor')},
            {"reset_odom": LaunchConfiguration('reset_odom')}
        ],
        output="screen"
    )

    return LaunchDescription([
        frequency_arg,
        port_arg,
        baudrate_arg,
        odom_frame_arg,
        base_frame_arg,
        enable_motor_arg,
        reset_odom_arg,
        romaa_driver_node
    ])
