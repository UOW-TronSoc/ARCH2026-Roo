from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/roo_esp32',   # stable symlink from your udev rule
            description='Serial device for ESP32'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Serial baud rate'
        ),

        # Level 4: Serial transport
        Node(
            package='roo_serial',
            executable='serial_rx',
            name='serial_rx_node',
            output='screen',
            parameters=[{'port': port, 'baud': baud}],
        ),
        Node(
            package='roo_serial',
            executable='serial_tx',
            name='serial_tx_node',
            output='screen',
            parameters=[{'port': port, 'baud': baud}],
        ),

        # Level 3: Protocol encode/decode
        Node(
            package='roo_protocol',
            executable='rx_decode',
            name='rx_decode_node',
            output='screen',
        ),
        Node(
            package='roo_protocol',
            executable='tx_encode',
            name='tx_encode_node',
            output='screen',
        ),
	Node(
	package='roo_cmd_mux',
	executable='cmd_mux',
        name='cmd_mux_node',
        output='screen',
        parameters=[
            {'default_source': 'joy'},
            {'timeout_s': 0.35},
            {'publish_hz': 50.0},
	],
	),

    ])
