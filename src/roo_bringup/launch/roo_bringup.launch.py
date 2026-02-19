from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    # Web dashboard args
    web_enable = LaunchConfiguration('web_enable')
    web_dir = LaunchConfiguration('web_dir')
    web_port = LaunchConfiguration('web_port')

    # WebBridge args
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/roo_esp32',  # stable symlink from your udev rule
            description='Serial device for ESP32'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Serial baud rate'
        ),

        # ---- Web dashboard server (static HTML) ----
        DeclareLaunchArgument(
            'web_enable',
            default_value='true',
            description='Enable serving the dashboard HTML from the Pi'
        ),
        DeclareLaunchArgument(
            'web_dir',
            default_value='/home/roo/roo_ros2/web_dashboard',
            description='Directory containing index.html'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='HTTP port for the dashboard'
        ),

        # ---- WebBridge (websocket) ----
        DeclareLaunchArgument(
            'ws_host',
            default_value='0.0.0.0',
            description='WebBridge bind host'
        ),
        DeclareLaunchArgument(
            'ws_port',
            default_value='8765',
            description='WebBridge websocket port'
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

        # Command mux
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

        # WebBridge node (publishes /roo/web_cmd/* etc)
        Node(
            package='roo_web_bridge',
            executable='web_bridge',
            name='web_bridge',
            output='screen',
            parameters=[
                {'ws_host': ws_host},
                {'ws_port': ws_port},
            ],
        ),

        # Static web server for dashboard
        ExecuteProcess(
            condition=None,  # keep simple: if you want a real conditional later, tell me
            cmd=[
                'python3', '-m', 'http.server', web_port,
                '--directory', web_dir
            ],
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.12,
                'autorepeat_rate': 50.0,
            }],
        ),

        Node(
            package='roo_teleop_ps5',
            executable='teleop',   # <-- IMPORTANT: confirm with `ros2 pkg executables roo_teleop_ps5`
            name='roo_teleop_ps5',
            output='screen',
            parameters=[{
                'publish_hz': 50.0
            }],
        ),
    ])
