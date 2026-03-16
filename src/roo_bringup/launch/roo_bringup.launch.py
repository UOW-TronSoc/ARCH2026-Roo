import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            default_value='/home/roo/roo_ws/web_dashboard',
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

        # Level 4: Serial transport (unified RX/TX bridge)
        Node(
            package='roo_serial',
            executable='serial_tx', # This is the script we just modified
            name='serial_bridge_node',
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
            condition=None,  
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
            executable='teleop',   
            name='roo_teleop_ps5',
            output='screen',
            parameters=[{
                'publish_hz': 50.0
            }],
        ),

# --- CAMERA 0 (FRONT) ---
        Node(
            package='roo_libcamera',
            executable='libcamera_node',
            name='camera_front',
            namespace='roo', # Added for consistency
            parameters=[{
                'camera_id': 0,
                'image_topic': '/roo/front/image', # Standardized path
                'width': 480, 'height': 360, 'framerate': 12
            }],
            respawn=True,
            respawn_delay=2.0,
        ),

        # --- CAMERA 1 (BACK) ---
        Node(
            package='roo_libcamera',
            executable='libcamera_node',
            name='camera_back',
            namespace='roo',
            parameters=[{
                'camera_id': 1,
                'image_topic': '/roo/back/image', # Standardized path
                'width': 480, 'height': 360, 'framerate': 12
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
        # # --- CAMERA 1 (BACK) ---
        # # NOTE: Disabled temporarily to resolve "Device or resource busy" error.
        # Node(
        #     package='roo_libcamera',
        #     executable='libcamera_node',
        #     name='camera_back',
        #     namespace='roo',
        #     parameters=[{
        #         'camera_id': 1,
        #         'image_topic': '/roo/back/image', # Standardized path
        #         'width': 480, 'height': 360, 'framerate': 12
        #     }],
        #     respawn=True,
        #     respawn_delay=2.0,
        # ),

        # --- CAMERA 2 (GIMBAL - USB) ---
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_gimbal',
            namespace='roo',
            parameters=[{
                'video_device': '/dev/roo_camera',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',       # Switched from MJPG to avoid the cv_bridge crash
                'output_encoding': 'rgb8',    # Explicitly set the output
                'time_per_frame' : [1, 30],  # Camera hardware only supports 30fps at 640x480 YUYV
            }],
            respawn=True,
            respawn_delay=2.0,
            remappings=[
                # Remap the node-local 'image_raw' to a global topic
                ('image_raw', '/roo/gimbal/image'),
            ]
        ),

        # --- Gimbal Image Compressor ---
        # The v4l2_camera_node outputs raw images. This node compresses them into
        # JPEG format so that web_video_server can stream them efficiently without
        # doing its own CPU-intensive compression.
        Node(
            package='roo_libcamera',
            executable='image_compressor',
            name='gimbal_compressor',
            parameters=[{'image_topic': '/roo/gimbal/image'}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # --- HTTP Video Streamer ---
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8081,
                'address': '0.0.0.0',
                'type':'ros_compressed',
                'ros_threads': 4
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
