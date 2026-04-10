# so101_web_teleop.launch.py
#
# Starts:
#   1. rosbridge_websocket  — WebSocket ↔ ROS bridge used by the browser
#   2. web_teleop_node      — HTTP server + IK + session manager
#
# Run alongside the follower bringup and the leader teleop component:
#   ros2 launch so101_bringup so101_robot.launch.py
#   ros2 launch so101_teleop  so101_leader_teleop.launch.py
#   ros2 launch so101_web_teleop so101_web_teleop.launch.py
#
# HTTPS / mkcert quick-start (one-time, run on the host):
#   sudo apt install mkcert
#   mkcert -install
#   mkcert <robot-lan-ip>          # e.g. mkcert 192.168.1.42
#   mkdir -p ./certs
#   mv <robot-lan-ip>*.pem ./certs/robot.pem
#   mv <robot-lan-ip>*-key.pem ./certs/robot-key.pem
#
# Then set ssl_cert/ssl_key in config/so101_web_teleop.yaml and pass:
#   ros2 launch so101_web_teleop so101_web_teleop.launch.py \
#       http_port:=8443 ssl_cert:=/certs/robot.pem ssl_key:=/certs/robot-key.pem
#
# Android Chrome dev shortcut (no cert needed):
#   chrome://flags/#unsafely-treat-insecure-origin-as-secure
#   Add: http://<robot-ip>:8080

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── launch arguments ──────────────────────────────────────────────────
    http_port_arg = DeclareLaunchArgument(
        'http_port', default_value='8080',
        description='Port for the embedded HTTP/HTTPS web server',
    )
    ws_port_arg = DeclareLaunchArgument(
        'ws_port', default_value='9090',
        description='Port for rosbridge WebSocket server',
    )
    ssl_cert_arg = DeclareLaunchArgument(
        'ssl_cert', default_value='',
        description='Path to TLS certificate (PEM); leave empty for plain HTTP',
    )
    ssl_key_arg = DeclareLaunchArgument(
        'ssl_key', default_value='',
        description='Path to TLS private key (PEM); leave empty for plain HTTP',
    )

    http_port = LaunchConfiguration('http_port')
    ws_port   = LaunchConfiguration('ws_port')
    ssl_cert  = LaunchConfiguration('ssl_cert')
    ssl_key   = LaunchConfiguration('ssl_key')
    ssl_enabled = PythonExpression(["'", ssl_cert, "' != '' and '", ssl_key, "' != ''"])

    # ── rosbridge websocket — launched directly as a Node to avoid XML include issues ──
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port':              ws_port,
            'ssl':               ssl_enabled,
            'certfile':          ssl_cert,
            'keyfile':           ssl_key,
            'authenticate':      False,
            'max_message_size':  10000000,
        }],
    )

    # ── web teleop node ────────────────────────────────────────────────────
    config = PathJoinSubstitution([
        FindPackageShare('so101_web_teleop'),
        'config',
        'so101_web_teleop.yaml',
    ])

    web_node = Node(
        package='so101_web_teleop',
        executable='web_teleop_node',
        name='web_teleop_node',
        output='screen',
        parameters=[
            config,
            {
                'http_port': http_port,
                'ssl_cert':  ssl_cert,
                'ssl_key':   ssl_key,
            },
        ],
    )

    return LaunchDescription([
        http_port_arg,
        ws_port_arg,
        ssl_cert_arg,
        ssl_key_arg,
        rosbridge_node,
        web_node,
    ])
