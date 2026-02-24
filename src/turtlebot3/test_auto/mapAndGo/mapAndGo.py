import math
import os
import sys
import termios

import signal #for sigint

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


def generate_launch_description():
    tb3_cart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            )
        )
    )


ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg

class SigintSkipper:
    def __init__(self, callback):
        self.callback = callback
    def __enter__(self):
        self.got = False
        self.handler_old = signal.signal(signal.SIGINT, self.handler)

    def handler(self, sig, frame):
        self.got = (sig, frame)
        self.callback()

    def __exit__(self, type, value, traceback):
        print('exiting sigint skipper')

class Turtlebot3CreateMap(Node):
    def __init__(self):
        super().__init__('turtlebot3_create_map')

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False

        qos = QoSProfile(deth=10)

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos
        )

        generate_launch_description

        self.update_timer = self.create_timer(0.010, self.update_callback)

        self.get_logger().info('TurtleBot3 create map node has been initialized.')


    def generate_stop(self):
        self.get_logger().info('generte stop is running')
        twist = CmdVelMsg()
        if not self.init_odom_state:
            return
        
        if not self.get_key_state:
            self.get_key_state = True

        else:
            angle = self.last_pose_theta
            angular_velocity = 0.0

            self.get_logger().info('stop now')
            twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3CreateMap()

    def finish_callback():
        node.generate_stop()
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

    with SigintSkipper(finish_callback):
        rclpy.spin(node)

if __name__ == '__main__':
    main()