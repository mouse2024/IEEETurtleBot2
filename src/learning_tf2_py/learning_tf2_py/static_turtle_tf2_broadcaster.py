#from here: https://github.com/ros/geometry_tutorials/blob/rolling/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('static_turtle_tf2_broadcaster')

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_z = 0.0
        self.last_quat_x = 0.0
        self.last_quat_y = 0.0
        self.last_quat_z = 0.0
        self.last_quat_w = 0.0

        self.need_init_odom_state = True

        qos = QoSProfile(depth=10) #basically TCP vs UDP, how much we care about getting all the msgs vs fast communication, more here: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos) #message type, topic name, function called when a message is received, quality of service go google it
        
        self.get_logger().info("got through init")
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        #self.make_transforms()

        self.get_logger().info("end of init")

    def odom_callback(self, msg): #function called when we get data from odometry subscription
        if(self.need_init_odom_state == True):
            self.get_logger().info("odom calledback")
            self.last_pose_x = msg.pose.pose.position.x
            self.last_pose_y = msg.pose.pose.position.y
            self.last_pose_z = msg.pose.pose.position.z
            self.last_quat_x = msg.pose.pose.orientation.x
            self.last_quat_y = msg.pose.pose.orientation.y
            self.last_quat_z = msg.pose.pose.orientation.z
            self.last_quat_w = msg.pose.pose.orientation.w

            self.need_init_odom_state = False #this tells us whether we should get the start pose or not

            self.make_transforms()

    def make_transforms(self):
        if(self.need_init_odom_state == False): #if we have already gotten our start pose
            self.get_logger().info("making transforms")
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'robot_start'

            t.transform.translation.x = self.last_pose_x
            t.transform.translation.y = self.last_pose_y
            t.transform.translation.z = self.last_pose_z
            t.transform.rotation.x = self.last_quat_x
            t.transform.rotation.y = self.last_quat_y
            t.transform.rotation.z = self.last_quat_z
            t.transform.rotation.w = self.last_quat_w

            self.tf_static_broadcaster.sendTransform(t)

            self.get_logger().info("sent transform")


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
