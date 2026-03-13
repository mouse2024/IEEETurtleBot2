# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseWithCovarianceStamped as PoseWithCovarianceStamped

from geometry_msgs.msg import PoseStamped 
from nav2_msgs.action import NavigateToPose 
from rclpy.action import ActionClient 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub')

        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        qos = QoSProfile(depth=10)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos) #type of data, topic name, that qos nonsense
        msg = PoseWithCovarianceStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = -0.437
        msg.pose.pose.position.y = 0.1222
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.705
        msg.pose.pose.orientation.w = 0.705

        self.initial_pose_pub.publish(msg) #it doesn't show up on rviz, but it shows up on topic echo the same as the rviz one

        self.get_logger().info("message sent")

class GoToPoint(Node): 
   def __init__(self): 
       super().__init__('go_to_point') 
       self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose') 
 
   def go(self, x, y, yaw=0.0, frame='map'): 
       goal = NavigateToPose.Goal() 
       pose = PoseStamped() 
       pose.header.frame_id = frame 
       pose.header.stamp = self.get_clock().now().to_msg() 
       pose.pose.position.x = x 
       pose.pose.position.y = y 
       pose.pose.orientation.z = numpy.sin(yaw / 2.0) 
       pose.pose.orientation.w = numpy.cos(yaw / 2.0) 
       goal.pose = pose 
 
       self._client.wait_for_server() 
       return self._client.send_goal_async(goal) 

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    node = GoToPoint() 

    rclpy.spin(minimal_publisher)

    future = node.go(-0.36, 0.55, numpy.pi/2)   # m, m, rad 
    rclpy.spin_until_future_complete(node, future) 
    
    '''my rotate code would work better here
    future = node.go(-0.36, 0.55, 3*(numpy.pi/2))   # m, m, rad 
    rclpy.spin_until_future_complete(node, future) 
    '''

    '''
    future = node.go(-0.36, -0.5, 3*(numpy.pi/2))   # m, m, rad 
    rclpy.spin_until_future_complete(node, future) 
    '''
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
