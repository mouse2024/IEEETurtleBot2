import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
from nav2_msgs.action import NavigateToPose 
from rclpy.action import ActionClient 
import numpy
 
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
   node = GoToPoint() 
   future = node.go(-0.36, 0.55, numpy.pi/2)  # m, m, rad 
   rclpy.spin_until_future_complete(node, future) 
   rclpy.shutdown() 
 
if __name__ == '__main__': 
   main() 
