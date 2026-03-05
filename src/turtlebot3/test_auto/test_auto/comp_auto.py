#from https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_example/turtlebot3_example/turtlebot3_relative_move/turtlebot3_relative_move.py

'''
As the robot drives through each square, mark it as travelled. We will know this based on data from /odom. 
Maybe we will need to relocalize at some point with lidar or just the laser (laser's topic is called /scan and there’s a video for it), -- this is what Kalman filters are for, https://github.com/kimsooyoung/kalman_filter_ros2_tutorial/blob/main/kalman_filter/kalman_filter/kalman_filter_solution.py
especially if the asteroids throw us off course, we’ll need to test it. 
First thing is to drop off our beacon if we have one, and we’ll get the aprilTag ID at the same time if we’re using it. 
Then we head straight for the cave, no matter how many asteroids we do or don’t have 
(ideally we could dump some on the way if we are reliable enough, but i don’t think we will be). 
As soon as we touch that square inside the cave, we can go back and dump whatever geodinium we’ve collected into the close CSC. 
Let’s see how many asteroids our robot can hold, how long we can drive around in the cave, hopefully even outside it, before we have to go dump again. 
Maybe we could do a fancy math equation and calculate how much time we have left vs how far we currently are from the rendezvous pads 
to decide when exactly we need to leave to move the CSCs, but probably by the last 30 seconds (endgame anyone??? lol) 
'''
import math
import os
import sys
import termios

import signal #for sigint

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg

rows, cols = (4, 8)
field = [[0 for i in range(cols)] for j in range(rows)]
field[3][2] = 1 #starting sqaure (there are obviously no astroids here, but whatever)

#I copied this class exactly from Michael's multicontrol mynode, idk where he got it from; used to hopefully help stop the idiot at the end
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

class Turtlebot3RelativeMove(Node):
    def intom(inch): #inch to meter conversion, bc IEEE uses in and turtlebot uses m
        return inch * 0.0254
    def degtorad(deg):
        return deg * 3.141592 / 180

    def worldtostart(self, x, y, theta):
        
        x = x + self.start_pose_x
        y = y + self.start_pose_y
        theta = theta + self.start_pose_theta
        return (x, y, theta)
    
    def starttoworld(self, x, y, theta):
        x = x - self.start_pose_x
        y = x - self.start_pose_y
        theta = theta - self.start_pose_theta
        return (x, y, theta)

    #I am not dealing with more python nonsense, just go with it
    #states = [(intom(12), 0, 0, "x"), (intom(12), 0, degtorad(-90), "theta"), (intom(12), intom(12), degtorad(-90), "y"), (intom(12), intom(12), degtorad(-180), "theta"), (0, intom(12), degtorad(-180), "x"), (0, intom(12), degtorad(90), "theta"), (0, 0, degtorad(90), "y"), (0, 0, 0, "theta")]
    state = 0
    states = [(intom(12), 0, 0, "x")]

    def __init__(self):
        super().__init__('turtlebot3_relative_move')

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.start_pose_x = 0.0
        self.start_pose_y = 0.0
        self.start_pose_theta = 0.0

        self.odom_reset = False #I'm going to use this variable to reset odometry at the beginning of the code
        self.get_key_state = False # whether we have new user input to work off of
        self.init_odom_state = False # whether we have new odometry data to work off of

        qos = QoSProfile(depth=10) #basically TCP vs UDP, how much we care about getting all the msgs vs fast communication, more here: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos) #type of data, topic name, that qos nonsense

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos) #message type, topic name, function called when a message is received, qos see above

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'robot_start', 'base_link').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.update_timer = self.create_timer(0.010, self.update_callback) #call that function every 0.01 seconds, i think it does start automatically, idk why it needs to go into a variable

        self.get_logger().info('TurtleBot3 relative move node has been initialised.') #log a message with INFO severity (into the log file i guess, you can find it somewhere in rviz? go ask the ros tutorial)
    #this came from here https://stackoverflow.com/questions/74976911/create-an-odometry-publisher-node-in-python-ros2
    #its gone, i dont think it worked
    #this sets of previous (current) position based on the odometry data (better hope its correct)
    def odom_callback(self, msg): #function called when we get data from odometry subscription
        if(not self.odom_reset): 
            self.start_pose_x = msg.pose.pose.position.x
            self.start_pose_y = msg.pose.pose.position.y
            self.start_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
            self.odom_reset = True

        #this puts our numbers back as if we started at 0,0,0
        self.last_pose_x, self.last_pose_y, self.last_pose_theta = self.starttoworld(msg.pose.pose.position.x, msg.pose.pose.position.y, self.euler_from_quaternion(msg.pose.pose.orientation)[2])

        self.get_logger().info('msg data ' + str(msg.pose.pose.position.x) + " " + str(msg.pose.pose.position.y) + ' ' + str(self.euler_from_quaternion(msg.pose.pose.orientation)[2]))
        self.get_logger().info('mtd data ' + str(self.last_pose_x) + " " + str(self.last_pose_y) + ' ' + str(self.last_pose_theta))
        self.init_odom_state = True #this tells us whether we should trust the data in last_pose

    #if we have new odometry data, make a new path
    def update_callback(self): #called from the timer every 0.01 seconds
        #self.get_logger().info('update calledback')
        if self.init_odom_state: #if we have updated odometry data
            self.generate_path()

    def generate_path(self):
        twist = CmdVelMsg() #the message we will eventually publish to move
        if self.state > len(self.states):
            self.get_logger().info('no more states')
            twist.linear.x = 0
            twist.angular. z = 0
        elif not self.init_odom_state: #if we don't have new odometry data, no reason to move
            self.get_logger().info('no new odom')
            return
        elif not self.get_key_state:
            #self.goal_pose_x, self.goal_pose_y, self.goal_pose_theta = self.states[self.state][0:3]
            input_x, input_y, input_theta = self.states[self.state][0:3]

            '''self.get_logger().info('set goal poses')
            input_x_global = ( #converting local input into global frame (i guess this will help)
                math.cos(self.start_pose_theta) * input_x - math.sin(self.start_pose_theta) * input_y
            )
            input_y_global = (
                math.sin(self.start_pose_theta) * input_x + math.cos(self.start_pose_theta) * input_y
            )

            self.goal_pose_x = self.last_pose_x + input_x_global
            self.goal_pose_y = self.last_pose_y + input_y_global
            self.goal_pose_theta = self.last_pose_theta + input_theta

            self.goal_pose_x = input_x - self.start_pose_x
            self.goal_pose_y = input_y - self.start_pose_y
            self.goal_pose_theta = input_theta - self.start_pose_theta '''

            #self.goal_pose_x, self.goal_pose_y, self.goal_pose_theta = self.worldtostart(input_x, input_y, input_theta)
            try: 
                #lookup transform between robot_start and odom
                t = self.tf_buffer.lookup_transform(
                    'odom', #to frame rel (target frame)
                    'robot_start', #from frame rel (source frame)
                    rclpy.time.Time())
                self.get_logger().info(str(t))
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform: {ex}')
                return
            
            self.goal_pose_x = input_x + t.transform.translation.x
            self.goal_pose_y = input_y + t.transform.translation.y
            self.goal_pose_theta = input_theta + self.euler_from_quaternion(t.transform.rotation)[2]

            self.get_logger().info('input data ' + str(input_x) + " " + str(input_y) + ' ' + str(input_theta))
            self.get_logger().info('trans data ' + str(t.transform.translation.x) + " " + str(t.transform.translation.y) + ' ' + str(self.euler_from_quaternion(t.transform.rotation)[2]))
            self.get_logger().info('mathd data ' + str(self.goal_pose_x) + " " + str(self.goal_pose_y) + ' ' + str(self.goal_pose_theta))

            self.get_key_state = True #this indicates if we have new user input to move based on

        else:
            self.get_logger().info('array state ' + str(self.state))
            self.get_logger().info('what should we do ' + str(self.states[self.state][3]))
            #make goal between 0 and 2pi
            if self.goal_pose_theta > 6.3: #does it really need to be while?
                self.goal_pose_theta = self.goal_pose_theta - 6.28
                self.get_logger().info('subtracted 2pi from goal')
            if self.goal_pose_theta < -0.1:
                self.goal_pose_theta = self.goal_pose_theta + 6.28
                self.get_logger().info('added 2pi to goal')
            #make current between 0 and 2pi
            if self.last_pose_theta > 6.3: #does it really need to be while?
                self.last_pose_theta = self.last_pose_theta - 6.28
                self.get_logger().info('subtracted 2pi from last')
            if self.last_pose_theta < -0.1:
                self.last_pose_theta = self.last_pose_theta + 6.28
                self.get_logger().info('added 2pi to last')
            
            deltay = self.goal_pose_y - self.last_pose_y
            deltax = self.goal_pose_x - self.last_pose_x
            deltat = self.goal_pose_theta - self.last_pose_theta
            #straight
            if (self.states[self.state][3] == "y") and abs(deltay) > 0.1:
                if (deltay > 0.1): #this may need to change for precision's sake
                    twist.linear.x = -0.075 #change this for speed
                    self.get_logger().info('y (goal, curr) ' + str(self.goal_pose_y) + ' ' + str(self.last_pose_y)) #add telemetry
                elif (deltay < -0.1):
                    twist.linear.x = 0.075 #apparently x is robot centric forward
                    self.get_logger().info('-y (goal, curr) ' + str(self.goal_pose_y) + ' ' + str(self.last_pose_y))

            elif (self.states[self.state][3] == "x") and abs(deltax) > 0.1:
                if (deltax > 0.1): #this may need to change for precision's sake
                    twist.linear.x = 0.075 #change this for speed
                    self.get_logger().info('x (goal, curr) ' + str(self.goal_pose_x) + ' ' + str(self.last_pose_x)) #add telemetry
                elif (deltax) < -0.1:
                    twist.linear.x = -0.075
                    self.get_logger().info('-x (goal, curr) ' + str(self.goal_pose_x) + ' ' + str(self.last_pose_x))

            #turn
            elif (self.states[self.state][3] == "theta") and abs(deltat) > 0.1:
                if deltat > 0.05: #this number may need to change
                    twist.angular.z = 0.5 #can't flip this or it will oscillate around -1.5 (pi/2)
                    self.get_logger().info('z (goal, curr) ' + str(self.goal_pose_theta) + ' ' + str(self.last_pose_theta))
                elif deltat < -0.05: #if it drifted slightly, that's ok, we're ignoring that
                    twist.angular.z = -0.5
                    self.get_logger().info('-z (goal, curr) ' + str(self.goal_pose_theta) + ' ' + str(self.last_pose_theta))
            else:
                self.state = self.state + 1
                self.get_key_state = False 

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)

    #converts angle representations, what fun.
    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args) #just write this, idk or care what exactly it does
    node = Turtlebot3RelativeMove() #this calls init from above

    #this code is all from Michael's multicontrol mynode, except i moved it outside main()
    def finish_callback():
        #node.generate_stop()
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist) #publish an empty cmdVelMsg to stop
        node.destroy_node()
        rclpy.shutdown()

    with SigintSkipper(finish_callback):
        #threading.Thread(target=pyglet.app.run, args=tuple()).start() #I think this is just for controller input and i can ignore it
        rclpy.spin(node)

if __name__ == '__main__': #this just lives here in python
    main()
