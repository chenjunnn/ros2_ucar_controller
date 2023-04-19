#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Point
from tf.transformations import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import tf
import sys

class distanceCheckNode():
    def __init__(self):
        # Give the node a name
        rospy.init_node('distance_check_node', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        # Set the distance to travel
        if len(sys.argv) == 2: # 传入1个参数则默认是前进距离x
            try:
                self.test_dx = float(sys.argv[1])  
                self.test_dy = 0.0
            except Exception: # 读取参数失败则默认前进一米
                self.test_dx = 1.0 # meters
                self.test_dy = 0.0 # meters
        elif len(sys.argv) == 3: # 传入2个参数则分别对应x、y方向位移
            try:
                self.test_dx = float(sys.argv[1])
                self.test_dy = float(sys.argv[2])
            except Exception: # 读取参数失败则默认前进一米
                self.test_dx = rospy.get_param('~test_dx',   1.0) # meters
                self.test_dy = rospy.get_param('~test_dy',   0.0) # meters

        else:
            self.test_dx = rospy.get_param('~test_dx',   1.0) # meters
            self.test_dy = rospy.get_param('~test_dy',   0.0) # meters

        self.speed       = rospy.get_param('~speed', 0.15) # meters per second
        self.tolerance   = rospy.get_param('~tolerance', 0.01) # meters
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
  
        # self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position, self.orientation = self.get_pose()
        (roll, pitch, self.yaw) = euler_from_quaternion(self.orientation)
        # print(self.position)
        # print(self.orientation)
        # print(self.yaw)

        self.target_x = self.position[0] + math.cos(self.yaw) * self.test_dx - math.sin(self.yaw) * self.test_dy
        self.target_y = self.position[1] + math.sin(self.yaw) * self.test_dx + math.cos(self.yaw) * self.test_dy
        # x_start = self.position.x
        # y_start = self.position.y
        # move_cmd = Twist()
            
        while not rospy.is_shutdown():
            print("move_loop")
            # Stop the robot by default
            move_cmd = Twist()
            
            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position, self.orientation = self.get_pose()
                (roll, pitch, self.yaw) = euler_from_quaternion(self.orientation)
                
                # Compute the Euclidean distance from the target point
                distance = math.sqrt(pow((self.position[0] - self.target_x), 2) +
                                pow((self.position[1] - self.target_y), 2))
                print("distance: ", distance)
                # Correct the estimated distance by the correction factor
                # distance *= self.odom_linear_scale_correction
                
                # How close are we?
                # error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(distance) < self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    rospy.loginfo(params)

                else:
                    # If not, move in the appropriate direction
                    # move_cmd.linear.x = copysign(self.speed, -1 * error)
                    self.speed_theta = math.atan2((self.target_y - self.position[1]),(self.target_x - self.position[0])) - self.yaw
                    self.speed_theta += move_cmd.angular.z / self.rate / 2
                    move_cmd.linear.x = self.speed * math.cos(self.speed_theta)
                    move_cmd.linear.y = self.speed * math.sin(self.speed_theta)
            else:
                break
                # self.position = self.get_position()
                # x_start = self.position.x
                # y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def get_pose(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (trans, rot)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == "__main__":
    try:
        distanceCheckNode()
        rospy.spin()
    except Exception as e:
        print("TRY_ERROR: ")
        print(e)
