import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
import sys
import tf_transformations
from tf2_ros import LookupException, ConnectivityException, TransformException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class angleCheckNodeV2(Node):
    def __init__(self):
        # Give the node a name
        super().__init__('angle_check_node')

        self.imu_sub = self.create_subscription(Imu, "/imu", self.imutopic,1)
        self.imu_sub  # prevent unused variable warning 

        self.target_yaw = math.pi
        self.r_speed     = 1.0 # rad per second
        self.r_tolerance = 0.10 # meters
        # self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = True
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Bring up rqt_reconfigure to control the test.")

        # Publisher to control the robot's speed
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.05, self.timer_callback) # 单位是秒
        self.imu_data = 0
        self.cntFre = 0
        self.iniPose = True

    def timer_callback(self):
        # print("odomCB, 20Hz")   
        
        from_frame_rel = 'base_link'
        to_frame_rel = 'odom'
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            now)
        self.yaw = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
        if self.iniPose == True:
            self.get_logger().info('初始位置')
            self.get_logger().info('self.yaw："%f"' % self.yaw)
            self.get_logger().info('self.imu_data"%f"' % self.imu_data)
            self.iniPose=False

        if self.cntFre == 10:
            self.cntFre = 0
            self.get_logger().info('self.yaw："%f"' % self.yaw)
            self.get_logger().info('self.imu_data"%f"' % self.imu_data)
        self.cntFre = self.cntFre + 1
        

        self.d_yaw = self.target_yaw - self.yaw
        # print("d_yaw: ",self.d_yaw)
        if abs(self.d_yaw) < self.r_tolerance or not self.start_test:
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            self.get_logger().info('停止')
            self.get_logger().info('self.yaw："%f"' % self.yaw)
            self.get_logger().info('self.imu_data"%f"' % self.imu_data)
            self.target_yaw = 0
            return
        else:
            move_cmd = Twist()
            if self.d_yaw < 0:
                move_cmd.angular.z = -1 * self.r_speed
            else:
                move_cmd.angular.z =      self.r_speed
            self.cmd_vel.publish(move_cmd)

    def imutopic(self, data):
        # print("imutopic")
        self.imu_data = tf_transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2]
        # self.get_logger().info('self.imu_data"%f"' % self.imu_data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = angleCheckNodeV2()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()