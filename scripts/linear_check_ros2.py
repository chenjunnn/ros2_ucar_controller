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

from tracemalloc import start
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Twist
import sys
import tf_transformations
from tf2_ros import LookupException, ConnectivityException, TransformException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


import math



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('distance_check_node')

        
        self.declare_parameter('time', 0.05)
        self.periodtime = self.get_parameter(
            'time').get_parameter_value().double_value
        self.get_logger().info('self.periodtime: "%f"' % self.periodtime)
       
        # self._loop_rate = self.create_rate(2.0)
        # self._loop_rate.sleep()
        # self.get_logger().info('hello 1')
        # while rclpy.ok():
        #     self.get_logger().info('hello 2')

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
                self.test_dx = 1.0 # meters
                self.test_dy = 0.0 # meters

        else:
            self.test_dx = 1.0 # meters
            self.test_dy = 0.0 # meters

        self.speed       =  0.40 # meters per second
        self.tolerance   = 0.01 # meters
        self.start_test = True
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Bring up rqt_reconfigure to control the test.")

        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel',  1)
      
        self.timer = self.create_timer(self.periodtime, self.timer_callback) # 单位是秒


        # from_frame_rel = 'base_link'
        # to_frame_rel = 'odom'
        # now = rclpy.time.Time()
        # when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
        # trans = self.tf_buffer.lookup_transform(
        #     to_frame_rel,
        #     from_frame_rel,
        #     when,
        #     timeout=rclpy.time.Duration(seconds=10))
        # self.get_logger().info('找到第一次tf')

        self.initial = True



        


    def timer_callback(self):
        if self.initial == True:
            self.initial = False
            self.get_logger().info('self.initial变成False')
            from_frame_rel = 'base_link'
            to_frame_rel = 'odom'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
            self.get_logger().info('第一次找到tf')
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            self.yaw = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
            self.get_logger().info('self.x："%f"' % self.x)
            self.get_logger().info('self.y："%f"' % self.y)
            self.get_logger().info('self.yaw："%f"' % self.yaw)
            self.target_x = self.x + math.cos(self.yaw) * self.test_dx - math.sin(self.yaw) * self.test_dy
            self.target_y = self.y + math.sin(self.yaw) * self.test_dx + math.cos(self.yaw) * self.test_dy

        else :
            print("move_loop, 20Hz")
            move_cmd = Twist()
            
            from_frame_rel = 'base_link'
            to_frame_rel = 'odom'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
            self.get_logger().info('找到tf')
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            self.yaw = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
            self.get_logger().info('self.x："%f"' % self.x)
            self.get_logger().info('self.y："%f"' % self.y)
            self.get_logger().info('self.yaw："%f"' % self.yaw)
            
            distance = math.sqrt(pow((self.x - self.target_x), 2) +
                                pow((self.y - self.target_y), 2))
            print("distance: ", distance)
            
            if not self.start_test or abs(distance) < self.tolerance:
                self.start_test = False
                self.get_logger().info('start_test："%d"' % self.start_test)
            else:
                # If not, move in the appropriate direction
                # move_cmd.linear.x = copysign(self.speed, -1 * error)
                self.speed_theta = math.atan2((self.target_y - self.y),(self.target_x - self.x)) - self.yaw
                self.speed_theta += move_cmd.angular.z * self.periodtime / 2 # 我改成乘号
                move_cmd.linear.x = self.speed * math.cos(self.speed_theta)
                move_cmd.linear.y = self.speed * math.sin(self.speed_theta)

            # move_cmd.linear.x = 2.0

            self.cmd_vel.publish(move_cmd)

    def get_pose(self):
        # Get the current transform between the odom and base frames
        try:
            from_frame_rel = 'base_link'
            to_frame_rel = 'odom'
            now = rclpy.time.Time()
            (trans, rot)  = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            now)
        except (Exception, ConnectivityException, LookupException):
            self.get_logger().info("TF Exception")
            return

        return (trans, rot)





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
