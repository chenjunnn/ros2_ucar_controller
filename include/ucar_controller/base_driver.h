#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <math.h>
#include <serial/serial.h>  //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <tf2_ros/transform_broadcaster.h>
#include <ucar_controller/crc_table.h>
#include <ucar_controller/data_struct.h>
#include <ucar_controller/fdilink_data_struct.h>

#include <fstream>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <ucar_interfaces/srv/get_battery_info.hpp>
#include <ucar_interfaces/srv/get_max_vel.hpp>
#include <ucar_interfaces/srv/set_led_mode.hpp>
#include <ucar_interfaces/srv/set_max_vel.hpp>

using namespace std;
#define ODOM_POSE_COVARIANCE {1e-3, 0, 0, 0, 0, 0,\
                              0, 1e-3, 0, 0, 0, 0,\
                              0, 0, 1e6, 0, 0, 0,\
                              0, 0, 0, 1e6, 0, 0,\
                              0, 0, 0, 0, 1e6, 0,\
                              0, 0, 0, 0, 0, 1e3}

#define ODOM_POSE_COVARIANCE2 {1e-9, 0, 0, 0, 0, 0,\
                              0, 1e-3, 1e-9, 0, 0, 0,\
                              0, 0, 1e6, 0, 0, 0,\
                              0, 0, 0, 1e6, 0, 0,\
                              0, 0, 0, 0, 1e6, 0,\
                              0, 0, 0, 0, 0, 1e-9}

#define ODOM_TWIST_COVARIANCE {1e-3, 0, 0, 0, 0, 0,\
                               0, 1e-3, 0, 0, 0, 0,\
                               0, 0, 1e6, 0, 0, 0,\
                               0, 0, 0, 1e6, 0, 0,\
                               0, 0, 0, 0, 1e6, 0,\
                               0, 0, 0, 0, 0, 1e3}

#define ODOM_TWIST_COVARIANCE2 {1e-9, 0, 0, 0, 0, 0,\
                                0, 1e-3, 1e-9, 0, 0, 0,\
                                0, 0, 1e6, 0, 0, 0,\
                                0, 0, 0, 1e6, 0, 0,\
                                0, 0, 0, 0, 1e6, 0,\
                                0, 0, 0, 0, 0, 1e-9}

namespace ucar_controller
{
#define Pi 3.1415926

#define WRITE_DATA_LONGTH 8
#define READ_MSG_LONGTH  14 // 13+1(new)
#define READ_DATA_LONGTH 12 // 11+1(new)
#define WRITE_MSG_LONGTH 16 // 13+3(new)
#define CS_LONGTH 1

//LED_MODE
#define LED_MODE_NORMAL  0
#define LED_MODE_BLINK   1
#define LED_MODE_BREATH  2

//MOTOR_MODE
#define MOTOR_MODE_JOY     0
#define MOTOR_MODE_CMD     1
#define MOTOR_MODE_MOVE    2

class UcarController : public rclcpp::Node
{
public:
  explicit UcarController(const rclcpp::NodeOptions &options);
  ~UcarController();
  void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool getMaxVelCB(const ucar_interfaces::srv::GetMaxVel::Request::SharedPtr req, const ucar_interfaces::srv::GetMaxVel::Response::SharedPtr res);
  bool setMaxVelCB(const ucar_interfaces::srv::SetMaxVel::Request::SharedPtr req, const ucar_interfaces::srv::SetMaxVel::Response::SharedPtr res);
  bool stopMoveCB (const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);
  bool getBatteryStateCB(const ucar_interfaces::srv::GetBatteryInfo::Request::SharedPtr req, const ucar_interfaces::srv::GetBatteryInfo::Response::SharedPtr res);
  bool setLEDCallBack(const ucar_interfaces::srv::SetLEDMode::Request::SharedPtr req, const ucar_interfaces::srv::SetLEDMode::Response::SharedPtr res);
  bool updateMileage(double vx, double vy, double dt);
  bool getMileage();
  void processBattery();
  void processLoop();
  void joyLoop();
  void writeLoop();
  void setWriteCS(int len);
  bool checkCS(int len);
  bool checkSN();
  void processOdometry();
  void processIMU(uint8_t head_type);
  void checkSN(int type);  // for imu
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);
  void setSerial();
  void openSerial();
  void callHandle();
  void updateSN();
  bool read_msg();

private:
  bool write_msg(double linear_x, double linear_y, double angular_z);

  std::thread pJoyThread_;
  std::thread processThread_;
  std::thread writeThread_;
  std::recursive_mutex Control_mutex_;
  
  //version
  std::string ws_version_;
  std::string hw_version_;
  std::string base_type_name_;

  //setting
  bool provide_odom_tf_, debug_log_;
  int controll_type_ ;
  bool joy_enable_;

  int encode_resolution_;
  double wheel_radius_;
  double period_;
  double wheel_avg_distance_;
  double vy_coefficient;
  
  //position
  
  //moving info
  double  linear_speed_min_;
  double angular_speed_min_;
  double  linear_speed_max_;
  double angular_speed_max_;
  
  //sum info
  double Mileage_sum_;
    double Mileage_last_;
  int sn_lost_  = 0;
  int cs_error_ = 0;
  uint32_t write_sn_ = 0;
  uint32_t read_sn_  = 0;
  
  //flages 
  bool read_first_;
  bool imu_frist_sn_;

  //serial
  std::string port_;
  int baud_;
  int serial_timeout_;
  int rate_;
  double duration_;

  //pose
  double x_,y_,th_;
  //vel
  nav_msgs::msg::Odometry current_odom_;

  //battery
  float current_battery_percent_;
  //led_values
  int   led_mode_type_;
  float led_frequency_;
  float led_red_value_;
  float led_green_value_;
  float led_blue_value_;
  double led_t_0;
  int led_timer;

  //data
  pack_write pack_write_;
  pack_read  pack_read_;

  //fdlink data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;

  //joy ctl
  double linear_gain_;
  double twist_gain_;
  double joy_linear_x_,  joy_linear_y_,  joy_angular_z_;
  double cmd_linear_x_,  cmd_linear_y_,  cmd_angular_z_;
  double move_linear_x_, move_linear_y_, move_angular_z_;
  double cmd_dt_threshold_;

  string Mileage_file_name_;
  string Mileage_backup_file_name_;
  
  //frame name
  string base_frame_, odom_frame_;
  string imu_frame_id_;

  //topic
  string vel_topic_, joy_topic_, odom_topic_;
  string battery_topic_;
  string imu_topic_, mag_pose_2d_topic_;

  // rclcpp time
  rclcpp::Time current_time_, last_time_;
  rclcpp::Time last_cmd_time_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Service<ucar_interfaces::srv::SetMaxVel>::SharedPtr set_max_vel_server_;
  rclcpp::Service<ucar_interfaces::srv::GetMaxVel>::SharedPtr get_max_vel_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_move_server_;
  rclcpp::Service<ucar_interfaces::srv::GetBatteryInfo>::SharedPtr get_battery_state_server_;
  rclcpp::Service<ucar_interfaces::srv::SetLEDMode>::SharedPtr set_led_server_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  serial::Serial serial_; //声明串口对象

};

}  // namespace ucar_controller

#endif
