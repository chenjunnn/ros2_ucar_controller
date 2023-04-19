#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ucar_controller/base_driver.h>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/utilities.hpp>

namespace ucar_controller
{
UcarController::UcarController(const rclcpp::NodeOptions & options)
: Node("ucar_controller", options), x_(0), y_(0), th_(0)
{
  RCLCPP_INFO(get_logger(), "Declaring params");

  provide_odom_tf_ = this->declare_parameter("provide_odom_tf", true);
  vel_topic_ = this->declare_parameter("vel_topic", std::string("/cmd_vel"));  ///smooth_cmd_vel
  joy_topic_ = this->declare_parameter("joy_topic", std::string("/joy"));
  odom_topic_ = this->declare_parameter("odom_topic", std::string("/odom"));
  battery_topic_ = this->declare_parameter("battery_topic", std::string("/battery_state"));

  // serial
  port_ = this->declare_parameter("port", std::string("/dev/base_serial_port"));
  baud_ = this->declare_parameter("baud", 921600);
  serial_timeout_ = this->declare_parameter("serial_timeout", 50);  // ms
  rate_ = this->declare_parameter("rate", 20);                      // hz
  duration_ = this->declare_parameter("duration", 0.01);
  cmd_dt_threshold_ = this->declare_parameter("cmd_timeout", 0.2);

  base_frame_ = this->declare_parameter("base_frame", std::string("base_footprint"));
  odom_frame_ = this->declare_parameter("odom_frame", std::string("odom"));

  encode_resolution_ = this->declare_parameter("encode_resolution", 270);  //
  wheel_radius_ = this->declare_parameter("wheel_radius", 0.04657);        // m
  period_ = this->declare_parameter("period", 50.0);                       // ms
  wheel_avg_distance_ = this->declare_parameter("wheel_avg_distance", 0.2169);  // m
  vy_coefficient = this->declare_parameter("vy_coefficient", 0.975);
  
  linear_speed_max_ = this->declare_parameter("linear_speed_max", 3.0);     // m/s
  angular_speed_max_ = this->declare_parameter("angular_speed_max", 3.14);  // rad/s

  Mileage_file_name_ =
    this->declare_parameter("Mileage_file_name", std::string("car_Mileage_info.txt"));
  Mileage_file_name_ = ament_index_cpp::get_package_share_directory("ucar_controller") +
                       "/log_info/" + Mileage_file_name_;
  Mileage_backup_file_name_ = Mileage_file_name_ + ".bp";
  debug_log_ =
    this->declare_parameter("debug_log", true);  //  true rclcpp info 等等  打印log数据

  imu_topic_ = this->declare_parameter("imu_topic", std::string("/imu"));
  imu_frame_id_ = this->declare_parameter("imu_frame", std::string("imu"));
  mag_pose_2d_topic_ =
    this->declare_parameter("mag_pose_2d_topic", std::string("/mag_pose_2d"));
  read_first_ = false;
  imu_frist_sn_ = false;
  controll_type_ = MOTOR_MODE_CMD; // 1:vel_mode 0:joy_node
  linear_gain_   = 0.3;
  twist_gain_    = 0.7;
  linear_speed_min_  = 0;
  angular_speed_min_ = 0;
  current_battery_percent_ = -1;
  led_mode_type_   = 0;
  led_frequency_   = 0;
  led_red_value_   = 0;
  led_green_value_ = 0;
  led_blue_value_  = 0;

  getMileage();

  RCLCPP_INFO(get_logger(), "Creating publishers and subscribers");
  using std::placeholders::_1;
  using std::placeholders::_2;

  odom_pub_    = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_,10);
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic_,10);
  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    vel_topic_, 1, std::bind(&UcarController::velCallback, this, _1));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, 1, std::bind(&UcarController::joyCallback, this, _1));
  imu_pub_     = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  mag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);

  RCLCPP_INFO(get_logger(), "Creating services");
  stop_move_server_ = this->create_service<std_srvs::srv::Trigger>(
    "stop_move", std::bind(&UcarController::stopMoveCB, this, _1, _2));
  set_max_vel_server_ = this->create_service<ucar_interfaces::srv::SetMaxVel>(
    "set_max_vel", std::bind(&UcarController::setMaxVelCB, this, _1, _2));
  get_max_vel_server_ = this->create_service<ucar_interfaces::srv::GetMaxVel>(
    "get_max_vel", std::bind(&UcarController::getMaxVelCB, this, _1, _2));
  get_battery_state_server_ = this->create_service<ucar_interfaces::srv::GetBatteryInfo>(
    "get_battery_state", std::bind(&UcarController::getBatteryStateCB, this, _1, _2));
  set_led_server_ = this->create_service<ucar_interfaces::srv::SetLEDMode>(
    "set_led_light", std::bind(&UcarController::setLEDCallBack, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Creating odom_broadcaster");
  odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Setting serial port");
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none); //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    serial_.open(); 
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port "); 
    exit(0); 
  }
  if(serial_.isOpen()) 
  { 
    RCLCPP_INFO(this->get_logger(), "Serial Port initialized"); 
  }else{ 
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to initial Serial port "); 
    exit(0);
  } 
  current_time_ = this->now();
  last_time_ = this->now();
  last_cmd_time_ = this->now();
  setSerial();
  openSerial();
  writeThread_   = std::thread(std::bind(&UcarController::writeLoop,   this));
  processThread_ = std::thread(std::bind(&UcarController::processLoop, this));

  RCLCPP_INFO(this->get_logger(), "ucar_controller Ready!");
}

void UcarController::setSerial()
{
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    // serial_.open(); 
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "AIcarController setSerial failed, try again!");
    RCLCPP_ERROR(this->get_logger(), "AIcarController setSerial: %s", e.what());
    setSerial();
  }
  catch (...)
  {
    RCLCPP_ERROR(this->get_logger(), "AIcarController setSerial failed with unknow reason, try again!");
    setSerial();
  }
  rclcpp::sleep_for(std::chrono::milliseconds(200));
}

void UcarController::openSerial()
{
  bool first_open = true;
	while(rclcpp::ok())
  {
    try
    {
      if(serial_.isOpen()==1)
      {
        RCLCPP_INFO(this->get_logger(), "AIcarController serial port open success.\n");
        return;
      }
      else
      {
        if (first_open){
          RCLCPP_INFO(this->get_logger(), "AIcarController openSerial: start open serial port\n");
        } 
        serial_.open();
      }
    }
    catch(const std::exception& e)
    {
      if (first_open)
      {
        RCLCPP_ERROR(this->get_logger(), "AIcarController openSerial: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "AIcarController openSerial: unable to open port, keep trying\n");
        // std::cerr << e.what() << '\n';
      }
    }
    catch(...)
    {
      if (first_open)
      {
        RCLCPP_ERROR(this->get_logger(), "AIcarController openSerial: unable to open port with unknow reason, keep trying\n");
      }
    }
    first_open = false;
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
}//openSerial

void UcarController::writeLoop()
{
  RCLCPP_INFO(this->get_logger(), "baseBringup::writeLoop: start");
  led_timer = 0;
  rclcpp::Rate loop_rate(rate_);
  while(rclcpp::ok())
  {
    try
    {
      std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
      int cur_controll_type = controll_type_;
      lock.unlock();
      double linear_x = 0.0;
      double linear_y = 0.0;
      double angular_z = 0.0;
      switch (cur_controll_type)
      {
        case MOTOR_MODE_JOY:{
          lock.lock();
          linear_x  = joy_linear_x_;
          linear_y  = joy_linear_y_;
          angular_z = joy_angular_z_;
          lock.unlock();
          break;
        }
        case MOTOR_MODE_CMD:{
          lock.lock();
          double dt = (this->now() - last_cmd_time_).seconds();
          if (dt > cmd_dt_threshold_)
          {
            linear_x  = 0;
            linear_y  = 0;
            angular_z = 0;
          }        
          else
          {
            linear_x  = cmd_linear_x_;
            linear_y  = cmd_linear_y_;
            angular_z = cmd_angular_z_;
          }
          lock.unlock();
          break;
        }
        case MOTOR_MODE_MOVE:{
          lock.lock();
          linear_x  = move_linear_x_;
          linear_y  = move_linear_y_;
          angular_z = move_angular_z_;
          lock.unlock();
          break;
        }
        default:
          RCLCPP_ERROR(this->get_logger(), "base_driver-writeLoop: controll_type_ error!");
          break;
      }
      if(linear_x > linear_speed_max_)
        linear_x = linear_speed_max_;
      else if(linear_x < -linear_speed_max_)
        linear_x = -linear_speed_max_;
      
      if(linear_y > linear_speed_max_)
        linear_y = linear_speed_max_;
      else if(linear_y < -linear_speed_max_)
        linear_y = -linear_speed_max_;

      if(angular_z > angular_speed_max_)
        angular_z = angular_speed_max_;
      else if(angular_z < -angular_speed_max_)
        angular_z = -angular_speed_max_;

      double vw1 = linear_x - linear_y - angular_z* (wheel_avg_distance_);
      double vw2 = linear_x + linear_y + angular_z* (wheel_avg_distance_);
      double vw3 = linear_x - linear_y + angular_z* (wheel_avg_distance_);
      double vw4 = linear_x + linear_y - angular_z* (wheel_avg_distance_);
      lock.lock();
      pack_write_.write_tmp[0] = 0x63;
      pack_write_.write_tmp[1] = 0x75;
      pack_write_.pack.ver     = 0;                 // 0x00  version
      pack_write_.pack.len     = 11; // motor + led 
      // pack_write_.pack.sn_num  = write_sn_++;
      pack_write_.pack.data.pluse_w1 = -period_/1000.0*vw1*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w2 =  period_/1000.0*vw2*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w3 = -period_/1000.0*vw4*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w4 =  period_/1000.0*vw3*(encode_resolution_/(2.0*Pi*wheel_radius_));
      //LED value
      int cur_led_mode = led_mode_type_;

      led_timer++;
      lock.unlock();
      switch (cur_led_mode)
      {
        // case LED_MODE_NORMAL:
        case ucar_interfaces::srv::SetLEDMode::Request::MODE_NORMAL: 
        {
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }
        case ucar_interfaces::srv::SetLEDMode::Request::MODE_BLINK:
        {
          double t = (double)led_timer/(double)rate_;
          lock.lock();
          // double t = (int)this->now().toSec();
          double f = led_frequency_;
          int blink = (int)(2.0*t*f)%2;
          pack_write_.pack.red_value   = led_red_value_   * blink;
          pack_write_.pack.green_value = led_green_value_ * blink;
          pack_write_.pack.blue_value  = led_blue_value_  * blink;
          lock.unlock();
          break;
        }
        case ucar_interfaces::srv::SetLEDMode::Request::MODE_BREATH:
        {
          double t = this->now().seconds();
          lock.lock();
          // double t = (double)led_timer/(double)rate_;
          double w = 2 * Pi * led_frequency_;
          pack_write_.pack.red_value   = 0.5 * (led_red_value_   + led_red_value_   * sin(w * t));
          pack_write_.pack.green_value = 0.5 * (led_green_value_ + led_green_value_ * sin(w * t));
          pack_write_.pack.blue_value  = 0.5 * (led_blue_value_  + led_blue_value_  * sin(w * t));
          lock.unlock();
          break;
        }
        default:{
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }
      }
      lock.lock();
      setWriteCS(WRITE_MSG_LONGTH);
      serial_.write(pack_write_.write_tmp,WRITE_MSG_LONGTH);
      lock.unlock();
      if(debug_log_){
        cout << "write buf:" << endl;
        for (size_t i = 0; i < WRITE_MSG_LONGTH; i++)
        {
          cout << std::hex << (int)pack_write_.write_tmp[i] << " ";
        }
        cout << std::dec << endl;
      }
      loop_rate.sleep();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "AIcarController writeLoop: %s\n", e.what());
      RCLCPP_ERROR(this->get_logger(), "AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      RCLCPP_ERROR(this->get_logger(), "AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
  }  
}

UcarController::~UcarController()
{
  if( serial_.isOpen() )   
    serial_.close();
}

bool UcarController::getBatteryStateCB(const ucar_interfaces::srv::GetBatteryInfo::Request::SharedPtr req,
                                    const ucar_interfaces::srv::GetBatteryInfo::Response::SharedPtr res)
{
  if (current_battery_percent_ == -1)  //初始值为 -1. 即没有获取到电量
  {
    res->battery_state.power_supply_status     = 0; // UNKNOWN
    res->battery_state.power_supply_health     = 0; // UNKNOWN
    res->battery_state.power_supply_technology = 0; // UNKNOWN
    std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
    res->battery_state.percentage = current_battery_percent_;
    lock.unlock();
    return true;
  }
  else
  {
    res->battery_state.power_supply_status     = 2; // 放电中 即正在运行
    res->battery_state.power_supply_health     = 1; // 良好
    res->battery_state.power_supply_technology = 0; // UNKNOWN
    res->battery_state.present    = true;           // 存在电池
    std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
    res->battery_state.percentage = current_battery_percent_; // 电池电量百分比
    lock.unlock(); 
    return true;
  }
}

bool UcarController::setLEDCallBack(const ucar_interfaces::srv::SetLEDMode::Request::SharedPtr req, 
                                 const ucar_interfaces::srv::SetLEDMode::Response::SharedPtr res)
{
  if (!read_first_)
  {
    res->success = false;
    res->message = "Can't connect base's MCU."; 
    return true;
  }
  else
  {
    std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
    led_mode_type_   = req->mode_type;
    led_frequency_   = req->frequency;
    led_red_value_   = req->red_value;
    led_green_value_ = req->green_value;
    led_blue_value_  = req->blue_value;
    led_t_0 = this->now().seconds();
    led_timer = 0;
    lock.unlock();
    res->success = true;
    res->message = "Set LED success.";
    return true;
  }
}


bool UcarController::getMaxVelCB(const ucar_interfaces::srv::GetMaxVel::Request::SharedPtr req, const ucar_interfaces::srv::GetMaxVel::Response::SharedPtr res)
{
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_);
  res->max_linear_velocity  = linear_speed_max_ ;
  res->max_angular_velocity = angular_speed_max_;
  lock.unlock();
  return true;
}
bool UcarController::setMaxVelCB(const ucar_interfaces::srv::SetMaxVel::Request::SharedPtr req, const ucar_interfaces::srv::SetMaxVel::Response::SharedPtr res)
{
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  linear_speed_max_  = req->max_linear_velocity ;
  angular_speed_max_ = req->max_angular_velocity;
  this->set_parameter(rclcpp::Parameter("linear_speed_max", linear_speed_max_));
  this->set_parameter(rclcpp::Parameter("angular_speed_max", angular_speed_max_));
  lock.unlock();
  if (linear_speed_max_ == req->max_linear_velocity && angular_speed_max_ == req->max_angular_velocity)
  {
    res->success = true;
    res->message = "Max vel set successfully.";
    return true;
  }
  res->success = false;
  res->message = "Max vel set faild.";
  return true;
}

bool UcarController::stopMoveCB(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  //判断当前控制类型
  if (controll_type_ != MOTOR_MODE_MOVE){
    res->success = false;
    res->message = "Not in MOTOR_MODE_MOVE.";
    lock.unlock();
    return true;
  }
  else{
    move_linear_x_  = 0;
    move_linear_y_  = 0;
    move_angular_z_ = 0;
    res->success = true;
    res->message = "Move stopped ";
    controll_type_ = MOTOR_MODE_CMD;
    lock.unlock();
    return true;
  }
}

void UcarController::processLoop()
{
  RCLCPP_INFO(this->get_logger(), "baseBringup::processLoop: start");
  uint8_t check_head_last[1]    = {0xFF};
  uint8_t check_head_current[1] = {0xFF};
  while(rclcpp::ok()){
    if (!serial_.isOpen())
    {
      RCLCPP_ERROR(this->get_logger(), "serial unopen");
    }
    try
    {
      int head_type = 0;
      while(rclcpp::ok()) 
      {                        
        serial_.read(check_head_current,1);
        if (check_head_last[0] == 0x63 && check_head_current[0] == 0x76) {
          std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
          pack_read_.read_msg.head[0] = check_head_last[0];
          pack_read_.read_msg.head[1] = check_head_current[0];
          check_head_last[0] = 0xFF;
          lock.unlock();
          head_type = 1; // base
          break;
        }
        else if (check_head_last[0] == 0xfc && (check_head_current[0] == 0x40 || check_head_current[0] == 0x41 || head_type == TYPE_INSGPS || 
                                                check_head_current[0] == TYPE_GROUND || check_head_current[0] == 0x50))
        {
          std::unique_lock<std::recursive_mutex> lock(Control_mutex_);
          if (check_head_current[0] == 0x40)
          {
            imu_frame_.frame.header.header_start = 0xfc;
            imu_frame_.frame.header.data_type    = 0x40;
            head_type = 0x40;
          }
          else if (check_head_current[0] == 0x41)
          {
            ahrs_frame_.frame.header.header_start = 0xfc;
            ahrs_frame_.frame.header.data_type    = 0x41;
            head_type = 0x41;
          }
          else if (check_head_current[0] == TYPE_GROUND){
            head_type = TYPE_GROUND;
          }
          else if (check_head_current[0] == 0x50)
          {
            head_type = 0x50;
          }
          check_head_last[0] = 0xFF;
          lock.unlock();
          if(debug_log_){
            cout << "head_type: " << head_type << endl;
          }
          break;
        }
        check_head_last[0] = check_head_current[0];
      }
      if (head_type == 1)
      {
        serial_.read(pack_read_.read_msg.read_msg, READ_DATA_LONGTH);
        if(debug_log_){
          cout << "serial_read: " <<endl;
          for (size_t i = 0; i < READ_MSG_LONGTH; i++)
          {
            cout << std::hex << (int)pack_read_.read_tmp[i] << " ";
          }
          cout << std::dec << endl;
        }
        if(!checkCS(READ_MSG_LONGTH))
        {
          RCLCPP_WARN(this->get_logger(), "check cs error");
        }
        else
        {
          processBattery();
          processOdometry();
        }
        if(!read_first_)
        {
          read_first_ = true;
        }
      }
      else if (head_type == 0x40 || head_type == 0x41|| head_type == TYPE_GROUND || head_type == 0x50 || head_type == TYPE_INSGPS)
      {
        processIMU(head_type);
      }
      else
      {
        if(debug_log_)
        {
          RCLCPP_DEBUG(this->get_logger(), "head_type ERROR.");
        }
      }
    }// try end
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "AIcarController readLoop: %s\n", e.what());
      RCLCPP_ERROR(this->get_logger(), "AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      RCLCPP_ERROR(this->get_logger(), "AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
  }
}

void UcarController::processIMU(uint8_t head_type)
{
  uint8_t check_len[1] = {0xff};
  serial_.read(check_len, 1);
  if (debug_log_){
    std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
  }
  if (head_type == TYPE_IMU && check_len[0] != IMU_LEN)
  {
    RCLCPP_WARN(this->get_logger(), "head_len error (imu)");
    return;
  }else if (head_type == TYPE_AHRS && check_len[0] != AHRS_LEN)
  {
    RCLCPP_WARN(this->get_logger(), "head_len error (ahrs)");
    return;
  }else if (head_type == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
  {
    RCLCPP_WARN(this->get_logger(), "head_len error (insgps)");
    return;
  }
  else if (head_type == TYPE_GROUND || head_type == 0x50) // 无效数据，防止记录失败
  {
    uint8_t ground_sn[1];
    serial_.read(ground_sn, 1);
    if (++read_sn_ != ground_sn[0])
    {
      if ( ground_sn[0] < read_sn_)
      {
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_1.");
        }
        sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
        read_sn_ = ground_sn[0];
      }
      else
      {
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_2.");
        }
        sn_lost_ += (int)(ground_sn[0] - read_sn_);
        read_sn_ = ground_sn[0];
      }
    }
    uint8_t ground_ignore[500];
    serial_.read(ground_ignore, (check_len[0]+4));
    return;
  }
  //read head sn 
  uint8_t check_sn[1] = {0xff};
  serial_.read(check_sn, 1);
  uint8_t head_crc8[1] = {0xff};
  serial_.read(head_crc8, 1);
  uint8_t head_crc16_H[1] = {0xff};
  uint8_t head_crc16_L[1] = {0xff};
  serial_.read(head_crc16_H, 1);
  serial_.read(head_crc16_L, 1);
  if (debug_log_){
    std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
    std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
    std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
    std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
  }
  // put header & check crc8 & count sn lost
  if (head_type == TYPE_IMU)
  {
    imu_frame_.frame.header.data_size      = check_len[0];
    imu_frame_.frame.header.serial_num     = check_sn[0];
    imu_frame_.frame.header.header_crc8    = head_crc8[0];
    imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
    if (CRC8 != imu_frame_.frame.header.header_crc8)
    {
      RCLCPP_WARN(this->get_logger(), "header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = imu_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    UcarController::checkSN(TYPE_IMU);
  }
  else if (head_type == TYPE_AHRS)
  {
    ahrs_frame_.frame.header.data_size      = check_len[0];
    ahrs_frame_.frame.header.serial_num     = check_sn[0];
    ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
    ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
    if (CRC8 != ahrs_frame_.frame.header.header_crc8)
    {
      RCLCPP_WARN(this->get_logger(), "header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    UcarController::checkSN(TYPE_AHRS);
  }
  else if (head_type == TYPE_INSGPS)
  {
    insgps_frame_.frame.header.header_start   = 0xfc;
    insgps_frame_.frame.header.data_type      = TYPE_INSGPS;
    insgps_frame_.frame.header.data_size      = check_len[0];
    insgps_frame_.frame.header.serial_num     = check_sn[0];
    insgps_frame_.frame.header.header_crc8    = head_crc8[0];
    insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
    if (CRC8 != insgps_frame_.frame.header.header_crc8)
    {
      RCLCPP_WARN(this->get_logger(), "header_crc8 error");
      return;
    }
    else if(debug_log_)
    {
      std::cout << "header_crc8 matched." << std::endl;
    }
    
    UcarController::checkSN(TYPE_INSGPS);
  }
  if (head_type == TYPE_IMU)
  {
    uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      RCLCPP_WARN(this->get_logger(), "check crc16 faild(imu).");
      return;
    }
    else if(imu_frame_.frame.frame_end != FRAME_END)
    {
      RCLCPP_WARN(this->get_logger(), "check frame end.");
      return;
    }
    
  }
  else if (head_type == TYPE_AHRS)
  {
    uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      RCLCPP_WARN(this->get_logger(), "check crc16 faild(ahrs).");
      return;
    }
    else if(ahrs_frame_.frame.frame_end != FRAME_END)
    {
      RCLCPP_WARN(this->get_logger(), "check frame end.");
      return;
    }
  }
  else if (head_type == TYPE_INSGPS)
  {
    uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
    serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
    if (head_crc16 != CRC16)
    {
      RCLCPP_WARN(this->get_logger(), "check crc16 faild(insgps).");
      return;
    }
    else if(insgps_frame_.frame.frame_end != FRAME_END)
    {
      RCLCPP_WARN(this->get_logger(), "check frame end.");
      return;
    }
    
  }

  // publish magyaw topic
  if (head_type == TYPE_AHRS)
  {
    // publish imu topic
    sensor_msgs::msg::Imu imu_data;
    imu_data.header.stamp = this->now();
    imu_data.header.frame_id = imu_frame_id_;
    Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                              ahrs_frame_.frame.data.data_pack.Qx,
                              ahrs_frame_.frame.data.data_pack.Qy,
                              ahrs_frame_.frame.data.data_pack.Qz);
    Eigen::Quaterniond q_r =                          
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr =                          
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.orientation_covariance = {0.0017, 0, 0, 0, 0.0017, 0, 0, 0, 0.0017};

    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
    imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
    imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
    imu_data.angular_velocity_covariance = {4.887e-4, 0, 0, 0, 4.887e-4, 0, 0, 0, 4.887e-4};

    imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
    imu_data.linear_acceleration_covariance = {6.37e-3, 0, 0, 0, 6.37e-3, 0, 0, 0, 6.86e-3};

    imu_pub_->publish(imu_data);

    Eigen::Quaterniond rpy_q(imu_data.orientation.w,
                              imu_data.orientation.x,
                              imu_data.orientation.y,
                              imu_data.orientation.z);
    geometry_msgs::msg::Pose2D pose_2d;
    double magx, magy, magz, roll, pitch;
    magx  = -imu_frame_.frame.data.data_pack.magnetometer_x;
    magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
    magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
    Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
    roll  = EulerAngle[2];
    pitch = EulerAngle[1];

    double magyaw;
    magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
    pose_2d.theta = magyaw;
    mag_pose_pub_->publish(pose_2d);
  }
}

void UcarController::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * Pi;
  }
  // return magyaw;
}

void UcarController::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_3.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_4.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_5.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_6.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_7.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          RCLCPP_WARN(this->get_logger(), "detected sn lost_8.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

void UcarController::setWriteCS(int len)
{
	uint8_t ck = 0x00;
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_write_.write_tmp[i];
	}
	pack_write_.write_tmp[len - 1] = ck;
  lock.unlock();
}

void UcarController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
  if(debug_log_){
    std::cout << "joyCallback" << std::endl;
  }
  //mode_switch 
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  if (msg->buttons[0]==1)//turn off joy mode
  {
    controll_type_ = MOTOR_MODE_CMD;
    std::cout << "controll_type_ turn to MOTOR_MODE_CMD:" << controll_type_ << std::endl;

  }
  else if (msg->buttons[1] == 1)//turn on joy mode
  {
    controll_type_ = MOTOR_MODE_JOY;
    std::cout << "controll_type_ turn to MOTOR_MODE_JOY:" << controll_type_ << std::endl;
  }
  
  //set speed 
  if (msg->axes[6] == 1)        //twist_speed down
  {
    if(debug_log_){
      std::cout << "twist_speed down" << std::endl;
    }
    twist_gain_ -= 0.1;

  }else if(msg->axes[6] == -1)  //twist_speed up
  {
    if(debug_log_){
      std::cout << "twist_speed up" << std::endl;
    }
    twist_gain_ += 0.1;
  }
  if (msg->axes[7] == -1)      //linear_speed down
  {
    if(debug_log_){
      std::cout << "linear_speed down" << std::endl;
    }
    linear_gain_ -= 0.1;
  }else if(msg->axes[7] == 1)  //linear_speed up
  {
    if(debug_log_){
      std::cout << "linear_speed up" << std::endl;
    }
    linear_gain_ += 0.1;
  }
  //write speed
  double linear_x  = linear_gain_ * msg->axes[1];  // linear_gain_ = 0.3
  double linear_y  = linear_gain_ * msg->axes[0];  // twist_gain_  = 0.2  
  double angular_z = twist_gain_  * msg->axes[3];  // msg->axes[]  = [-1,1]

  if (debug_log_){
    cout << "linear_x=" << linear_x << "linear_y=" << linear_y << "angular_z=" << angular_z << endl;
  }  
  joy_linear_x_  =  linear_x;
  joy_linear_y_  =  linear_y;
  joy_angular_z_ =  angular_z;
  lock.unlock();
  return;
}

void UcarController::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_);
  if (controll_type_ != MOTOR_MODE_CMD)
  {
    lock.unlock();
    return;
  }
  if(debug_log_){
    std::cout << "vel mode" << std::endl;
  }
  cmd_linear_x_  =  msg -> linear.x;
  cmd_linear_y_  =  msg -> linear.y;
  cmd_angular_z_ =  msg -> angular.z;
  last_cmd_time_ = this->now();
  lock.unlock();
  return;
}

bool UcarController::checkCS(int len){
  uint8_t ck = 0;
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_read_.read_tmp[i];
	}
  return pack_read_.read_tmp[len - 1] == ck;
  // return true;
}

void UcarController::processBattery()
{
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  current_battery_percent_ = (int)pack_read_.pack.battery_percent;
  lock.unlock();
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.power_supply_status     = 2; // 放电中 即正在运行
  battery_msg.power_supply_health     = 1; // 良好
  battery_msg.power_supply_technology = 0; // UNKNOWN
  battery_msg.present    = true;           // 存在电池
  battery_msg.percentage = current_battery_percent_; // 电池电量百分比
  battery_pub_->publish(battery_msg);
}

void UcarController::processOdometry(){
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  current_time_ = this->now();
  double dt = (current_time_ - last_time_).seconds();
  last_time_ = current_time_;
  lock.unlock();
  double vw1,vw2,vw3,vw4;
  vw1 = -pack_read_.pack.data.pluse_w1 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw2 =  pack_read_.pack.data.pluse_w2 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw4 = -pack_read_.pack.data.pluse_w3 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw3 =  pack_read_.pack.data.pluse_w4 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);

  double Vx,Vy,Vth;
  Vx  = ( vw1+vw2+vw3+vw4)/4;
  Vy  = vy_coefficient*(-vw1+vw2-vw3+vw4)/4;
  Vth = (-vw1+vw2+vw3-vw4)/(4*(wheel_avg_distance_));

  double delta_x = (Vx * cos(th_) - Vy * sin(th_)) * dt;
  double delta_y = (Vx * sin(th_) + Vy * cos(th_)) * dt;
  double delta_th = Vth * dt;
  lock.lock();
  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;
  lock.unlock();
  nav_msgs::msg::Odometry odom_tmp;
  odom_tmp.header.stamp = this->now();
  odom_tmp.header.frame_id = odom_frame_;
  odom_tmp.child_frame_id  = base_frame_;
  odom_tmp.pose.pose.position.x = x_;
  odom_tmp.pose.pose.position.y = y_;
  odom_tmp.pose.pose.position.z = 0.0;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, th_);
  auto odom_quat = tf2::toMsg(tf2_quat);
  odom_tmp.pose.pose.orientation = odom_quat;
  if (!Vx||!Vy||!Vth){
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE;
  }else{
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE2;
  }
  odom_tmp.twist.twist.linear.x  = Vx;
  odom_tmp.twist.twist.linear.y  = Vy;
  odom_tmp.twist.twist.linear.z  = 0.0;
  odom_tmp.twist.twist.angular.x = 0.0;
  odom_tmp.twist.twist.angular.y = 0.0;
  odom_tmp.twist.twist.angular.z = Vth;
  if (!Vx||!Vy||!Vth){
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE;
  }else{
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE2;
  }
  odom_pub_->publish(odom_tmp);
  
  lock.lock();
  current_odom_ = odom_tmp;
  lock.unlock();

  provide_odom_tf_ = this->get_parameter("provide_odom_tf").as_bool();
  
  if(provide_odom_tf_)
  {
    geometry_msgs::msg::TransformStamped odom_trans;     /* first, we'll publish the transform over tf */
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id  = base_frame_;
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_->sendTransform(odom_trans);    /* send the transform */
  }
  updateMileage(odom_tmp.twist.twist.linear.x,odom_tmp.twist.twist.linear.y,dt);
}

bool UcarController::getMileage(){
  std::fstream fin;
  std::fstream fin_b;
  std::string str_in;
  std::string str_in_b;
  std::stringstream ss;
  
	fin.open(Mileage_file_name_); //Mileage_backup_file_name_
  fin_b.open(Mileage_backup_file_name_);
  if (fin.fail() && fin_b.fail())
  {
    RCLCPP_ERROR(this->get_logger(), "open Mileage files error, will creat a new file! \n");
    Mileage_sum_ = 0.0;
    return false;
  }
  if (!fin.fail()){
    fin >> str_in;
    fin.close();
  }
  if (!fin_b.fail()){
      fin_b >> str_in_b;
      fin_b.close();
  }
  if (!str_in.empty())
  {
    ss << str_in;
    ss >> Mileage_sum_;
    ss.clear();
  }
  else if (!str_in_b.empty())
  {
    ss << str_in_b;
    ss >> Mileage_sum_;
    ss.clear();
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Mileage_files empty. \n");
    Mileage_sum_ = 0.0;
  }
  return true;
}

bool UcarController::updateMileage(double vx, double vy, double dt){
  double speed  = sqrt(vx*vx + vy*vy);
  std::unique_lock<std::recursive_mutex> lock(Control_mutex_); 
  Mileage_sum_ += speed * dt;
  lock.unlock();
  double d_mileage = abs(Mileage_sum_ - Mileage_last_);
  if (d_mileage < 0.1)
  {
    return true;
  }
  FILE* fout = std::fopen(Mileage_file_name_.c_str(), "w");
  if (fout)
	{
		std::fprintf(fout,"%lf\n",Mileage_sum_);
		std::fclose(fout);
	}
  FILE* fout_b = std::fopen(Mileage_backup_file_name_.c_str(), "w");
  if (fout_b)
	{
		std::fprintf(fout_b,"%lf\n",Mileage_sum_);
		std::fclose(fout_b);
	}
  Mileage_last_ = Mileage_sum_;
  return true;
}

void UcarController::callHandle()
{
  serial_.open();
}

} // namespace ucar_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(ucar_controller::UcarController)
