# ucar_controller
[TOC]
## 简介
`ucar_controller`作为与下位机mcu通讯的ROS包实现了电机控制、灯光控制、mcu板载imu数据读取等功能。我们可以通过发布`/cmd_vel(geometry_msgs/Twist)`速度控制话题，指定xy方向的平移速度以及yaw角的旋转速度来控制小车运动。同时发布里程计话题`/odom(nav_msgs/Odometry)`实时发布小车里程计记录的速度、位置。如果是晓mini版本的智能驾驶小车，本包还会发布`/imu(sensor_msgs/Imu)`。

## 安装所需依赖
```bash
sudo apt update
sudo apt install ros-melodic-serial libeigen3-dev

sudo pip3 uninstall em # 卸载em，em可能未安装。忽视未安装无法卸载的报错即可
sudo pip3 install empy -i https://pypi.tuna.tsinghua.edu.cn/simple
```
编译：
```bash
cp -r ucar_controller ~/ucar_ws/src/
cd ~/ucar_ws
catkin_make
```
##  简单验证
1、启动节点：

注：晓版本的小车需要把`base_driver.launch`中导入的`driver_params_mini.yaml`注释，并换为：`driver_params_xiao.yaml`
即已经注释的那行。
```bash
roslaunch ucar_controller base_driver.launch
```

2、发布话题控制：
base_driver节点订阅/cmd_vel话题，msg类型为geometry_msgs/Twist。
我们可以通过发布/cmd_vel话题来控制小车移动，比如：
Ctrl + Alt + T重新开启一个终端。
用rostopic工具发布话题。（注意用Tab键自动补全来填写geometry_msgs/Twist数据，不然手动输入格式容易出错）
```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 1.0"
```
输入指令后小车会持续逆时针自转。

## 全部功能介绍

### 1. 实时控制
方式：发布话题。
话题名：`/cmd_vel`
msg类型：geometry_msgs/Twist
```
geometry_msgs/Vector3 linear
  float64 x                   # x方向线速度
  float64 y                   # y方向线速度
  float64 z                   # 无效
geometry_msgs/Vector3 angular
  float64 x                   # 无效
  float64 y                   # 无效
  float64 z                   # z轴角速度
```


### 2. 获取里程计数据
方式：订阅话题。
话题名：`/odom`
msg类型：nav_msgs/Odometry
```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position # 里程计坐标下的位置，z值无效
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation # 里程计坐标下的朝向
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear  # 小车实时线速度，z值无效
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular # 小车实时角速度，x,y值无效
      float64 x
      float64 y
      float64 z
  float64[36] covariance

```

可以通过执行`rostopic echo /odom `指令，在终端实时打印小车里程计数据。

### 3. 获取imu数据
>晓版本小车没用该功能，晓版本小车需要使用fdilink_ahrs包来获取imu数据

1. imu数据
方式：订阅话题。
话题名：`/imu`
msg类型：sensor_msgs/Imu
```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation # 小车姿态
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity # 小车实时角速度
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration # 小车实时加速度
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance
```
可以通过执行`rostopic echo /imu `指令，在终端实时打印小车imu数据。

2. 地磁指北数据
方式：订阅话题。
话题名：`/mag_pose_2d`
msg类型：geometry_msgs/Pose2D
```cpp
float64 x # 无效
float64 y # 无效 
float64 theta # 指北的yaw角值（单位：rad）
```
可以通过执行`rostopic echo /mag_pose_2d `指令，在终端实时打印。

### 4. 设置灯光效果
服务名：`get_battery_state`
服务数据类型：aicar_controller/GetBatteryInfo.srv
```
uint8 MODE_NORMAL = 0 # 常亮
uint8 MODE_BLINK  = 1 # 闪烁
uint8 MODE_BREATH = 2 # 呼吸

uint8   mode_type   # 灯光模式
float64 frequency   # 模式频率（常亮模式自动忽略）
uint8 red_value   # 红色光亮度范围（0-255）
uint8 green_value # 绿色光亮度范围（0-255）
uint8 blue_value  # 蓝色光亮度范围（0-255）
---
bool success        # 设置是否成功
string message      # 异常log信息
```
response枚举：
1、设置失败（可能与MUC通信异常）
  res.success = false;
  res.message = "Can't connect base's MCU.";
2、设置成功
  res.success = true;
  res.message = "Set LED success.";

### 5. 小车电池信息发布（仅电池剩余电量百分比有效，即：percentage）
话题名：`battery_state`
`sensor_msgs/BatteryState.msg`
```
POWER_SUPPLY_STATUS_UNKNOWN      = 0            # 未知
POWER_SUPPLY_STATUS_CHARGING     = 1            # 充电
POWER_SUPPLY_STATUS_DISCHARGING  = 2            # 放电中
POWER_SUPPLY_STATUS_NOT_CHARGING = 3            # 未充电
POWER_SUPPLY_STATUS_FULL         = 4            # 满电
POWER_SUPPLY_HEALTH_UNKNOWN               = 0   # 未知
POWER_SUPPLY_HEALTH_GOOD                  = 1   # 良好
POWER_SUPPLY_HEALTH_OVERHEAT              = 2   # 过热
POWER_SUPPLY_HEALTH_DEAD                  = 3   # 没电
POWER_SUPPLY_HEALTH_OVERVOLTAGE           = 4   # 过电压
POWER_SUPPLY_HEALTH_UNSPEC_FAILURE        = 5   # 未知错误
POWER_SUPPLY_HEALTH_COLD                  = 6   # 过冷
POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7   # 
POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE   = 8   # 
POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0   # 未知
POWER_SUPPLY_TECHNOLOGY_NIMH    = 1   # Ni-MH  battery 镍氢电池
POWER_SUPPLY_TECHNOLOGY_LION    = 2   # Li-ion battery 锂离子电池
POWER_SUPPLY_TECHNOLOGY_LIPO    = 3   # Li-Po  battery 锂聚合物电池
POWER_SUPPLY_TECHNOLOGY_LIFE    = 4   # li-Fe  battery 锂铁电池
POWER_SUPPLY_TECHNOLOGY_NICD    = 5   # NiCd   battery 镍镉电池
POWER_SUPPLY_TECHNOLOGY_LIMN    = 6   # LiMn   battery 锂锰电池

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 voltage                      # Voltage in Volts (Mandatory)
float32 current                      # Negative when discharging (A)  (If unmeasured NaN)
float32 charge                       # Current charge in Ah  (If unmeasured NaN)
float32 capacity                     # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity              # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage                   # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
```
当前版本实际使用到的status数据：
```cpp
  battery_msg.power_supply_status     = 2;           // 放电中 即正在运行
  battery_msg.power_supply_health     = 1;           // 良好
  battery_msg.power_supply_technology = 0;           // UNKNOWN
  battery_msg.present    = true;                     // 存在电池
  battery_msg.percentage = current_battery_percent_; // 电池电量百分比
```

### 6. 获取小车电量（仅电池剩余电量百分比有效，即：percentage）
服务名：`get_battery_state`
用途： 调用主动获取电池状态。
服务数据类型：aicar_controller/GetBatteryInfo.srv
```
---
sensor_msgs/BatteryState battery_state
```
即，服务的request为空，response为上面话题发布的电池状态。

response枚举：
  1、获取异常(可能是串口未与MCU通讯)：
  ```
  response.battery_state.percentage              = -1 # 表示异常
  response.battery_state.power_supply_status     =  0 # 未知
  response.battery_state.power_supply_health     =  0 # 未知
  response.battery_state.power_supply_technology =  0 # 未知
  ```

  2、正常获取：
  ```
  response.battery_state.percentage              = <current_percent> # 电池剩余电量百分比(0-100)。
  response.battery_state.power_supply_status     =  2 # 放电中
  response.battery_state.power_supply_health     =  1 # 良好
  response.battery_state.power_supply_technology =  0 # 未知
  ```

### 7. 获取当前最大速度
服务名：`get_max_vel`
服务数据类型：GetMaxVel.srv
```
---
float64 max_linear_velocity   # 小车最大线速度 (m)
float64 max_angular_velocity  # 小车最大角速度 (rad/s)
```

### 8. 设置小车最大速度
服务名：`set_max_vel`
服务数据类型：SetMaxVel.srv
```
float64 max_linear_velocity   # 小车最大线速度 (m)
float64 max_angular_velocity  # 小车最大角速度 (rad/s)
---
bool success
string message
```
注：调用后同时会更新参数服务器中参数`/base_driver/linear_speed_max`、`/base_driver/angular_speed_max`.
也可以通过这两个参数获取当前最大速度。
> 最大线速度受电机实际最大转速限制可达到的最大速度约为1m/s

> 以下两个是 tf_server提供的

### 9. 获取小车运行总里程
方式：获取参数（get_param）
参数名：`Mileage_sum`
单位：米。（即记录了该小车自出厂的运行里程）

### 10. 里程计与小车的坐标变换是否发布由参数实时配置
>主要用于cartographer算法建图时关闭`odom -> base_link`的TF，避免冲突。

参数名称：`publish_odom_tf`
1. 设置为true:   发布 odom -> base_link的坐标变换
2. 设置为false: 不发布 odom -> base_link的坐标变换



> 注，以下运动相关服务优先度大于实时运动控制（cmd_vel）当服务完成后才会继续执行实时控制。（手柄的优先级更高，不过不涉及web接口，此处不详细描述。手柄 > 运动服务 > cmd_vel）


## 里程计校准
使用`scripts`文件夹中的`linear_check.py`和`angular_check.py`脚本来校准里程计。
### 1. 平移校准
使用`linear_check.py`，脚本控制小车运动一定距离然后量取小车实际运动的距离。
如果有偏差，通过修改launch文件中加载的`driver_params_mini.yaml`或`driver_params_xiao.yaml`文件（该哪个看实际车型以及launch加载的哪个文件）中的`wheel_radius`参数。
这个参数设定了小车的轮径。里程计输出的速度与距离与这个参数成正比。
即如果实际运动的距离未达到设定距离则为里程计输出偏高。根据超出的比例减小`wheel_radius`的数值进行校准。
```bash
roscd ucar_controller/scripts
sudo chmod 777 ./* # 给可执行权限
```
使用`linear_check.py`:
```bash
# 不传参数默认小车前进一米
./linear_check.py 

# 一个参数，则小车向x方向运动（即前进）传入的距离（单位：m）
./linear_check.py 1.2 

# 两个参数，小车向x y 方向运动传入的距离（单位：m）,如下为x：1.2；y：1.0
./linear_check.py 1.2 1 
```

### 2. 旋转校准
注：旋转校准需要在平移校准之后进行，因为里程计给出的旋转角度同时与`wheel_avg_distance`、`wheel_radius`两个参数相关
即与`wheel_radius`（轮径）成正比，与`wheel_avg_distance`成反比。
我们需要确保平移校准后`wheel_radius`（轮径）准确在进行这一步。

使用`angular_check.py`，脚本控制小车旋转一定角度然后量取小车实际旋转的角度。
如果有偏差，通过修改launch文件中加载的`driver_params_mini.yaml`或`driver_params_xiao.yaml`文件（该哪个看实际车型以及launch加载的哪个文件）中的`wheel_avg_distance`参数进行教专。
这个参数设定了小车的平均轮距（即，前后轮距与左右轮距的均值）。里程计输出的角速度与角度与这个参数成反比。
即如果实际旋转角度未达到设定距离，则为里程计输出偏高。根据超出的比例增加`wheel_avg_distance`的数值进行校准。

这步在平移校准时执行过，可以省略。此处避免忘记给.py文件赋予可执行权限：
```bash
roscd ucar_controller/scripts
sudo chmod 777 ./* # 给脚本可执行权限
```
使用`angular_check.py`:
```bash
# 不传参数默认小车旋转360度
./linear_check.py 

# 整数参数，为小车旋转固定角度（单位：°）
./linear_check.py 180     # 旋转180度

# 小数参数，为小车旋转固定角度（单位：°）
./linear_check.py 3.14159 # 旋转180度
```