# MAVROS

## 消息结构

PositionTarget message

官方文档里面提供了原始数据定义

```cpp
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1 # Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8 # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
```

压缩版本

```cpp
uint8 FRAME_LOCAL_NED=1
uint8 FRAME_LOCAL_OFFSET_NED=7
uint8 FRAME_BODY_NED=8
uint8 FRAME_BODY_OFFSET_NED=9
uint16 IGNORE_PX=1
uint16 IGNORE_PY=2
uint16 IGNORE_PZ=4
uint16 IGNORE_VX=8
uint16 IGNORE_VY=16
uint16 IGNORE_VZ=32
uint16 IGNORE_AFX=64
uint16 IGNORE_AFY=128
uint16 IGNORE_AFZ=256
uint16 FORCE=512
uint16 IGNORE_YAW=1024
uint16 IGNORE_YAW_RATE=2048
std_msgs/Header header
uint8 coordinate_frame
uint16 type_mask
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
```

其中 

1. 在coordinate frame选项里面，定义了不同的坐标系选用方式

| Frame 名称                 | 位置参考点 | 位置参考方位 | 速度参考方位 |
| -------------------------- | ---------- | ------------ | ------------ |
| MAV_FRAME_LOCAL_NED        | home       | NED          | NED          |
| MAV_FRAME_LOCAL_OFFSET_NED | vehicle    | NED          | NED          |
| MAV_FRAME_BODY_OFFSET_NED  | vehicle    | FRD          | FRD          |
| MAV_FRAME_BODY_NED         | home       | NED          | FRD          |

### 名词解释

+ home 以home点的位置（通常是无人机初始化的点）作为参考位置
+ vehicle 以无人机的当前位置作为参考点
+ NED NORTH-EAST-DOWN 分别与x，y，z轴对应
+ FRD FOWAED-RIGHT-DOWN 分别与x,y,z轴对应

2. type_mask用于规定输入量是什么，采用了uint16为数据类型

| type_mask | yaw_rate | yaw  | force | AZ   | AY   | AX   | VZ   | VY   | VX   | PZ   | PY   | PX   |
| --------- | -------- | ---- | ----- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| bool      |          |      |       |      |      |      |      |      |      |      |      |      |

在决定输入的方式的时候，我们可以定义各个变量是否被采用（相应的位下面数字是1，表示位被ignore；换言之如果相应的位下面数字是0，表明位没有被屏蔽）在后续的输入的时候，无人机不会因为被屏蔽位上的量干扰。

比如，我们在采用xyz+yaw期望位置的控制方法的时候，我们的掩码格式为：

```cpp
someobject.type_mask = 0b100111111000 //0b表示的是二进制
```

或者写成更容易懂的结构：

```cpp
someobject.type_mask = mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|mavros_msgs::PositionTarget::IGNORE_AX|mavros_msgs::PositionTarget::IGNORE_AY|mavros_msgs::PositionTarget::IGNORE_AZ|IGNORE_YAW_RATE
```

当然，也可以采用十进制、十六进制的方式来设置掩码。

## 订阅话题

```cpp
some_pub = nodehandle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
```

消息类型是mavros_msgs::PositionTarget，订阅的话题名为“/mavros/setpoint_raw/local”

对应的，我们需要创建一个mavros_msgs::PositionTarget的实例，分配相应的数据（x,y,z,vx,vy,vz,ax,ay,az,type_mask,corrdinate_frame,force等等），然后调用发布者将这个示例发布出去就可以达到对飞机的控制。

这个消息类型比较灵活，推荐使用这个，相比较于单纯发布一个期望位置以及速度这种固定死了的发布，这个话题发布的命令可以通过设置掩码由自己决定。