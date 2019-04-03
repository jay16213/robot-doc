# Development
## Source code in Udoo
```
~/catkin_ws/
  build/                                <- the folder generated after compile
  devel/                                <- the folder generated after compile
  src/
    api_test/                           <- robot control api example(https://github.com/jay16213/Robot-Control-api)
    tracked_robot/
      tracked_robot/
        src/                            <- all source code for robot controlling
        all_in_one.launch
        CMakeLists.txt
      dynamixel-workbench/              <- dynamixel ros API (if use apt to install this api, you will not have this folder)
      dynamixel-workbench-msgs/         <- dynamixel ros API (if use apt to install this api, you will not have this folder)
      my_dynamixel_workbench_tutorial/  <- dynamixel ros API
        launch/
          position_control.launch       <- dynamixel launch file(all_in_one.launch would include this launch file)
      README.md
    CMakeLists.txt
```

## Robot Control
There 10 topics and 1 service client for this robot. We can publish our command to these topics to control the motion, speed, or arm angle of the robot.

For more detail code example, read `~/catkin_ws/src/tracked_robot/tracked_robot/src/Manual_mode.cpp`

### Topics
> **NOTE** 原則上, 在 source code 中看到 robot_ 都是跟行走馬達有關, leg_ 都是跟手臂馬達有關

#### Overview
| Topic name   | Function                        | Type                      |
| ------------ | ------------------------------- | ------------------------- |
| robot_motion | control the motion of the robot | std_msgs::Int32           |
| robot_speed  | set speed of the robot          | std_msgs::Int32MultiArray |
| robot_VA     | set acceleration of the robot   | std_msgs::Int32MultiArray |
| robot_MA     | 控制行走馬達角度                | std_msgs::Int32MultiArray |
| robot_HO     | 設定機器人基準點                | std_msgs::Int32MultiArray |
| leg_motion   | control the motion of arms      | std_msgs::Int32           |
| leg_speed    | set speed of arm                | std_msgs::Int32MultiArray |
| leg_VA       | set acceleration of arm         | std_msgs::Int32MultiArray |
| leg_MA       | set the angle of arm            | std_msgs::Int32MultiArray |
| leg_HO       | 設定手臂馬達目前所在角度之座標  | std_msgs::Int32MultiArray |

#### robot_motion
Publish an integer in [0, 4] to control the motion.

| Value | Command    |
| ----- | ---------- |
| 0     | stop       |
| 1     | forward    |
| 2     | back       |
| 3     | spin left  |
| 4     | spin right |

```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32>("robot_motion", 100);
std_msgs::Int32 robot_motion;

robot_motion.data = 1;      // 1: go forward
pub.publish(robot_motion);  // send the command
sleep(10);                  // the robot will go forward for 10 sec
robot_motion.data = 0;      // 0: stop the robot
pub.publish(robot_motion);  // send the command
```

#### robot_speed
- Control the speed of moving
- Default: 350
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
std_msgs::Int32MultiArray robot_speed;
robot_speed.data.clear();

// robot_speed.data[0] is left
// robot_speed.data[1] is right
robot_speed.data.push_back(350); // set left speed to 350
robot_speed.data.push_back(350); // set right speed to 350
pub.publish(robot_speed);
```

#### robot_VA
- 左右行走馬達加速度控制, 可改變起步加速度/煞車減速度
- default: 200
- range from 200 to 1000
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("robot_VA", 100);
std_msgs::Int32MultiArray robot_VA;
robot_VA.data.clear();

robot_VA.data.push_back(200); // set left acceleration to 200
robot_VA.data.push_back(200); // set right acceleration to 200
pub.publish(robot_VA);
```

#### leg_motion
| Value | Command                     |
| ----- | --------------------------- |
| 0     | Stop all the motion of arms |
| 1     | Front arm up                |
| 2     | Front arm down              |
| 3     | Back arm up                 |
| 4     | Back arm down               |

```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32>("leg_motion", 100);
std_msgs::Int32 leg_motion;

leg_motion.data = 1;      // 1: front arm up
pub.publish(leg_motion);  // send the command
sleep(10);                // the front arm will go up for 10 sec
leg_motion.data = 0;      // 0: stop all the arm
pub.publish(leg_motion);  // send the command
```

#### leg_speed
- Control the speed of arm up/down
- 設定 1000 即可, 不要調太快
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_speed", 100);

std_msgs::Int32MultiArray leg_speed;
leg_speed.data.clear();

leg_speed.data.push_back(1000); // set front arm speed to 1000
leg_speed.data.push_back(1000); // set back arm speed to 1000
pub.publish(leg_speed);
```

#### leg_MA
- 根據手臂馬達的基準點設定手臂角度位置
- 單位不是度
    - n = (欲轉到角度) * 4550, n 即為需要 publish 的數值
    - 此公式為實驗量出之約略值, 可依需求自由調整
    - 如有使用到 leg_HO 來重置基準點, 請注意正負號

```c++
//example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 100);
std_msgs::Int32MultiArray leg_MA;

leg_MA.data.clear();
leg_MA.data.push_back(-26000); // set front arm to -26000
leg_MA.data.push_back(-27000); // set back arm to -27000
pub.publish(leg_MA);
```

#### leg_HO
- 設定手臂馬達目前所在角度之座標
- 通常用於設定基準點 (e.g 將目前角度設為 0 以做為基準點)
```c++
pub = n.advertise<std_msgs::Int32MultiArray>("leg_HO", 1);
std_msgs::Int32MultiArray leg_HO;
leh_HO.data.clear();

leg_HO.data.push_back(0); // set current position of the front arm as 0
leg_HO.data.push_back(0); // set current position of the back arm as 0
pub.publish(leg_HO);
```

#### leg_VA
- 控制手臂馬達加速度, 可控制手臂抬起/放下之加速度
```c++
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_VA", 100);
std_msgs::Int32MultiArray leg_VA;
leg_VA.data.clear();

leg_VA.data.push_back(200); // set front arm acceleration to 200
leg_VA.data.push_back(200); // set back arm acceleration to 200
pub.publish(leg_VA);
```

### Service Client
#### Overview
| Service       | Function                   | Type                                   |
| ------------- | -------------------------- | -------------------------------------- |
| joint_command | 控制雲台馬達上下左右之角度 | dynamixel_workbench_msgs::JointCommand |

#### joint_command
- control the angle of dynamixel
- `top` controls up/down; `bottom` controls left/right
- Default:
    - top: 134
    - bottom: 46

```c++
//example
#define PI 3.14

ros::ServiceClient joint_command_client =
                n.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");

dynamixel_workbench_msgs::JointCommand joint_command;
int bottom = 46, top = 134;

joint_command.request.unit = "rad";
joint_command.request.id = 1;                          // id 1 control the bottom motor
joint_command.request.goal_position = bottom * PI/180; // the bottom control left-right direction of the motor
joint_command_client.call(joint_command);
joint_command.request.unit = "rad";
joint_command.request.id = 2;                          // id 2 control the top motor
joint_command.request.goal_position = top * PI/180;    // the top control up-down direction of the motor
joint_command_client.call(joint_command);
```

## Compile
### Command
```bash
cd ~/catkin_ws
catkin_make
```

#### Add Executable
```cmake
# in CMakeLists.txt, add this two lines and any other you need
add_executable(<node_name>, <filename 1> <filename 2> ...)
target_link_libraries(<node_name>, ${catkin_LIBRARIES} <other libraries you need>)
...
```
- after a new executable be compiled, you need to use `source` command so that you can use tab to complete the command automatically.
```bash
source ~/catkin_ws/devel/setup.bash
```
- Notice that in ROS, each package has its own CMakeLists.txt, modify the correct CMakeLists.txt (e.g in package `tracked_robot`, modify `~/catkin_ws/tracked_robot/tracked_robot/CMakeLists.txt`

> **WARN** Because Udoo does not connect to the internet, it does not update the system time to the correct time every time you boot it. So before modifing/adding new files on Udoo, you need to update system time manually, otherwise the compilation would not succeed. (modification in the future)
>    - Check system time
>    ```bash
>   date
>    ```
>    - Change system time
>    ```bash
>    sudo date MMDDhhmmYYYY
>    # MM: month
>    # DD: day
>    # hh: hour (24-hour)
>    # mm: minute
>    # YYYY: year
>    ```
>    - Change the last modification time of the file to the current system time
>    ```bash
>    touch <filename>
>    ```
