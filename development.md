# Development
> 機器人專案架構為 ROS 的架構, 需先學習 ROS

## Source code in Udoo
```
~/catkin_ws/
  build/                                <- ros 編譯資料夾
  devel/                                <- ros 編譯資料夾
  src/
    tracked_robot/
      tracked_robot/                    <- robot 的主程式
      dynamixel-workbench/              <- 雲台馬達 ros API
      dynamixel-workbench-msgs/         <- 雲台馬達 ros API
      my_dynamixel_workbench_tutorial/  <- 雲台馬達 ros API
      README.md
    CMakeList.txt
```

## Control Robot
此機器人共有 10 個 topic, 1個 ServiceClient, 可對各 topic 送參數來控制機器人的行走, 擺臂, 速度等.

Code example 可以參考 `~/catkin_ws/src/tracked_robot/tracked_robot/src/Manual_mode.cpp`

### Topics
> **NOTE** 原則上, 在 source code 中看到 robot_ 都是跟行走馬達有關, leg_ 都是跟手臂馬達有關

#### 概覽
| Topic name   | Function                 | Type                      |
| ------------ | ------------------------ | ------------------------- |
| robot_motion | 控制行走方向/停止        | std_msgs::Int32           |
| robot_speed  | 設定行走速度             | std_msgs::Int32MultiArray |
| robot_VA     | 設定行走加速度           | std_msgs::Int32MultiArray |
| robot_MA     | 控制                     | std_msgs::Int32MultiArray |
| robot_HO     | 控制                     | std_msgs::Int32MultiArray |
| leg_motion   | 控制手臂抬起/放下        | std_msgs::Int32           |
| leg_speed    | 設定手臂速度             | std_msgs::Int32MultiArray |
| leg_VA       | 設定人手臂加速度         | std_msgs::Int32MultiArray |
| leg_MA       | 設定手臂角度             | std_msgs::Int32MultiArray |
| leg_HO       | 將目前手臂位置設為基準點 | std_msgs::Int32MultiArray |

#### robot_motion
Publish 1 個 0 ~ 4 之間的數以控制行走
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
- 左右行走馬達速度控制, 可改變行走速度
- Default: 350
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
std_msgs::Int32MultiArray robot_speed;
robot_speed.data.clear();

robot_speed.data.push_back(350); // set left speed to 350
robot_speed.data.push_back(350); // set right speed to 350
pub.publish(robot_speed);
```

#### robot_MA

#### robot_VA
- 左右行走馬達加速度控制, 可改變起步加速度/煞車減速度
- 預設值 200
- 盡量設定在 200 ~ 1000 間
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
| Value | Command              |
| ----- | -------------------- |
| 0     | 停止所有手臂馬達動作 |
| 1     | 前臂上               |
| 2     | 前臂下               |
| 3     | 後臂上               |
| 4     | 後臂下               |

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
- 控制手臂抬起/放下之速度
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
- 控制手臂角度
- 單位不是度
    - n = (欲轉角度) * 4550, n 即為需要 publish 的數值
    - 此公式為實驗量出之約略值, 可依需求自由調整
    - 如有使用到 leg_HO 來重置基準點, 請注意正負號

```c++
//example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 100);
std_msgs::Int32MultiArray leg_MA;

leg_MA.data.clear();
leg_MA.data.push_back(-26000); // set front arm to -26000
leg_MA.data.push_back(-27000); // set front arm to -27000
pub.publish(leg_MA);
```

#### leg_HO
- 將手臂馬達目前所在的角度設為 0 度 (基準點)
```c++

```

#### leg_VA
- 控制手臂馬達加速度, 可控制手臂抬起/放下之加速度
```c++

```


### Service Client
#### 概覽
| Service       | Function                   |
| ------------- | -------------------------- |
| joint_command | 控制雲台馬達上下左右之角度 |

#### joint_command
- type: **dynamixel_workbench_msgs::JointCommand**
- 控制雲台馬達上下左右之角度
- 控制上下: top; 控制左右: bottom
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
joint_command.request.id = 1;                          // control the bottom motor
joint_command.request.goal_position = bottom * PI/180; // the bottom control left-right direction of the motor
joint_command_client.call(joint_command);
joint_command.request.unit = "rad";
joint_command.request.id = 2;                          // control the top motor
  joint_command.request.goal_position = top * PI/180;  // the top control up-down direction of the motor
joint_command_client.call(joint_command);
```

## Compile
- 依照 ROS 的架構 Compile, 如需新增 Executable 記得改 CMakeList.txt
- 如需引入第三方函式庫, 需依照 Cmake 的語法寫入 CMakeList.txt 中再統一用 `catkin_make` 編譯

> **WARN** 在板子上修改 / 新增檔案前請先確認板子的系統時間有無跑掉, 否則很可能無法編譯成功 (modification in the future)
>    - 確認系統時間
>    ```bash
>   date
>    ```
>    - 改系統時間
>    ```bash
>    sudo date MMDDhhmmYYYY
>    # MM: 月
>    # DD: 日
>    # hh: 時 (24時制)
>    # mm: 分
>    # YYYY: 西元年
>    ```
>    - 更改檔案最後修改時間為目前系統時間
>    ```bash
>    touch <filename>
>    ```
