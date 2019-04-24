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


## Robot Control API
If your application is written in C/C++, you may want to use [Robot Control API](control_api/) to control the robot.

This API is a wrapper for topics and publishers which are used in robot controlling. With this API, you don't need to understand what and how a topic is published.

For more information that how to control the robot via topics (publishers), please read the following document.

## Robot Control
There 10 topics and 1 service client for this robot. We can publish our command to these topics to control the motion, speed, or arm angle of the robot.

For more detail Code example, please read `~/catkin_ws/src/tracked_robot/tracked_robot/src/Manual_mode.cpp`


### Topics
> **NOTE** 原則上, 在 source code 中看到 robot_ 都是跟行走馬達有關, leg_ 都是跟手臂馬達有關

#### Overview
| Topic name   | Function                                 | Type                      |
| ------------ | ---------------------------------------- | ------------------------- |
| robot_motion | control the motion of the robot          | std_msgs::Int32           |
| robot_speed  | set speed of the robot                   | std_msgs::Int32MultiArray |
| robot_VA     | set acceleration of the robot            | std_msgs::Int32MultiArray |
| robot_MA     | set the position of moving motor         | std_msgs::Int32MultiArray |
| robot_HO     | Define home position of the moving motor | std_msgs::Int32MultiArray |
| leg_motion   | control the motion of arms               | std_msgs::Int32           |
| leg_speed    | set speed of arm                         | std_msgs::Int32MultiArray |
| leg_VA       | set acceleration of arm                  | std_msgs::Int32MultiArray |
| leg_MA       | set the absolute position of arm         | std_msgs::Int32MultiArray |
| leg_HO       | Define home position of arm              | std_msgs::Int32MultiArray |

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
- Default: 700
- at speed 1000, it costs about 7 seconds to move 1 meter
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
std_msgs::Int32MultiArray robot_speed;
// clear old data before using
robot_speed.data.clear();

// robot_speed.data[0] is left
// robot_speed.data[1] is right
robot_speed.data.push_back(350); // set left speed to 350
robot_speed.data.push_back(350); // set right speed to 350
pub.publish(robot_speed);
```

#### robot_VA
- Control the acceleration of robot moving
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
- default: 1000
- do not set too fast
```c++
// example
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_speed", 100);

std_msgs::Int32MultiArray leg_speed;
// clear old data before using
leg_speed.data.clear();

// leg_speed.data[0] is back
// leg_speed.data[1] is front
leg_speed.data.push_back(1000); // set back arm speed to 1000
leg_speed.data.push_back(1000); // set front arm speed to 1000
pub.publish(leg_speed);
```

#### leg_VA
- Control the acceleration of arm motors
```c++
ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("leg_VA", 100);
std_msgs::Int32MultiArray leg_VA;
leg_VA.data.clear();

leg_VA.data.push_back(200); // set back arm acceleration to 200
leg_VA.data.push_back(200); // set front arm acceleration to 200
pub.publish(leg_VA);
```

#### leg_MA
- Set the absolute position of arm
- Formula
    - `n = (target_position) * 4550`. **target_position** is degree and  **n** is the value you need to publish
    - This formula is an approximate result obtained by experiment and can   be adjusted freely if need.
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
- Set the current position of arm as home position
```c++
pub = n.advertise<std_msgs::Int32MultiArray>("leg_HO", 1);
std_msgs::Int32MultiArray leg_HO;
leh_HO.data.clear();

leg_HO.data.push_back(0); // set current position of the front arm as 0 (home)
leg_HO.data.push_back(0); // set current position of the back arm as 0 (home)
pub.publish(leg_HO);
```

### Service Client
#### Overview
| Service       | Function                       | Type                                   |
| ------------- | ------------------------------ | -------------------------------------- |
| joint_command | control the angle of dynamixel | dynamixel_workbench_msgs::JointCommand |

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
