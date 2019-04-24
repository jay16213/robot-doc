#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <dynamixel_workbench_msgs/JointCommand.h>

#define STOP       0
#define FORWARD    1
#define BACK       2
#define SPIN_LEFT  3
#define SPIN_RIGHT 4

#define FRONT_ARM_UP   1
#define FRONT_ARM_DOWN 2
#define BACK_ARM_UP    3
#define BACK_ARM_DOWN  4

#define DYNAMIXEL_UP    1
#define DYNAMIXEL_DOWN  2
#define DYNAMIXEL_LEFT  3
#define DYNAMIXEL_RIGHT 4

#define TOP_HOME    134
#define BOTTOM_HOME 45
#define PI 3.14159

#define DEGREE_BASE 4550

#define DEFAULT
class Robot {
public:
    Robot(int argc, char **argv, const char* node_name);
    ~Robot();

    void move(int motion);
    void setMoveSpeed(int left_speed, int right_speed);
    void setMoveAcceleration(int left_acc, int right_acc);

    void arm(int motion);
    void setArmSpeed(int front_speed, int back_speed);
    void setArmAcceleration(int front_acc, int back_acc);
    void armAngle(int front, int back);
    void setArmHomePosition();

    void dynamixelInit();
    void dynamixel(int motion);
    void dynamixelPosition(int top, int bottom);

    int dynamixel_current_top;
    int dynamixel_current_bottom;

private:
    // Publishers
	ros::Publisher pub_rob_speed;
	ros::Publisher pub_leg_speed;
	ros::Publisher pub_robot_MA;
	ros::Publisher pub_leg_MA;
	ros::Publisher pub_robot_HO;
	ros::Publisher pub_robot_VA;

	ros::Publisher pub_robot_motion;
	ros::Publisher pub_leg_motion;

	ros::Publisher pub_leg_HO;
	ros::Publisher pub_leg_VA;
	ros::Publisher pub_robot_MR;
	ros::ServiceClient joint_command_client;

    // data to publish
    std_msgs::Int32 robot_motion,leg_motion;
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;
    dynamixel_workbench_msgs::JointCommand joint_command;
};
#endif
