#include "robot.h"

Robot::Robot(int argc, char **argv, const char *node_name)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    pub_rob_speed = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
	pub_leg_speed = n.advertise<std_msgs::Int32MultiArray>("leg_speed", 100);
	pub_robot_MA = n.advertise<std_msgs::Int32MultiArray>("robot_MA", 100);
	pub_leg_MA   = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 100);
	pub_robot_HO = n.advertise<std_msgs::Int32MultiArray>("robot_HO", 100);
	pub_robot_VA = n.advertise<std_msgs::Int32MultiArray>("robot_VA", 100);

	pub_robot_motion = n.advertise<std_msgs::Int32>("robot_motion", 100);
	pub_leg_motion   = n.advertise<std_msgs::Int32>("leg_motion", 100);

	pub_leg_HO = n.advertise<std_msgs::Int32MultiArray>("leg_HO", 100);
	pub_leg_VA = n.advertise<std_msgs::Int32MultiArray>("leg_VA", 100);
	pub_robot_MR = n.advertise<std_msgs::Int32MultiArray>("robot_MR", 100);
	joint_command_client = n.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");

    dynamixel_current_top = TOP_HOME;
    dynamixel_current_bottom = BOTTOM_HOME;
}

Robot::~Robot() {}

void Robot::move(int motion)
{
    robot_motion.data = motion;
    pub_robot_motion.publish(robot_motion);

    char action[20];

    ROS_INFO("Robot action: %s", motion == STOP ? "stop" : \
                                 motion == FORWARD ? "forward" : \
                                 motion == BACK ? "back" : \
                                 motion == SPIN_LEFT ? "spin left" : "spin right");
    ros::spinOnce();
    return;
}

void Robot::setMoveSpeed(int left_speed, int right_speed)
{
    robot_speed.data.clear();
    robot_speed.data.push_back(left_speed);
    robot_speed.data.push_back(right_speed);
    pub_rob_speed.publish(robot_speed);
    ROS_INFO("Set robot speed, left: %d, right: %d", left_speed, right_speed);
    ros::spinOnce();
    return;
}

void Robot::setMoveAcceleration(int left_acc, int right_acc)
{
    robot_VA.data.clear();
    robot_VA.data.push_back(left_acc);
    robot_VA.data.push_back(right_acc);
    pub_robot_VA.publish(robot_VA);
    ROS_INFO("Set robot acceleration, left: %d, right: %d", left_acc, right_acc);
    ros::spinOnce();
    return;
}

void Robot::arm(int motion)
{
    leg_motion.data = motion;
    pub_leg_motion.publish(leg_motion);

    char action[20];

    ROS_INFO("Robot arm: %s", motion == FRONT_ARM_UP ? "front arm up" : \
                              motion == FRONT_ARM_DOWN ? "front arm down" : \
                              motion == BACK_ARM_UP ? "back arm up" : "back arm down");
    ros::spinOnce();
}

void Robot::setArmSpeed(int front_speed, int back_speed)
{
    leg_speed.data.clear();

    leg_speed.data.push_back(back_speed);
    leg_speed.data.push_back(front_speed);
    pub_leg_speed.publish(leg_speed);
    ROS_INFO("Set arm speed, front: %d, back: %d", front_speed, back_speed);
    ros::spinOnce();
    return;
}

void Robot::setArmAcceleration(int front_acc, int back_acc)
{
    leg_VA.data.clear();
    leg_VA.data.push_back(back_acc);
    leg_VA.data.push_back(front_acc);
    pub_leg_VA.publish(leg_VA);
    ROS_INFO("Set arm acceleration, front: %d, back: %d", front_acc, back_acc);
    ros::spinOnce();
    return;
}

void Robot::armAngle(int front, int back)
{
    leg_MA.data.clear();
    leg_MA.data.push_back(front * DEGREE_BASE);
    leg_MA.data.push_back(back * DEGREE_BASE);
    pub_leg_MA.publish(leg_MA);
    ROS_INFO("Arm go to angle, front: %d, back: %d", front, back);
    ros::spinOnce();
    return;
}

void Robot::setArmHomePosition()
{
    leg_HO.data.clear();
    leg_HO.data.push_back(0);
    leg_HO.data.push_back(0);
    pub_leg_HO.publish(leg_HO);
    ROS_INFO("Arm: set current position as home");
    ros::spinOnce();
    return;
}

void Robot::dynamixelInit()
{
    dynamixelPosition(TOP_HOME, BOTTOM_HOME);
    ROS_INFO("Dynamixel set to init position");
    ros::spinOnce();
    return;
}

void Robot::dynamixel(int motion)
{
    switch (motion)
    {
        case DYNAMIXEL_UP:
            dynamixelPosition(dynamixel_current_top + 10, dynamixel_current_bottom);
            ROS_INFO("Dynamixel up");
            break;
        case DYNAMIXEL_DOWN:
            dynamixelPosition(dynamixel_current_top - 10, dynamixel_current_bottom);
            ROS_INFO("Dynamixel down");
            break;
        case DYNAMIXEL_LEFT:
            dynamixelPosition(dynamixel_current_top, dynamixel_current_bottom + 10);
            ROS_INFO("Dynamixel turn left");
            break;
        case DYNAMIXEL_RIGHT:
            dynamixelPosition(dynamixel_current_top, dynamixel_current_bottom - 10);
            ROS_INFO("Dynamixel turn right");
            break;
        default:
            break;
    }
    ros::spinOnce();
    return;
}

void Robot::dynamixelPosition(int top, int bottom)
{
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = bottom * PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = top * PI/180;
    joint_command_client.call(joint_command);

    dynamixel_current_top = top;
    dynamixel_current_bottom = bottom;
    ROS_INFO("Dynamixel current position (degree), top: %d, bottom: %d", dynamixel_current_top, dynamixel_current_bottom);
    ros::spinOnce();
    return;
}
