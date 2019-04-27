#include "ros/ros.h"
#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <iostream> //lib use in c++
#include "conio.h"

#include "robot.h"

#define NODE_NAME "Manual_node_use_api"
#define HELP_MSG "==========================\n"                           \
                 "Key usage:\n"                                           \
                 " h: Show this message\n"                                \
                 " space: stop all motion\n"                              \
                 "---------\n"                                            \
                 " Up arrow:    Go front\n"                               \
                 " Down arrow:  Go back\n"                                \
                 " Left arrow:  Spin left\n"                              \
                 " Right arrow: Spin right\n"                             \
                 " t: set move speed\n"                                   \
                 " y: set move acceleration\n"                            \
                 " n: move speed +500 (max 2000)\n"                       \
                 " m: move speed -500 (min 100)\n"                        \
                 " b: reset move speed (700)\n"                           \
                 "---------\n"                                            \
                 " a: front arm up\n"                                     \
                 " z: front arm down\n"                                   \
                 " s: back arm up\n"                                      \
                 " x: back arm down\n"                                    \
                 " c: set arm speed\n"                                    \
                 " v: set arm acceleration\n"                             \
                 " d: set the position of arm\n"                          \
                 " f: set current arm position as home position of arm\n" \
                 "---------\n"                                            \
                 " o: init dynamixel position\n"                          \
                 " i: dynamixel down\n"                                   \
                 " k: dynamixel up\n"                                     \
                 " j: dynamixel left\n"                                   \
                 " l: dynamixel right\n"                                  \
                 "---------\n"                                            \
                 " q: Quit the program\n"                                 \
                 "==========================\n"

#define KEY_UP    65
#define KEY_DOWN  66
#define KEY_RIGHT 67
#define KEY_LEFT  68

#define MOVE_MAX_SPEED 2000
#define MOVE_MIN_SPEED 100
#define MOVE_DEFAULT_SPEED 700

int main(int argc, char **argv)
{
    // init the ros node
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    Robot robot(n);

    // set the frequency of the while loop to 100hz
    // avoid to publish the commands too frequently
    ros::Rate loop_rate(100);

    printf(HELP_MSG);

    int current_speed = MOVE_DEFAULT_SPEED;
    int left_speed, right_speed, left_acc, right_acc;
    int front_angle, back_angle, front_speed, back_speed, front_acc, back_acc;

    char user_input;
    while(ros::ok())
    {
        user_input = getch();
        switch (user_input)
        {
            case 'h':
                printf(HELP_MSG);
                break;
            case KEY_UP:
                robot.move(FORWARD);
                break;
            case KEY_DOWN:
                robot.move(BACK);
                break;
            case KEY_RIGHT:
                robot.move(SPIN_RIGHT);
                break;
            case KEY_LEFT:
                robot.move(SPIN_LEFT);
                break;
            case ' ':
                robot.move(STOP);
                break;
            case 'a':
                robot.arm(FRONT_ARM_UP);
                break;
            case 'z':
                robot.arm(FRONT_ARM_DOWN);
                break;
            case 's':
                robot.arm(BACK_ARM_UP);
                break;
            case 'x':
                robot.arm(BACK_ARM_DOWN);
                break;
            case 'f':
                robot.setArmHomePosition();
                break;
            case 'd':
                printf("Set arm position (angle):\n");
                printf("front (degrees): ");
                scanf("%d", &front_angle);
                printf("back (degrees): ");
                scanf("%d", &back_angle);
                robot.setArmAngle(front_angle, back_angle);
                break;
            case 't':
                printf("Set move speed:\n");
                printf("left:");
                scanf("%d", &left_speed);
                printf("right:");
                scanf("%d", &right_speed);
                robot.setMoveSpeed(left_speed, right_speed);
                break;
            case 'y':
                printf("Set move acceleration:\n");
                printf("left:");
                scanf("%d", &left_acc);
                printf("right:");
                scanf("%d", &right_acc);
                robot.setMoveAcceleration(left_acc, right_acc);
                break;
            case 'n':
                if(current_speed + 500 > MOVE_MAX_SPEED)
                    current_speed = MOVE_MAX_SPEED;
                else
                    current_speed += 500;
                robot.setMoveSpeed(current_speed, current_speed);
                break;
            case 'm':
                if((current_speed - 500) < MOVE_MIN_SPEED || current_speed < 500)
                    current_speed = MOVE_MIN_SPEED;
                else
                    current_speed -= 500;
                robot.setMoveSpeed(current_speed, current_speed);
                break;
            case 'b':
                current_speed = MOVE_DEFAULT_SPEED;
                robot.setMoveSpeed(current_speed, current_speed);
                break;
            case 'c':
                printf("Set arm speed:\n");
                printf("front:");
                scanf("%d", &front_speed);
                printf("back:");
                scanf("%d", &back_speed);
                robot.setArmSpeed(front_speed, back_speed);
                break;
            case 'v':
                printf("Set arm acceleration:\n");
                printf("front:");
                scanf("%d", &front_acc);
                printf("back:");
                scanf("%d", &back_acc);
                robot.setArmAcceleration(front_acc, back_acc);
                break;
            case 'o':
                robot.dynamixelInit();
                break;
            case 'i':
                robot.dynamixel(DYNAMIXEL_DOWN);
                break;
            case 'k':
                robot.dynamixel(DYNAMIXEL_UP);
                break;
            case 'j':
                robot.dynamixel(DYNAMIXEL_LEFT);
                break;
            case 'l':
                robot.dynamixel(DYNAMIXEL_RIGHT);
                break;
            case 'q':
                printf("Quit %s\n", NODE_NAME);
                exit(0);
                break;
            default:
                break;
        }

        // loop rate 100hz
        loop_rate.sleep();
    }
    return 0;
}
