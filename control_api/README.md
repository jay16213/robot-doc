Robot Control API
===

This API define a C++ class named `Robot`.

## Usage
First, Move `robot.h`, `robot.cpp` to your source code.

```c++
// in your source code, include robot.h
include "robot.h"

int main(argc, char **argv) {

    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle n;

    // declare a Robot instance, need to pass NodeHandle to the constructor for initialization
    Robot robot(n);

    // do something
}
```
Remember to compile your source code with `robot.cpp` together.

## Member Functions
### void move(int motion)
```c++
robot.move(FORWARD);
robot.move(BACK);
robot.move(SPIN_LEFT);
robot.move(SPIN_RIGHT);
robot.move(STOP); // stop all action, included arm motion
```

### void setMoveSpeed(int left_speed, int right_speed)
```c++
robot.setMoveSpeed(1000, 1000);
```

### void setMoveAcceleration(int left_acc, int right_acc)
```c++
robot.setMoveAcceleration(500, 500);
```

### void arm(int motion);
```c++
robot.arm(FRONT_ARM_UP);
robot.arm(FRONT_ARM_DOWN);
robot.arm(BACK_ARM_UP);
robot.arm(BACK_ARM_DOWN);
```

### void setArmSpeed(int front_speed, int back_speed)
```c++
robot.setArmSpeed(1000, 1000);
```

### void setArmAcceleration(int front_acc, int back_acc)
```c++
robot.setArmAcceleration(500, 500);
```

### void setArmAngle(int front, int back)
- front, back is in degree
```c++
robot.armAngle(45, 45);
```

### void setArmHomePosition()
```c++
// this function will set current arm position as arm home position
robot.setArmHomePosition();
```

### void dynamixelInit()
```c++
// set dynamixel to init position
robot.dynamixelInit();
```


### void dynamixel(int motion)
```c++
robot.dynamixel(DYNAMIXEL_UP);
robot.dynamixel(DYNAMIXEL_DOWN);
robot.dynamixel(DYNAMIXEL_LEFT);
robot.dynamixel(DYNAMIXEL_RIGHT);
```

### void dynamixelPosition(int top, int bottom)
```c++
robot.dynamixelPosition(114, 45);

// the following is equivalent to call robot.dynamixelInit();
robot.dynamixelPosition(TOP_HOME, BOTTOM_HOME);
```
