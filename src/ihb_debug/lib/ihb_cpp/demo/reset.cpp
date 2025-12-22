#include <iostream>
// #include "CanDriver.hpp"
#include <string>
#include <csignal>

#include <sstream>
#include <queue>
#include <thread>
#include "muscle_joint.h"
#include "CanDriver.hpp"
#include "servo_motorJoint.h" // 其实头文件不该大写的，但是我懒得改了
#include "force_sensors.h"
// can
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>


int main(int argc, char **argv)
{
    for(int i = 0; i < 5; i++)
    {
        std::cout << "reseting " << i << std::endl;
        ihb::ServoMotorJoint servo(i);
        servo.setPositonWithSpeedLimit(0, 400);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}