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
#include "hall_sensors.h"
#include "stepper_motor_joint.h"
// can
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <vector>

#define RIGHT 1
#define LEFT 1



int main(int argc, char **argv)
{
    float value[16];
    ihb::ForceSensors forceSensors;
    while (1)
    {
        // /* code */
        // float value = forceSensors.getValue(0);
        // 获取16个
        // std::cout << "va: " ;
        for(int i = 0;i<20;i++){
            // value[i] = forceSensors.getValue(i);
            std::cout.width(6);
		    std::cout.fill(' ');
            std::cout << forceSensors.getValue(i);
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    return 0;
}