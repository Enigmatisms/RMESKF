/*stm32云台板通信节点
作者:hqy
最后修改日期：2020.1.14 00:15
问题：SerialCom多个函数可以优化，但是现在还没有清晰的思路
*/

#include <csignal>
#include "serial_com/SerialCom.hpp"
#include "serial_com/uwb.h"

serial::Serial *ptr;
void signalHandler(int sig){
    uint8_t buffer[13];
    for(int i = 0; i < 13; ++i){
        buffer[i] = 0;
    }
    ptr->write(buffer, 13);
    ROS_INFO("Node SerialCom shutting down. Null msg sent.\n");
    ros::shutdown();
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "stm_com");
    SerialCom cm;
    ptr = &cm.ser;
    signal(SIGINT, signalHandler);
    ros::Rate rate(1000);
    while (ros::ok()) {
        cm.sendGimbalInfo();
        rate.sleep();
    }
    return 0;
}