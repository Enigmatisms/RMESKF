#ifndef _SERIAL_COM_HPP
#define _SERIAL_COM_HPP

#define CHANGE_THRESHOLD 1

#include <chrono>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "serial_com/uwb.h"
// #define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
    #define serial_debug printf
    #define ros_debug ROS_INFO
#else
    #define serial_debug(...)
    #define ros_debug(...)
#endif
#define MAX_INFO_BYTES 1024*1024
//#define POSITION					//使用位置式的标签,默认取消，位置式云台会抽搐

// 通信协议 转换：电控将对应数据进行处理（乘以1000之类的），放到packet中，发送buffer。视觉收buffer后用union解包
union Translate {                   // 52字节 4 * 13
    uint8_t buffer[64];
    struct Packet {
        int16_t uwb[2];               // 4 * 4 UWB
        uint16_t uwb_angle;
        int16_t angles[2];          // 编码器pitch与云台陀螺仪yaw积分 （p y）
        int16_t accel[3];           // 加速度
        int16_t angular[3];         // 角速度
        int16_t magneto[3];         // 磁场
        uint8_t reserved[36];       // 暂时没有用        
    } packet;
};

class SerialCom{
public: 
    SerialCom();                                    //构造函数，在此打开串口
    ~SerialCom();                                   //析构
    ros::NodeHandle nh;                             //节点管理器
    ros::Publisher mag_pub;                         // 假设所有信息都是最新的，有用的
    ros::Publisher imu_pub;
    ros::Publisher uwb_pub;                         // 视觉端先不考虑 uwb数据重复的事情    
    serial::Serial ser;                             
public:
    /**
     * @brief 数据转为 uint_8数组（即char数组），以进行云台板数据收发
    */
    void sendGimbalInfo(); //回调函数，负责与云台板进行收发

private:
    int getDataFromSerial(
        sensor_msgs::Imu& imu,
        sensor_msgs::MagneticField& mag,
        serial_com::uwb& uwb
    );

    bool receiveData(
        const uint8_t *buffer,
        sensor_msgs::Imu& imu,
        sensor_msgs::MagneticField& mag,
        serial_com::uwb& uwb
    );    

    int serialOK(char *output);                                     //查找可用设备，查找到将返回0
private:
    Translate tl;

    std::ofstream uwb_freq;
    std::ofstream ser_freq;

    bool uwb_init;
    bool ser_init;

    double uwb_start_time;
    double ser_start_time;

    double uwb_cnt;
    double ser_cnt;

    int16_t old_x;
    int16_t old_y;
    uint16_t old_ang;
};
#endif //_SERIAL_COM_HPP