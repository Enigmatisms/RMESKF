#include "serial_com/SerialCom.hpp"

#define ALL_OK 0
#define NO_UWB 1
#define FAILED -1
#define RESERVE 2       // 还可能没有磁力计信息

static void angle2Quat(float r, float p, float y, geometry_msgs::Quaternion& quat, bool is_rad = false){
    if (is_rad == false){
        r *= 0.0174533;       // pi / 180 = 0.0174533
        p *= 0.0174533;
        y *= 0.0174533;     
    }
    quat.w = cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2);  // w
    quat.x = sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2);  // x
    quat.y = cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2);  // y
    quat.z = cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2);  // z
}

SerialCom::SerialCom(){
    try{
        char path[128]="/dev/serial/by-id/";
        if(!serialOK(path)==0){
            ROS_ERROR_STREAM("Unable to open port. Not exception. Possible null path detected.");
            exit(-1);
        }
        ser.setPort(path);
        std::cout<< path <<std::endl;     
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setBaudrate(115200);
        //串口设置timeout
        ser.setTimeout(to);
        ser.open();
        printf("Serial port opened.\n");
    }
    catch (serial::IOException &e){
        ROS_ERROR_STREAM("Unable to open port with Exception.");
        exit(-1);
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else{
        ROS_ERROR_STREAM("Open serial port failed.");
        exit(-1);
    }
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("msg_info", 10);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_info", 10);
    uwb_pub = nh.advertise<serial_com::uwb>("uwb_info", 4);
}

SerialCom::~SerialCom(){
    ser.close();
}

//串口收发数据测试
//此函数是subscriber的回调函数
void SerialCom::sendGimbalInfo(){
    serial_com::uwb uwb;
    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField mag;
    int serial_flag = getDataFromSerial(imu, mag, uwb);
    if (serial_flag == ALL_OK) {        // all information acquired.
        imu_pub.publish(imu);
        mag_pub.publish(mag);
        uwb_pub.publish(uwb);
    }
    else if (serial_flag == NO_UWB) {   // no uwb position
        imu_pub.publish(imu);
        mag_pub.publish(mag);
    }
}

//从串口接收数据
bool SerialCom::receiveData(
    const uint8_t *buffer,
    sensor_msgs::Imu& imu,
    sensor_msgs::MagneticField& mag,
    serial_com::uwb& uwb
) {
    // 三轴陀螺仪（角加速度） + 三轴加速度（加速度） + 三轴磁场（方向）
    for (int i = 0; i < 64; i++) {
        tl.buffer[i] = buffer[i];
    }
    imu.angular_velocity.x = float(tl.packet.angular[0]) / 1000.0;
    imu.angular_velocity.y = float(tl.packet.angular[1]) / 1000.0;
    imu.angular_velocity.z = float(tl.packet.angular[2]) / 1000.0;

    imu.linear_acceleration.x = float(tl.packet.accel[0]) / 1000.0;
    imu.linear_acceleration.y = float(tl.packet.accel[1]) / 1000.0;
    imu.linear_acceleration.z = float(tl.packet.accel[2]) / 1000.0;

    /// 磁力计
    mag.magnetic_field.x = float(tl.packet.magneto[0]) / 1000.0;
    mag.magnetic_field.y = float(tl.packet.magneto[1]) / 1000.0;
    mag.magnetic_field.z = float(tl.packet.magneto[2]) / 1000.0;

    bool uwb_update = false;
    for (int i = 0; i < 3; i++) {       // 只要有非0的数据，就能说明uwb给出的数据是更新了的
        if (std::abs(tl.packet.uwb[i]) > 1e-5) {
            uwb_update = true;
            break;
        }
    }
    angle2Quat(0.0, float(tl.packet.angles[0]) / 91.0,
            float (tl.packet.angles[1]) / 91.0, imu.orientation);
    
    if (uwb_update == false) {          // uwb频率10Hz，电控以高频率（160+Hz发送IMU信息时，可能没有UWB信息）
        uwb.x = 0.0;
        uwb.y = 0.0;
        uwb.z = 0.0;
        uwb.angle = 0.0;                // 没有UWB，无法与云台yaw进行融合（磁力计融合不在此处）
        return false;
    }
    
    // imu.orientation.x = 0.0;
    // imu.orientation.y = 0.0;
    // imu.orientation.z = 0.0;
    // imu.orientation.w = 0.0;
    uwb.x = tl.packet.uwb[0];
    uwb.y = tl.packet.uwb[1];
    uwb.z = tl.packet.uwb[2];
    uwb.angle = tl.packet.uwb[3];
    /// @todo 这个应该在UWB correct 函数里面进行融合，只融合特定方向
    return true;
}

//查找需要的串口
int SerialCom::serialOK(char *output){
    std::cout<<"Directory dev/serial exists."<<std::endl;
    DIR *dir = opendir("/dev/serial/by-id");
    if(!dir){                                                   //是否能正常打开目录
        std::cout<<"Open directory error."<<std::endl;
        return -1;
    }
    struct dirent* files;
    while((files = readdir(dir))){                              //查找名称长度大于5的为USB设备
        for(int i = 0; i < 256 && files->d_name[i] > 0; ++i){
            if(i>5){
                strcat(output, files->d_name);                  // 云台板断电，重新给予权限
                std::string uwband = "echo \"bfjg\" | sudo -S chmod 777 " + std::string(output);
                system(uwband.c_str());
                return 0;
            }
        }
    }
    return -1;                                                  //所有设备名称不符合要求
}

int SerialCom::getDataFromSerial(
    sensor_msgs::Imu& imu,
    sensor_msgs::MagneticField& mag,
    serial_com::uwb& uwb)
{  //输出是两个float值+一个状态码
    if(ser.waitReadable()){                                    //缓冲区内有信息                                    
        size_t num = ser.available();                       //获取缓冲区内数据量
        if(num >= 64){
            std_msgs::String res;
            res.data = ser.read(num);                         //一次性读出缓冲区里所有的数据
            if(res.data.size() < 64)
                ROS_ERROR_STREAM("There has been a Read Error.\n");
            serial_debug("Data received.\n");
            uint8_t result[52];
            for(int i = 0; i< 52 ; ++i)
                result[i] = res.data[i];                    //云台发来的数据全部使用
            if (receiveData(result, imu, mag, uwb) == true)
                return ALL_OK;
            return NO_UWB;
        }
        ROS_ERROR_STREAM("Buffer length less than 64.");
        return FAILED;
    }
    ROS_ERROR_STREAM("No data available.");
    return FAILED;
}