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
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_info", 8);
    wheel_pub = nh.advertise<serial_com::chassis>("wheel_info", 8);
    uwb_pub = nh.advertise<serial_com::uwb>("uwb_info", 8);
    old_x = 0;
    old_y = 0;
    old_ang = 0;

    uwb_cnt = 0;
    ser_cnt = 0;
    uwb_init = false;
    ser_init = false;
    uwb_start_time = 0.0;
    ser_start_time = 0.0;

    uwb_freq.open("/home/xjturm/ESKF/uwb.txt", std::ios::out);
    ser_freq.open("/home/xjturm/ESKF/ser.txt", std::ios::out);

}

SerialCom::~SerialCom(){
    ser.close();
    double uwb_interval = (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - uwb_start_time) / uwb_cnt;
    double ser_interval = (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - ser_start_time) / ser_cnt;
    uwb_freq << uwb_interval << std::endl;
    ser_freq << ser_interval << std::endl;
    uwb_freq.close();
    ser_freq.close();
}

//串口收发数据测试
//此函数是subscriber的回调函数
void SerialCom::sendGimbalInfo(){
    serial_com::uwb uwb;
    serial_com::chassis wh;
    sensor_msgs::Imu imu;
    int serial_flag = getDataFromSerial(imu, wh, uwb);
    imu_pub.publish(imu);
    wheel_pub.publish(wh);
    if (serial_flag == ALL_OK) {
        if (uwb_init == false) {
            uwb_start_time = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
            uwb_init = true;
        }
        uwb_cnt ++;
        double uwb_interval = std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - uwb_start_time;
        uwb_freq << uwb_interval << std::endl;
        uwb_pub.publish(uwb);
    }
    ser_cnt ++;
}

//从串口接收数据
bool SerialCom::receiveData(
    const uint8_t *buffer,
    sensor_msgs::Imu& imu,
    serial_com::chassis& wh,
    serial_com::uwb& uwb
) {
    // 三轴陀螺仪（角加速度） + 三轴加速度（加速度） + 三轴磁场（方向），一次收64字节的包（有些是reserve的位置）
    for (int i = 0; i < 64; i++) {
        tl.buffer[i] = buffer[i];
    }
    // printf("UWB: %d, %d, %d\n", tl.packet.uwb[0],tl.packet.uwb[1], tl.packet.uwb_angle);
    // printf("Angle position: %d, %d\n", tl.packet.angles[0], tl.packet.angles[1]);
    // printf("Angular vel: %d, %d, %d\n", tl.packet.angular[0], tl.packet.angular[1], tl.packet.angular[2]);
    // 电控角速度是0.1度/s的，所以需要改一下，变成度后转弧度
    imu.angular_velocity.x = float(tl.packet.angular[0]) / 10 * 0.017453;
    imu.angular_velocity.y = float(tl.packet.angular[1]) / 10 * 0.017453;
    imu.angular_velocity.z = float(tl.packet.angular[2]) / 10 * 0.017453;

    /// 加速度
    // printf("Acceleration: %d, %d, %d\n", tl.packet.accel[0], tl.packet.accel[1], tl.packet.accel[2]);
    imu.linear_acceleration.x = float(tl.packet.accel[0]) / 1000.0 * 9.81;
    imu.linear_acceleration.y = float(tl.packet.accel[1]) / 1000.0 * 9.81;
    imu.linear_acceleration.z = float(tl.packet.accel[2]) / 1000.0 * 9.81;

    /// 轮速，从右下角开始顺时针1 2 3 4
    wh.v1 = float(tl.packet.wheels[0]) / 1000.0;
    wh.v2 = float(tl.packet.wheels[1]) / 1000.0;
    wh.v3 = float(tl.packet.wheels[2]) / 1000.0;
    wh.v4 = float(tl.packet.wheels[3]) / 1000.0;
    wh.yaw_angle = float(tl.packet.relative_angle) / 10 * 0.017453;     // 应该转化为弧度

    angle2Quat(0.0, 0.0, float (tl.packet.angles[1]) / 182.0, imu.orientation);      // 只需要收yaw

    int16_t now_x = tl.packet.uwb[0], now_y = tl.packet.uwb[1];
    uint16_t now_ang = tl.packet.uwb_angle;
    if (old_x == now_x && old_y == now_y && old_ang == now_ang) {       // no uwb
        uwb.x = 0.0;
        uwb.y = 0.0;
        uwb.z = 0.0;
        uwb.angle = 0.0;
        return false;
    }
    uwb.x = float(now_x) / 100.0;
    uwb.y = float(now_y) / 100.0;
    uwb.z = 0.0;
    uwb.angle = float(now_ang) / 100.0;                                 // 需要融合它了
    old_x = now_x;
    old_y = now_y;
    old_ang = now_ang;
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
    serial_com::chassis& wh,
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
            uint8_t result[64];
            for(int i = 0; i< 64 ; ++i)
                result[i] = res.data[i];                    //云台发来的数据全部使用
            if (receiveData(result, imu, wh, uwb) == true)
                return ALL_OK;
            return NO_UWB;
        }
        ROS_ERROR_STREAM("Buffer length less than 64.");
        return FAILED;
    }
    ROS_ERROR_STREAM("No data available.");
    return FAILED;
}