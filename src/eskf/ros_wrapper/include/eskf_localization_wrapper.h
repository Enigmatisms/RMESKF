#include "eskf_localizer/eskf_localizer.h"
#include "serial_com/SerialCom.hpp"

#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/MagneticField.h>

#define DRIFT_THRESHOLD 0.05		// 低于0.05的IMU加速度初始平均值不算零漂

class ESKFLocalizationWrapper{

public:
	ESKFLocalizationWrapper(ros::NodeHandle nh);
	~ESKFLocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void UwbPositionCallback(const serial_com::uwbConstPtr& uwb_msg_ptr);
    void MagCallback(const sensor_msgs::MagneticFieldConstPtr& mag_msg_ptr);

private:
	ros::Subscriber imu_sub_;
	ros::Subscriber uwb_sub_;
	ros::Subscriber mag_sub_;
	ros::Publisher fused_pose_pub_;
	ros::Publisher fused_path_pub_;
	ros::Publisher uwb_path_pub_;
	ros::Publisher fused_odom_pub_;
	std::ofstream file;

    nav_msgs::Path ros_path_;
	nav_msgs::Path uwb_path_;

	tf::TransformBroadcaster br_;
	Eigen::Vector3d old_accel;

	std::unique_ptr<ESKF_Localization::ESKF_Localizer> eskf_localizer_;

	void addStateToPath(ESKF_Localization::State* state);
	void addUWB2Path(Eigen::Vector3d uwb_pos);
	void publishState(bool pub_uwb = false);
private:
	bool accel_init;
	double filter_ratio;
	double start_time;
};
