#include "eskf_localizer/eskf_localizer.h"
#include "serial_com/SerialCom.hpp"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/MagneticField.h>

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

    nav_msgs::Path ros_path_;
	nav_msgs::Path uwb_path_;

	tf::TransformBroadcaster br_;

	std::unique_ptr<ESKF_Localization::ESKF_Localizer> eskf_localizer_;

	void addStateToPath(ESKF_Localization::State* state);
	void addUWB2Path(Eigen::Vector3d uwb_pos);
	void publishState(bool pub_uwb = false);
};
