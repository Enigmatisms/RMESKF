#include "eskf_localization_wrapper.h"
#include "eskf_localizer/base_type.h"

#include <tf/tf.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

ESKFLocalizationWrapper::ESKFLocalizationWrapper(ros::NodeHandle nh)
{
	//obtain constants from ros param
	double am_noise, wm_noise, ab_noise, wb_noise;
	double x, y, z;

	nh.param("am_noise", am_noise, 1.0);
	nh.param("wm_noise", wm_noise, 0.05);
	nh.param("ab_noise", ab_noise, 1e-4);
	nh.param("wb_noise", wb_noise, 1e-4);

	nh.param("I_p_Uwb_x", x, 0.0);
	nh.param("I_p_Uwb_y", y, 0.0);
	nh.param("I_p_Uwb_z", z, 0.0);
	nh.param("a_filter_ratio", a_filter_ratio, 0.7);
	nh.param("w_filter_ratio", w_filter_ratio, 0.7);
	

	const Eigen::Vector3d I_p_Uwb(x, y, z);

	//initialize eskf_localizer with these parameters
	eskf_localizer_ = std::make_unique<ESKF_Localization::ESKF_Localizer>(
		am_noise, wm_noise, ab_noise, wb_noise, I_p_Uwb);

	imu_sub_ = nh.subscribe("imu_info", 8, &ESKFLocalizationWrapper::ImuCallback, this);
	uwb_sub_ = nh.subscribe("uwb_info", 8, &ESKFLocalizationWrapper::UwbPositionCallback, this);
	wheel_sub_ = nh.subscribe("wheel_info", 8, &ESKFLocalizationWrapper::WheelCallback, this);

	fused_pose_pub_ = nh.advertise<geometry_msgs::Pose>("/fused_pose", 8);
	fused_path_pub_ = nh.advertise<nav_msgs::Path>("/fused_path", 8);
	uwb_path_pub_ = nh.advertise<nav_msgs::Path>("/uwb_path", 8);

	file.open("/home/sentinel/ESKF/origin.txt", std::ios::out);
    ang_file.open("/home/sentinel/ESKF/angs.txt", std::ios::out);

	old_accel.setZero();
	old_data_init = false;
	old_angvel = 0.0;
	start_time = 0.0;
}

void ESKFLocalizationWrapper::reset() {
	old_accel.setZero();
	old_data_init = false;
	old_angvel = 0.0;
	eskf_localizer_->reset();
}

ESKFLocalizationWrapper::~ESKFLocalizationWrapper()
{
	file.close();
	ang_file.close();
}

void ESKFLocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{
	ESKF_Localization::ImuDataPtr imu_data_ptr = std::make_shared<ESKF_Localization::ImuData>();
	double now_time = std::chrono::system_clock::now().time_since_epoch().count(), interval = 0.0;
	now_time /= 1e9;
	imu_data_ptr->timestamp = ros::Time::now().toSec();
	if (old_data_init == false) {
		old_data_init = true;
		old_accel << imu_msg_ptr->linear_acceleration.x, imu_msg_ptr->linear_acceleration.y, -9.81;
		old_angvel = imu_msg_ptr->linear_acceleration.z;
		start_time = now_time; 
	}
	Eigen::Vector3d now_accel = (1 - a_filter_ratio) * Eigen::Vector3d(imu_msg_ptr->linear_acceleration.x,
						imu_msg_ptr->linear_acceleration.y, -9.81) + a_filter_ratio * old_accel;
	now_accel(2) = -9.81;
	imu_data_ptr->accel = now_accel;
	old_accel = now_accel.eval();

	printf("Accel Actual: %f, %f, %f\n", imu_msg_ptr->linear_acceleration.x, imu_msg_ptr->linear_acceleration.y, imu_msg_ptr->linear_acceleration.z);
	interval = now_time - start_time;
	file << interval << "," << imu_msg_ptr->linear_acceleration.x << "," <<imu_msg_ptr->linear_acceleration.y << "," << imu_msg_ptr->linear_acceleration.z << std::endl;
	ang_file << interval << "," << imu_msg_ptr->angular_velocity.x << "," << imu_msg_ptr->angular_velocity.y << "," << imu_msg_ptr->angular_velocity.z << std::endl;
	imu_data_ptr->gyro << 0, 0, imu_msg_ptr->angular_velocity.z * (1 - w_filter_ratio) + w_filter_ratio * old_angvel;
	old_angvel = imu_data_ptr->gyro.z();
	ang_file << interval << "," << imu_data_ptr->gyro.x() << "," << imu_data_ptr->gyro.y() << "," << imu_data_ptr->gyro.z() << std::endl;

	imu_data_ptr->quat = Eigen::Quaterniond(imu_msg_ptr->orientation.w,
											imu_msg_ptr->orientation.x,
											imu_msg_ptr->orientation.y,
											imu_msg_ptr->orientation.z);
	eskf_localizer_->processImuData(imu_data_ptr);
	publishState();
}

/// @todo UWB的角度信息只有一个平面角，尚不清楚如何进行融合
void ESKFLocalizationWrapper::UwbPositionCallback(const serial_com::uwbConstPtr &uwb_msg_ptr)
{
	printf("UWB from serial com: %f, %f, %f\n", uwb_msg_ptr->x, uwb_msg_ptr->y, uwb_msg_ptr->z);
	ESKF_Localization::UwbPositionDataPtr uwb_data_ptr = std::make_shared<ESKF_Localization::UwbPositionData>();
	uwb_data_ptr->timestamp = ros::Time::now().toSec();
	uwb_data_ptr->orient = uwb_msg_ptr->angle;
	uwb_data_ptr->z(0) = uwb_msg_ptr->x;
	uwb_data_ptr->z(1) = uwb_msg_ptr->y;
	uwb_data_ptr->z(2) = 0.0;
	eskf_localizer_->processUwbData(uwb_data_ptr);
	publishState(true);
}

void ESKFLocalizationWrapper::WheelCallback(const serial_com::chassisConstPtr& wh_msg_ptr)
{
	ESKF_Localization::WheelDataPtr wh_data_ptr = std::make_shared<ESKF_Localization::WheelData>();
	wh_data_ptr->timestamp = ros::Time::now().toSec();

	wh_data_ptr->wheel << wh_msg_ptr->v1, wh_msg_ptr->v2, wh_msg_ptr->v3, wh_msg_ptr->v4;
	wh_data_ptr->yaw_angle = wh_msg_ptr->yaw_angle;

	eskf_localizer_->processWheelData(wh_data_ptr);
	publishState();
}

void ESKFLocalizationWrapper::addStateToPath(ESKF_Localization::State *state)
{
	ros_path_.header.frame_id = "world";
	ros_path_.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped pose;
	pose.header = ros_path_.header;

	pose.pose.position.x = state->G_p_I[0];
	pose.pose.position.y = state->G_p_I[1];
	pose.pose.position.z = state->G_p_I[2];

	const Eigen::Quaterniond G_q_I(state->G_R_I);
	pose.pose.orientation.x = G_q_I.x();
	pose.pose.orientation.y = G_q_I.y();
	pose.pose.orientation.z = G_q_I.z();
	pose.pose.orientation.w = G_q_I.w();

	ros_path_.poses.push_back(pose);
}

/// 未融合的UWB位置显示
void ESKFLocalizationWrapper::addUWB2Path(Eigen::Vector3d uwb_pos)
{
	uwb_path_.header.frame_id = "world";
	uwb_path_.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped pose;
	pose.header = uwb_path_.header;

	pose.pose.position.x = uwb_pos.x();
	pose.pose.position.y = uwb_pos.y();
	pose.pose.position.z = uwb_pos.z();

	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;
	uwb_path_.poses.push_back(pose);
}

void ESKFLocalizationWrapper::publishState(bool pub_uwb)
{
	//publish fused states
	fused_pose_pub_.publish(eskf_localizer_->getFusedPose());
	addStateToPath(eskf_localizer_->getState());
	fused_path_pub_.publish(ros_path_);

	if (pub_uwb == true)
	{
		addUWB2Path(eskf_localizer_->getUWBPos());
		uwb_path_pub_.publish(uwb_path_);
	}

	//publish transform
	ESKF_Localization::State *state = eskf_localizer_->getState();
	tf::Transform tf;
	tf.setOrigin(tf::Vector3(state->G_p_I[0], state->G_p_I[1], state->G_p_I[2]));
	Eigen::Quaterniond quat(state->G_R_I);
	tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w()); //Caution! tf::Quaternion is in (x,y,z,w)
	tf.setRotation(q);
	br_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "XIMU"));
}