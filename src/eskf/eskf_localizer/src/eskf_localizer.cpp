#include "eskf_localizer/eskf_localizer.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"

#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

namespace ESKF_Localization{
	ESKF_Localizer::ESKF_Localizer(const double am_noise, const double wm_noise,
			const double ab_noise, const double wb_noise, Eigen::Vector3d I_p_Uwb){
		state_.G_p_I = Eigen::Vector3d::Zero();
		state_.G_v_I = Eigen::Vector3d::Zero();
		state_.G_R_I = Eigen::Matrix3d::Identity();
		state_.ab = Eigen::Vector3d::Zero();
		state_.wb = Eigen::Vector3d::Zero();

		initializer_ = std::make_unique<Initializer>(20,20,20,&state_);
		Eigen::Vector3d g(0,0,-9.81);
		imu_processor_ = std::make_unique<ImuProcessor>(am_noise,wm_noise,ab_noise,wb_noise,g);
		uwb_processor_ = std::make_unique<UwbProcessor>(I_p_Uwb);
		wh_processor_ = std::make_unique<WheelProcessor>(0.7 * Eigen::Matrix3d::Identity());
		last_t_ = ros::Time::now().toSec();
		file.open("/home/sentinel/ESKF/filtered.txt", std::ios::out);
		start_time = 0.0;
		start_t_set = false;
	}

	void ESKF_Localizer::processImuData(ImuDataPtr imu_data){
		double now_time = std::chrono::system_clock::now().time_since_epoch().count(), interval = 0.0;
		now_time /= 1e9;
		if (start_t_set == false) {
			start_time = now_time; 
			start_t_set = true;
		}
		interval = now_time - start_time;
		
		if(!initializer_->is_initialized()){
			initializer_->Imu_initialize(imu_data);
			printf("Initializing accel: %f, %f, %f\n", imu_data->accel.x(), imu_data->accel.y(), imu_data->accel.z());
			file << interval << "," << imu_data->accel.x() << "," << imu_data->accel.y() << "," << imu_data->accel.z() << std::endl;
			return;
		}
		else {
			Eigen::Vector3d accel = imu_data->accel;
			accel(0) -= initializer_->accel_x_mean;				// substract bias
			accel(1) -= initializer_->accel_y_mean;
			accel(0) *= std::cos(initializer_->pitch_angle);	// projection
			accel(1) *= std::cos(initializer_->roll_angle);
			imu_data->accel = accel.eval();
			printf("Undistorted: %f, %f, %f\n", imu_data->accel.x(), imu_data->accel.y(), imu_data->accel.z());
			file << interval << "," << imu_data->accel.x() << "," << imu_data->accel.y() << "," << imu_data->accel.z() << std::endl;
		}

		double t = imu_data->timestamp;
		double dt = t - last_t_;
		last_t_ = t;
		if (dt >= 0.5 || dt <= 0){dt = 0.01;}

		imu_processor_->Imu_predict(imu_data,dt,&state_);
	}

	void ESKF_Localizer::processUwbData(UwbPositionDataPtr uwb_data){
		if(!initializer_->is_initialized()){
			initializer_->Uwb_initialize(uwb_data);
			printf("Not initialized\n");
			return;
		}
		printf("Start correcting sequence.\n");
		uwb_processor_->Uwb_correct(uwb_data, &state_, uwb_pos);
	}

	void ESKF_Localizer::processWheelData(WheelDataPtr wh_data){
		if(!initializer_->is_initialized()){
			initializer_->Wheel_initialize(wh_data);
			return;
		}
		wh_processor_->Wheel_correct(wh_data, &state_);
	}

	State* ESKF_Localizer::getState(){
		return &state_;
	}

	Eigen::Vector3d ESKF_Localizer::getUWBPos() const {
		return uwb_pos;
	}

    geometry_msgs::Pose ESKF_Localizer::getFusedPose(){
    	geometry_msgs::Pose fusedPose;
    	fusedPose.position.x = state_.G_p_I.x();
    	fusedPose.position.y = state_.G_p_I.y();
    	fusedPose.position.z = state_.G_p_I.z();

    	Eigen::Quaterniond q(state_.G_R_I);

    	fusedPose.orientation.w = q.w();
    	fusedPose.orientation.x = q.x();
    	fusedPose.orientation.y = q.y();
    	fusedPose.orientation.z = q.z();

    	return fusedPose;
    }

    nav_msgs::Odometry ESKF_Localizer::getFusedOdometry(){
    	nav_msgs::Odometry odom;

    	odom.header.frame_id = "world";
    	odom.header.stamp = ros::Time::now();
    	odom.child_frame_id = "XIMU";

    	odom.pose.pose = getFusedPose();

    	Eigen::Vector3d I_v = state_.G_R_I.transpose() * state_.G_v_I;//global velocity to local velocity
    	//Eigen::Vector3d I_w = ; NO angular velocity estimation

    	//Note: twist message is specified in frame given by "child_frame_id", i.e. XIMU.
    	odom.twist.twist.linear.x = I_v[0];
    	odom.twist.twist.linear.y = I_v[1];
    	odom.twist.twist.linear.z = I_v[2];

    	return odom;
    }

}



