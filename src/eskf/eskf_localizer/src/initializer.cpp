#include "eskf_localizer/initializer.h"
#include "eskf_localizer/base_type.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

//FIXME
#include <ros/console.h>

namespace ESKF_Localization{

Initializer::Initializer(int imu_initialize_goal, int uwb_initialize_goal,int wheel_initialize_goal, State* state){
	imu_initialize_goal_ = imu_initialize_goal;
	uwb_initialize_goal_ = uwb_initialize_goal;
	wheel_initialize_goal_ = wheel_initialize_goal;
	state_ = state;
	uwb_average.setZero();

	//initial value of P
	state->P.block<3,3>(0,0) = 10.0 * Eigen::Matrix3d::Identity();		//position cov
	state->P.block<3,3>(3,3) = 5.0 * Eigen::Matrix3d::Identity();		//velocity cov
	state->P.block<3,3>(6,6) = 0.04 * Eigen::Matrix3d::Identity();		//roll-pitch-yaw cov
	state->P.block<3,3>(9,9) = 0.0004 * Eigen::Matrix3d::Identity();	//acceleration bias cov
	state->P.block<3,3>(12,12) = 0.0004 * Eigen::Matrix3d::Identity();	//gyroscope bias cov

	accel_x_mean = 0.0;
	accel_y_mean = 0.0;
	pitch_angle = 0.0;
	roll_angle = 0.0;
}

void Initializer::Imu_initialize(ESKF_Localization::ImuDataPtr imu_data){
	if(!got_first_uwb_message_){
		return;
	}

	//if (uwb_initialized_){return;}magnetic is slower than imu, constantly update buffer

	imu_buffer_.push_back(imu_data);
	if (imu_buffer_.size() > imu_initialize_goal_){
		imu_buffer_.pop_front();
		imu_initialized_ = true;
		Eigen::Vector3d imu_mean = Eigen::Vector3d::Zero();
		for (const ImuDataPtr& ptr: imu_buffer_) {
			imu_mean += ptr->accel;
		}
		imu_mean /= imu_initialize_goal_;
		accel_x_mean = imu_mean.x();
		accel_y_mean = imu_mean.y();
		pitch_angle = std::asin(std::abs(accel_x_mean) / 9.81);
		roll_angle = std::asin(std::abs(accel_y_mean) / 9.81);
	}
}

void Initializer::Uwb_initialize(ESKF_Localization::UwbPositionDataPtr uwb_data){
	got_first_uwb_message_ = true;

	if (uwb_initialized_){return;}

	if (uwb_buffer_.size() < uwb_initialize_goal_){
		uwb_buffer_.push_back(uwb_data);
	}else{
		//compute lla_origin
		uwb_average.setZero();
		double uwb_orient_mean = 0.0;
		for (ESKF_Localization::UwbPositionDataPtr uwb_d : uwb_buffer_){
			uwb_average += uwb_d->z;
			uwb_orient_mean += uwb_d->orient;
		}
		uwb_average /= uwb_initialize_goal_;
		uwb_initialized_ = true;
		ROS_INFO("[Initializer] Uwb initialized with %f, %f, %f", uwb_average.x(), uwb_average.y(), uwb_average.z());
	}
}

void Initializer::Wheel_initialize(ESKF_Localization::WheelDataPtr wh_data){
	if (!got_first_uwb_message_ || wheel_initialized_){
		return;
	}
	if (wheel_buffer_.size() < wheel_initialize_goal_){
		wheel_buffer_.push_back(wh_data);
	}
	if (wheel_buffer_.size() == wheel_initialize_goal_){
		Eigen::Vector4d wh_init = Eigen::Vector4d::Zero();

		for (ESKF_Localization::WheelDataPtr wh_d : wheel_buffer_){
			wh_init += wh_d->wheel;
		}
		wh_init /= wheel_buffer_.size();

		wheel_initialized_ = true;
	}
}

bool Initializer::is_initialized(){
	eskf_initialized_ = imu_initialized_ && uwb_initialized_ && wheel_initialized_;
	return eskf_initialized_;
}
}
