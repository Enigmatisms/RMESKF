#pragma once

#include "eskf_localizer/base_type.h"
#include <deque>

namespace ESKF_Localization{

class Initializer{

public:
	Initializer(int imu_initialize_goal, int uwb_initialize_goal, int wheel_initialize_goal, State* state);

	void Imu_initialize(ESKF_Localization::ImuDataPtr imu_data);
	void Uwb_initialize(ESKF_Localization::UwbPositionDataPtr uwb_data);
	void Wheel_initialize(ESKF_Localization::WheelDataPtr wh_data);

	Eigen::Vector3d getUWBInitialPos() const {
		return this->uwb_average;
	}

	Eigen::Vector4d getWheelInitialBias() const {
		return this->wh_vel;
	}

	bool is_initialized();
public:
	double accel_x_mean;
	double accel_y_mean;
	double roll_angle;
	double pitch_angle;
private:

	int imu_initialize_goal_ = 10;
	int uwb_initialize_goal_ = 10;
	int wheel_initialize_goal_ = 10;

	bool got_first_uwb_message_ = false;//usually GPS message comes after imu/mag messages.
							   //start collecting IMU/MAG data samples after the first message is received,
							   //which indicates consecutive GPS data begins.

	bool imu_initialized_ = false;
	bool uwb_initialized_ = false;
	bool wheel_initialized_ = false;
	bool eskf_initialized_ = false;

	std::deque<ImuDataPtr> imu_buffer_;
	std::deque<UwbPositionDataPtr> uwb_buffer_;
	std::deque<WheelDataPtr> wheel_buffer_;

	Eigen::Vector3d uwb_average;
	Eigen::Vector4d wh_vel;			// velocity bias of wheels 

	State* state_;
};

}
