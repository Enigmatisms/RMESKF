#pragma once

#include "eskf_localizer/base_type.h"
#include <deque>

namespace ESKF_Localization{

class Initializer{

public:
	Initializer(int imu_initialize_goal, int uwb_initialize_goal, int mag_initialize_goal, State* state);

	void Imu_initialize(ESKF_Localization::ImuDataPtr imu_data);
	void Uwb_initialize(ESKF_Localization::UwbPositionDataPtr uwb_data);
	void Mag_initialize(ESKF_Localization::MagDataPtr mag_data);
	
	Eigen::Quaterniond getMagInitDir() const {
		return this->mag_init_dir_;
	}

	bool is_initialized();
private:

	int imu_initialize_goal_ = 10;
	int uwb_initialize_goal_ = 10;
	int mag_initialize_goal_ = 10;

	bool got_first_uwb_message_ = false;//usually GPS message comes after imu/mag messages.
							   //start collecting IMU/MAG data samples after the first message is received,
							   //which indicates consecutive GPS data begins.

	bool imu_initialized_ = false;
	bool uwb_initialized_ = false;
	bool mag_initialized_ = false;
	bool eskf_initialized_ = false;

	std::deque<ImuDataPtr> imu_buffer_;
	std::deque<UwbPositionDataPtr> uwb_buffer_;
	std::deque<MagDataPtr> mag_buffer_;

	Eigen::Quaterniond mag_init_dir_;		// 在标定 / 初始化之后得到的初始磁偏方向

	State* state_;
};

}
