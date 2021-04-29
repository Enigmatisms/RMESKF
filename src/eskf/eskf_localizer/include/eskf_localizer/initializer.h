#pragma once

#include "eskf_localizer/base_type.h"
#include <deque>

namespace ESKF_Localization{

class Initializer{

public:
	Initializer(int imu_initialize_goal, int uwb_initialize_goal, int wheel_initialize_goal, State* state);

	void Imu_initialize(ESKF_Localization::ImuDataPtr imu_data);
	void Uwb_initialize(ESKF_Localization::UwbPositionDataPtr uwb_data, State* state);
	void Wheel_initialize(ESKF_Localization::WheelDataPtr wh_data);
	void reset(State* state);

	Eigen::Vector4d getWheelInitialBias() const {
		return this->wh_vel;
	}

	// 获得初始旋转角度, 此角度就是IMU在Global坐标系下如何转动的角度，故直接让其旋转即可
	// 注意旋转的右手定则，所以此处的角度应该是相对与x轴正方向的差（逆时针为正）
	Eigen::Matrix3d getInitialRotation() const {
		Eigen::AngleAxisd rv(uwb_starting_angle, Eigen::Vector3d::UnitZ());
		return rv.toRotationMatrix();
	}

	bool is_initialized(){
		eskf_initialized_ = imu_initialized_ && uwb_initialized_ && wheel_initialized_;
		return eskf_initialized_;
	}
public:
	double accel_x_mean;
	double accel_y_mean;
	double roll_angle;
	double pitch_angle;
	double uwb_starting_angle;
private:

	int imu_initialize_goal_ = 10;
	int uwb_initialize_goal_ = 10;
	int wheel_initialize_goal_ = 10;

	bool got_first_uwb_message_; 	//usually GPS message comes after imu/wheel messages.
							   		//start collecting IMU/Wheels data samples after the first message is received,
							   		//which indicates consecutive GPS data begins.
	bool imu_initialized_;
	bool uwb_initialized_;
	bool wheel_initialized_;
	bool eskf_initialized_;

	std::deque<ImuDataPtr> imu_buffer_;
	std::deque<UwbPositionDataPtr> uwb_buffer_;
	std::deque<WheelDataPtr> wheel_buffer_;

	Eigen::Vector3d uwb_average;
	Eigen::Vector4d wh_vel;			// velocity bias of wheels 
};

}
