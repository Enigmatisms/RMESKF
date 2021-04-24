#pragma once

#include "eskf_localizer/base_type.h"
#include "eskf_localizer/initializer.h"
#include "eskf_localizer/imu_processor.h"
#include "eskf_localizer/uwb_processor.h"
#include "eskf_localizer/mag_processor.h"

#include <fstream>
#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

namespace ESKF_Localization{

class ESKF_Localizer{

public:
	ESKF_Localizer(const double am_noise, const double wm_noise,
			const double ab_noise, const double wb_noise, Eigen::Vector3d I_p_Uwb);

	~ESKF_Localizer() {
		file.close();
	}

	void processImuData(ImuDataPtr imu_data);
	void processUwbData(UwbPositionDataPtr gps_data);
	void processMagData(MagDataPtr mag_data);

	State* getState();
	Eigen::Vector3d getUWBPos() const;
	geometry_msgs::Pose getFusedPose();
	nav_msgs::Odometry getFusedOdometry();

	Eigen::Quaterniond getMagInitDir() const {
		return initializer_->getMagInitDir();
	}
private:

	std::unique_ptr<Initializer> initializer_;
	std::unique_ptr<ImuProcessor> imu_processor_;
	std::unique_ptr<UwbProcessor> uwb_processor_;
	std::unique_ptr<MagProcessor> mag_processor_;
	std::ofstream file;

	Eigen::Vector3d uwb_pos;
	State state_;

	double last_t_;
	double start_time;
	bool start_t_set;
};
}
