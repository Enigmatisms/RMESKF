#pragma once

#include "eskf_localizer/base_type.h"
#include <Eigen/Core>

namespace ESKF_Localization{

class WheelProcessor{

public:
	WheelProcessor(Eigen::Matrix3d V);
	void Wheel_correct(const WheelDataPtr WhData, State* state);
private:
	Eigen::Matrix3d V_;				//covariance
};
}
