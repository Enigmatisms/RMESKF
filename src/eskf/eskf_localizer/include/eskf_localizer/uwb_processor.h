#pragma once

#include "eskf_localizer/base_type.h"
#include <Eigen/Core>

namespace ESKF_Localization{

class UwbProcessor{

public:
	UwbProcessor(Eigen::Vector3d I_p_Uwb);
	void Uwb_correct(const UwbPositionDataPtr UwbData, State* state, Eigen::Vector3d& uwb);
private:
	Eigen::Vector3d I_p_Uwb_;		//position of Uwb in IMU frame
};

}
