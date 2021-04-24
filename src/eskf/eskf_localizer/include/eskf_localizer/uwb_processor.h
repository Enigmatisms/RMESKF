#pragma once

#include "eskf_localizer/base_type.h"
#include <Eigen/Core>

namespace ESKF_Localization{

class UwbProcessor{

public:
	UwbProcessor(Eigen::Vector3d I_p_Uwb);
	void Uwb_correct(const UwbPositionDataPtr UwbData, State* state, Eigen::Vector3d& uwb);
	void setInitPos(Eigen::Vector3d pos) {
		init_position_set = true;
		I_p_Uwb_ = pos;
	}

	bool isInitialSet() const {
		return this->init_position_set;
	}

	void setInitPosFlag(bool flag = true) {
		init_position_set = flag;
	}
private:
	Eigen::Vector3d I_p_Uwb_;		//position of Uwb in IMU frame
	bool init_position_set;
};

}
