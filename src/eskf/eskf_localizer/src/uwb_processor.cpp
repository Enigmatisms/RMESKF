#include "eskf_localizer/uwb_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>


namespace ESKF_Localization{

UwbProcessor::UwbProcessor(Eigen::Vector3d I_p_Uwb){
	UwbProcessor::I_p_Uwb_ = I_p_Uwb;
	init_position_set = false;
}

/// @todo 此处最好还有一个angle的correct
void UwbProcessor::Uwb_correct(const UwbPositionDataPtr UwbData, State* state, Eigen::Vector3d& uwb){
	uwb(0) = UwbData->z(0);
	uwb(1) = UwbData->z(1);
	uwb(2) = UwbData->z(2);

	// Eigen::Vector3d magn = state->G_R_I.transpose() * state->m_ref;	// 磁场当前旋转到的3D方向
	// magn 的投影 可以求到一个方向角（投影）

	// 需要设置一下 I_p_Uwb （IMU与UWB的相对位置关系）
	Eigen::Vector3d h_x = state->G_p_I + state->G_R_I * I_p_Uwb_;		// Initial Uwb transformed to Current pos.

	Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
	H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	H.block<3,3>(0,6) = -state->G_R_I * hat(I_p_Uwb_);

	Eigen::Matrix3d V;
	V << 0.1,0,0,0,0.1,0,0,0,0.05;

	printf("State before correction: %f, %f, %f\n", state->G_p_I.x(), state->G_p_I.y(), state->G_p_I.z());
	ESKF_correct(uwb,h_x,H,V,state);
	printf("State after correction: %f, %f, %f\n", state->G_p_I.x(), state->G_p_I.y(), state->G_p_I.z());
}
}
