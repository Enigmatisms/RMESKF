#include "eskf_localizer/uwb_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>

const Eigen::Vector3d UnitX = Eigen::Vector3d::UnitX();


namespace ESKF_Localization{

UwbProcessor::UwbProcessor(Eigen::Vector3d I_p_Uwb){
	UwbProcessor::I_p_Uwb_ = I_p_Uwb;
	init_position_set = false;
}

// ===================== UWB 角度以及
void UwbProcessor::Uwb_correct(const UwbPositionDataPtr UwbData, State* state, Eigen::Vector3d& uwb){
	Eigen::Matrix<double, 6, 1> z = Eigen::Matrix<double, 6, 1>::Zero();
	z.block<3, 1>(0, 0) = UwbData->z;

	Eigen::AngleAxisd rv(UwbData->orient, Eigen::Vector3d::UnitZ());
	Eigen::Matrix3d rMat = rv.toRotationMatrix();
	z.block<3, 1>(3, 0) = rMat * UnitX;							// uwb 提供的角度信息（观测）

	// 需要设置一下 I_p_Uwb （IMU与UWB的相对位置关系）
	// state->G_R_I 在开始时就需要利用好UWB信息，将自己正确转过来 初始化的问题
	Eigen::Matrix<double, 6, 1> h_x = Eigen::Matrix<double, 6, 1>::Zero();			// Initial Uwb transformed to Current pos.
	h_x.block<3, 1>(0, 0) = state->G_p_I + state->G_R_I * I_p_Uwb_;
	h_x.block<3, 1>(3, 0) = state->G_R_I.transpose() * UnitX;	// 指向前方的单位向量，自己根据UWB初始角度以及IMU积分得到的状态
	

	Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
	H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	H.block<3,3>(0,6) = -state->G_R_I * hat(I_p_Uwb_);

	Eigen::Matrix<double,4,3> temp;
	temp << Eigen::Matrix<double,1,3>::Zero(), 0.5*Eigen::Matrix3d::Identity();
	Eigen::Quaterniond q(state->G_R_I);
	H.block<3,3>(3,6) = diff_qT_a_q_diff_q(q, UnitX) * quat_l(q) * temp;		

	Eigen::Matrix<double, 6, 6> V;
	Eigen::Matrix<double, 6, 1> _V;
	_V << 0.1, 0.1, 0.05, 0.08, 0.08, 0.01;
	V = _V.asDiagonal();

	printf("State before correction: %f, %f, %f\n", state->G_p_I.x(), state->G_p_I.y(), state->G_p_I.z());
	ESKF_correct(z, h_x, H, V, state);
	printf("State after correction: %f, %f, %f\n", state->G_p_I.x(), state->G_p_I.y(), state->G_p_I.z());
	uwb << z.block<3, 1>(0, 0);
}
}
