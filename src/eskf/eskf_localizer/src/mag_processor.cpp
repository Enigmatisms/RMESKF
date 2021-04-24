#include "eskf_localizer/mag_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

//FIXME Only for testing
#include <ros/console.h>


namespace ESKF_Localization{

MagProcessor::MagProcessor(Eigen::Matrix3d V){
	V_ = V;
}

void MagProcessor::Mag_correct(const MagDataPtr MagData, State* state){
	Eigen::Vector3d z = MagData->mag;
	Eigen::Vector3d h_x = state->G_R_I.transpose() * state->m_ref;
	Eigen::Quaterniond q(state->G_R_I);

	Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
	Eigen::Matrix<double,4,3> temp;
	temp << Eigen::Matrix<double,1,3>::Zero(), 0.5*Eigen::Matrix3d::Identity();

	/// 前面的 diff_qT_a_q_diff_q 就是对四元数产生的点旋转进行求导，注意原来hx是q的逆进行的旋转，故不是diff_q_a_qT_diff_q
	/// 这部分实际上是这样的，H=dh/dx * dx/d(\delta x)， 因为这是ESKF 需要对error state 进行求导 
	/// 我觉得这一步写的很对，参见论文 Quaternion kinematics for the error-state KF P34
	H.block<3,3>(0,6) = diff_qT_a_q_diff_q(q,state->m_ref) * quat_l(q) * temp;

	ESKF_correct(z,h_x,H,V_,state);
}
}
