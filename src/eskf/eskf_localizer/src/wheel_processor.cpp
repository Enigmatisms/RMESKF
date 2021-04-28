#include "eskf_localizer/wheel_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/console.h>

namespace ESKF_Localization
{

WheelProcessor::WheelProcessor(Eigen::Matrix3d V)
{
	V_ = V;
}

static Eigen::Vector3d solveChassis(const WheelDataPtr wh_data)
{
	// wh_data中的yaw_angle是谁相对与谁 底盘相对于云台还是相反？
	Eigen::Matrix<double, 4, 3> A = Eigen::Matrix<double, 4, 3>::Zero();
	Eigen::Vector4d B = wh_data->wheel.eval();
	Eigen::Vector3d C = Eigen::Vector3d::Zero();

	A << 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1;
	C = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
	C(2) = 0.0;
	// 此处是云台相对于底盘的转角为yaw_angle，顺重力方向顺时针旋转为正（yaw_angle）
	Eigen::AngleAxisd rv(- wh_data->yaw_angle, Eigen::Vector3d::UnitZ());
	Eigen::Matrix3d rMat = rv.toRotationMatrix();
	C = (rMat * C).eval();
	return C;
}

void WheelProcessor::Wheel_correct(const WheelDataPtr WhData, State *state)
{
	Eigen::Vector3d z = solveChassis(WhData);			// 解出来是纯平移
	Eigen::Vector3d h_x = state->G_v_I;
	Eigen::Quaterniond q(state->G_R_I);

	Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
	H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

	ESKF_correct(z, h_x, H, V_, state);
}
} // namespace ESKF_Localization
