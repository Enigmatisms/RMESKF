#include "eskf_localizer/base_type.h"
#include "eskf_localizer/eskf.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/console.h>

// 个人倾向于，电控角度积分是状态量，但是初始角度需要UWB提供的角度

namespace ESKF_Localization{

	void ESKF_update_nominal(ImuDataPtr imu_data,double dt, State* state, Eigen::Vector3d g){
		double dt2_2 = 0.5*dt*dt;
		State old_state = *state;

		state->G_p_I = old_state.G_p_I + dt * old_state.G_v_I + dt2_2 * (old_state.G_R_I * (imu_data->accel - old_state.ab) + g);
		state->G_v_I = old_state.G_v_I + dt * (old_state.G_R_I * (imu_data->accel - old_state.ab) + g);

		ROS_INFO("Updating orientation by quaternion");
		// quaternion valid, update orientation according to delta quaternion
		// 默认电控会传送IMU积分角度
		Eigen::Matrix3d d_rot(imu_data->last_quat.conjugate() * imu_data->quat);
		state->G_R_I = old_state.G_R_I * d_rot;
		const Eigen::Vector3d d_theta = (- old_state.wb) * dt;
		if (d_theta.norm() >= 1e-12){
			Eigen::AngleAxisd d_rot2(d_theta.norm(),d_theta.normalized());
			state->G_R_I *= d_rot2.toRotationMatrix();
		}

		//current quaternion valid, haven't got last quaternion.
		imu_data->last_quat = imu_data->quat;
	}

	void ESKF_predict(Eigen::MatrixXd Fx, Eigen::MatrixXd Q, State* state){
		State old_state = *state;
		state->P = (Fx * old_state.P * Fx.transpose() + Q).eval();
	}

	void ESKF_correct(Eigen::VectorXd z,Eigen::VectorXd h_x, Eigen::MatrixXd H, Eigen::MatrixXd V, State* state){
		State old_state = *state;

		Eigen::MatrixXd K = old_state.P * H.transpose() * (H * old_state.P * H.transpose() + V).inverse();
		Eigen::VectorXd del_x = K * (z - h_x);

		Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
		state->P = (I_KH * old_state.P * I_KH.transpose() + K * V * K.transpose()).eval();

		//inject error state
		state->G_p_I = old_state.G_p_I + del_x.segment<3>(0);
		state->G_v_I = old_state.G_v_I + del_x.segment<3>(3);
		const Eigen::Vector3d del_theta = del_x.segment<3>(6);
		if (del_theta.norm() >= 1e-12){
			state->G_R_I = old_state.G_R_I * Eigen::AngleAxisd(del_theta.norm(),del_theta.normalized()).toRotationMatrix();
		}
		state->ab = old_state.ab + del_x.segment<3>(9);
		state->wb = old_state.wb + del_x.segment<3>(12);
	}
}



