#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ESKF_Localization{

struct ImuData{
	double timestamp;			//Unit: Seconds

	Eigen::Vector3d accel;		//Unit: m/s^2
	Eigen::Vector3d gyro;		//Unit: rad/s

	Eigen::Quaterniond quat;	//Orientation estimation from IMU. For example, DMP.
	Eigen::Quaterniond last_quat;
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct UwbPositionData{
	double timestamp;			//Unit: Seconds
	double orient;				// Unknown
	Eigen::Vector3d z;			// 10Hz observation
};
using UwbPositionDataPtr = std::shared_ptr<UwbPositionData>;

struct MagData{
	double timestamp;			//Unit: Seconds

	Eigen::Vector3d mag;		//Unit: uT
};
using MagDataPtr = std::shared_ptr<MagData>;

struct State{
	double timestamp;

	//Nominal states
	Eigen::Vector3d G_p_I;			//Position of IMU frame(I) in global frame(G)
	Eigen::Vector3d G_v_I;			//Velocity of IMU frame(I) in global frame(G)
	Eigen::Matrix3d G_R_I;			//Rotation from IMU frame(I) to global frame(G)
	Eigen::Vector3d ab;				//Accelerometer bias
	Eigen::Vector3d wb;				//Gyroscope bias

	Eigen::Vector3d m_ref;
	//magnetic field reference

	Eigen::Matrix<double,15,15> P;	//Covariance of the error state
};

}
