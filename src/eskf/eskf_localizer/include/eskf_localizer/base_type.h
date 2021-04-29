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
	double timestamp;			// Unit: Seconds
	double orient;				// Unknown
	Eigen::Vector3d z;			// 10Hz observation
};
using UwbPositionDataPtr = std::shared_ptr<UwbPositionData>;

// 轮速
struct WheelData{
	double timestamp;			// Unit: Seconds
	double yaw_angle;			// From encoder: the relative angle between chassis and pan-tilt
	Eigen::Vector4d wheel;		// Unit: m/s
};
using WheelDataPtr = std::shared_ptr<WheelData>;

struct State{
	double timestamp;

	//Nominal states
	Eigen::Vector3d G_p_I;			//Position of IMU frame(I) in global frame(G)
	Eigen::Vector3d G_v_I;			//Velocity of IMU frame(I) in global frame(G)
	// 没讲清楚，这里的G_R_I的意思是：IMU坐标系下的坐标如何转换到世界坐标系下 只需要左乘这个矩阵即可，比如UWB相对位置
	// 所以G——R——I 刚好是IMU在世界坐标系下的旋转情况（世界如何旋转可以得到IMU（让两个坐标系重合））
	Eigen::Matrix3d G_R_I;			//Rotation from IMU frame(I) to global frame(G)
	Eigen::Vector3d ab;				//Accelerometer bias
	Eigen::Vector3d wb;				//Gyroscope bias

	Eigen::Matrix<double,15,15> P;	//Covariance of the error state
};

}
