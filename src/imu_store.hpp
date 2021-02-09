#ifndef IMU_STORE_HPP_
#define IMU_STORE_HPP_

// other libraries' headers
#include "Eigen/Dense"
#include "Eigen/Geometry"

/** An enum type to indicate possible IMU placements
*/
//enum IMUPosition
//{
//	kTorso = 0,
//	kRThigh = 1,
//	kRShank = 2,
//	kLThigh = 3,
//	kLShank = 4,
//	kNumIMUPos = 5 ///< the total number of IMU available
//};

/** Storage for IMU data
*
* Stores IMU data.\n
* Data transfer between hardware and software is handled by IMUComm.\n
* Data manipulation is done by IMUDataManipulation. \n
* Changing of endian format is handled here.
*
* x = right, y = up, z = backwards(OpenGL)
*
* Created by:
* @author Er Jie Kai (EJK)
 */
struct IMUStore 
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //needed for using Eigen fixed size vector and matrix in struct and class
	Eigen::Quaternionf quat = { 0,0,0,0 };
	Eigen::Quaternionf reference = { 0,0,0,0 };
	Eigen::Quaternionf center = { 0,0,0,0 };
	Eigen::Vector3f accel = { 0,0,0 };
	//Eigen::Vector3f vel= { 0,0,0 }; //linear velocity
	Eigen::Vector3f gyro = { 0,0,0 };

	unsigned char button= 0; // with IMU facing you, 1=left_button, 2=right_button
	unsigned int interval= 0;
	unsigned int timestamp= 0;
	int serial_number= 0;
	int position = -1; //number of possible IMU position (torso, 2 thighs, 2 shanks)

	char rawdata[(11 * 4 + 1)] = { 0 }; // 11 becoz 3 for accel, 3 for gyro, 4 for quat, 1 for timestamp. *4 coz each is float which is 4 bytes. +1 coz there is button state
	int rawdatasize= 0;
};

#endif //IMU_STORE_HPP_