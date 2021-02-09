#ifndef FOOT_SENSOR_HPP
#define FOOT_SENSOR_HPP

#include <iostream>
#include <chrono>

#include "Eigen/Dense"
#include "Eigen/Geometry"


#include "serial_stream.hpp"
#include "matrix_io.hpp"


using namespace std;


struct PressureData
{
	const int n_row = 15;
	const int n_col = 7;
	const int sensor_size = n_row * n_col;

	Eigen::MatrixXi sensor_left;
	Eigen::MatrixXi sensor_right;

	float right_cop_x = 0;
	float left_cop_x = 0;
	float right_cop_y = 0;
	float left_cop_y = 0;
	float right_pressure = 0;
	float left_pressure = 0;

    float right_cop_x_average = 0;
	float left_cop_x_average = 0;
	float right_cop_y_average = 0;
	float left_cop_y_average = 0;
	float right_pressure_average = 0;
	float left_pressure_average = 0;

	float right_cop_x_standard = 0;
	float left_cop_x_standard = 0;
	float right_cop_y_standard = 0;
	float left_cop_y_standard = 0;
	float right_pressure_standard = 0;
	float left_pressure_standard = 0;

	float right_cop_x_scale = 0;
	float left_cop_x_scale = 0;
	float right_cop_y_scale = 0;
	float left_cop_y_scale = 0;
	float right_pressure_scale = 0;
	float left_pressure_scale = 0;

	// Pressure gradiant to be used for FilterSpike() and StepFinish()
	float right_pressure_grad = 0;
	float left_pressure_grad = 0;
	float right_pressure_prev = 0;
	float left_pressure_prev = 0;

	// Pressure_average gradiant to be used for Heel_Strike & StepFinish()
	float right_pressure_aver_grad = 0;
	float left_pressure_aver_grad = 0;
	float right_pressure_aver_prev = 0;
	float left_pressure_aver_prev = 0;
};



class FootSensor
{
public:
	void OpenSerialPort(USBStream* serial_port);

	void ReadPressureData(USBStream* serial_port, PressureData* pressure_data);

	void CalcPressureGradiant(PressureData* pressure_data);

	void CalcPressureAverGrad(PressureData* pressure_data);

	void CalcCOP(PressureData* pressure_data);

    void FilterSpike_Init(USBStream* serial_port, PressureData* pressure_data);

    void FilterSpike(PressureData* pressure_data, bool* spike_check);

	int getHeelStrike_Init(int step_status, int* heel_check);

	int getHeelStrike(PressureData* pressure_data, int* heel_check);


private:
	// time_interval is used for calculating pressure gradiants
	std::chrono::time_point<std::chrono::steady_clock> time_point_prev;
	std::chrono::time_point<std::chrono::steady_clock> time_point_curr;
	std::chrono::duration<float, std::ratio<1, 1>> time_interval;	// in seconds

	// Threshold to check for heel strike
	const float right_pressure_threshold = 30000;
	const float left_pressure_threshold = 20000;

	// Threshold to filter the spike when reading foot-sensor
	const int threshold = 10000000;

	void CalcCOP_SingleSensor(Eigen::MatrixXi *pressure_mat, float *CoP_x, float *CoP_y);

};


#endif // FOOT_SENSOR_HPP