#include "foot_sensor.hpp"


using namespace std;


/*
@brief	Open the serial port to read foot-pressure sensor

User to check and change the serial-port name if used in different PC
In Linux, left-sensor -> /dev/ttyS0 ; right --> /dev/ttyS1
In Windows, left-sensor -> COM13 ; right-sensor -> COM7

@param[in]	serial_port	:	name of the serial_port
@return	void
*/
void FootSensor::OpenSerialPort(USBStream* serial_port)
{
	// Define the serial port number
	std::string* comport = new std::string[2];
#if defined(_WIN32) || defined(_WIN32)
	comport[0] = "13";		// User to define COM port of left foot
	comport[1] = "7";		// User to define COM port of right foot
#elif defined (__unix__)
	comport[0] = "0";		// User to define COM port of left foot_sensor
	comport[1] = "1";		// User to define COM port of right foot_sensor
#endif

    const int USB_num = 2;

	for (int k = 0; k < USB_num; k++)
	{
		// Declare the configuration of serial port
#if defined(_WIN32) || defined(WIN32)
		serial_port_CoP[k].configurePort(115200, 8, 0, 0, 0);
#elif defined(__unix__)
		serial_port[k].configurePort(115200, 8, 0, 1, 0);
#endif
		// Open serial port  &  Set configuration  &  Set wait-comm-event
		serial_port[k].Open(comport[k].c_str());
		// Print out the status of opening serial port
		if (!serial_port[k].good())
		{
			std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
					<< "Error: Could not open serial port: " << comport[k].c_str()
					<< std::endl;
			exit(1);
		}
		else
			std::cout << "Successfully open serial port: " << comport[k].c_str() << std::endl;
			// Set time-outs

#if defined(_WIN32) || defined(WIN32)
		serial_port_CoP[k].setTimeouts(0.005, 1, 0.01, 0.1, 0.1);
#endif
	}
}


/*
@brief	Read the pressure value of each of the 99 pixels of the foot sensors
Log the time-stamps of this sensor reading

1 data-package (from serial port) includes 105x2 bytes (uint16_t).
However foot-sensor has only 99 valid pixels, so certain bytes are NULL.

First code checks the 1st byte as when to start recording subsequent bytes.
Second merge every 2-bytes into 1-value.
Third store the 1D array into 2D Eigen matrix

@param[in]	serial_port	object to handle the serial Communication
@param[out]`pressure_data	struct that contains 2 Eigen matrices [15x7] to store pressure @ pixels
*/
void FootSensor::ReadPressureData(USBStream* serial_port, PressureData* pressure_data)
{
    bool read_success = false;
	unsigned char *data = new unsigned char[210]; // To store 105x <uint16_t> data from STM32

	for (int k = 0; k < 2; k++) // Start reading the serial port 1-by-1
	{
		while (read_success == false)
		{
			// Send command to start the Arduino Communication
			unsigned char serial_command[1];
			serial_command[0] = 255; // any Serial_Command can all trigger the sensor reading
			serial_port[k].write((char *)serial_command, 1);

			// Skip the matching step of the returned Serial_Command, proceed to read (2x)105 bytes of pressure sensor
			if (true) 
			{
				serial_port[k].read((char *)data, 210);

				// Convert the data from   uint16_t >> uint8_t >> int   and store in matrix [15 x 7]
				for (int i = 0; i < 210; i++)
				{
					int cur_row = (i / 2) / 7;
					int cur_col = (i / 2) % 7;
					if (k == 0)
					{
						if ((i % 2) == 1)
						{
							uint16_t temp_holder = (data[i] << 8) | data[i - 1];
							pressure_data->sensor_left(cur_row, cur_col) = int(static_cast<uint16_t>(temp_holder));
						}
					}
					else
					{
						if ((i % 2) == 1)
						{
							uint16_t temp_holder = (data[i] << 8) | data[i - 1];
							pressure_data->sensor_right(cur_row, cur_col) = int(static_cast<uint16_t>(temp_holder));
						}
					}
				}
				read_success = true;
			}
		}

		read_success = false; // to switch to read the second leg
	}

	// Log time-stamps of reading foot sensor data
	time_point_curr = std::chrono::steady_clock::now();
    time_interval = time_point_curr - time_point_prev;
	time_point_prev = std::chrono::steady_clock::now();

	delete[] data;
}


/**/
void FootSensor::CalcPressureGradiant(PressureData* pressure_data)
{
	// Calculate pressure gradiant from the RAW sensor reading
    pressure_data->right_pressure_grad = (pressure_data->right_pressure - pressure_data->right_pressure_prev) / time_interval.count();
    pressure_data->left_pressure_grad = (pressure_data->left_pressure - pressure_data->left_pressure_prev) / time_interval.count();

	// Save the current reading as previous data, after calculating
	pressure_data->right_pressure_prev = pressure_data->right_pressure;
	pressure_data->left_pressure_prev = pressure_data->left_pressure;
}


/**/
void FootSensor::CalcPressureAverGrad(PressureData* pressure_data)
{
	// Get the current time_point to calculate time_interval
    time_point_curr = std::chrono::steady_clock::now();
    time_interval = time_point_curr - time_point_prev;

	// Calculate pressure gradiant from the AVERAGE sensor reading
	pressure_data->right_pressure_aver_grad = (pressure_data->right_pressure_average - pressure_data->right_pressure_aver_prev) / time_interval.count();
	pressure_data->left_pressure_aver_grad = (pressure_data->left_pressure_average - pressure_data->left_pressure_aver_prev) / time_interval.count();

	// Save previous data with the RAW average_pressure
	pressure_data->right_pressure_aver_prev = pressure_data->right_pressure_average;
	pressure_data->left_pressure_aver_prev = pressure_data->left_pressure_average;
}


/*
@brief	Calculate the CoP of a single sensor, either left or right
This is an internal function, not used in main().

@param[in]	pressure_mat	the matrix that contains 99 pixel-pressure
@param[out]	cop_x			the x-coordinate of the COP
@param[out]	cop_y			the y-coordinate of the COP

@return	nothing
*/
void FootSensor::CalcCOP_SingleSensor(Eigen::MatrixXi *pressure_mat, float *cop_x, float *cop_y)
{
	float pressure_sum = pressure_mat->sum();

	Eigen::MatrixXi multiplier_colY(7, 1);
	multiplier_colY << 1, 2, 3, 4, 5, 6, 7;

	if (pressure_sum == 0)
	{
		*cop_y = 0;
	}
	else
	{
		*cop_y = ((*pressure_mat) * (multiplier_colY)).sum() / pressure_sum;
	}

	Eigen::MatrixXi multiplier_rowX(15, 1);
	multiplier_rowX << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
	if (pressure_sum == 0)
	{
		*cop_x = 0;
	}
	else
	{
		*cop_x = ((pressure_mat->transpose()) * (multiplier_rowX)).sum() / pressure_sum;
	}
}


/*
@brief	Call the calculating-COP method for each foot-sensor, then save COP results

@param[in/out]	pressure_data	struct that contains pressure-pixels as input and COP-x/y as output

@return nothing
*/
void FootSensor::CalcCOP(PressureData* pressure_data)
{
	pressure_data->right_pressure = pressure_data->sensor_right.sum();
	pressure_data->left_pressure = pressure_data->sensor_left.sum();

	CalcCOP_SingleSensor(&(pressure_data->sensor_right), &(pressure_data->right_cop_x), &(pressure_data->right_cop_y));
	CalcCOP_SingleSensor(&(pressure_data->sensor_left), &(pressure_data->left_cop_x), &(pressure_data->left_cop_y));
}




/*
@brief	Store the first foot-sensor reading, in order to filter the spike later.
Read sensor once, before the while-loop.
Calculate the first pressure-sum as reference.

@param[in]	serial_port		object of serial_comm to read the pressure each pixel
@param[out]	pressure_data	struct that store the pressure-sum
*/
void FootSensor::FilterSpike_Init(USBStream* serial_port, PressureData* pressure_data)
{
	pressure_data->sensor_right.setZero(pressure_data->n_row, pressure_data->n_col);
	pressure_data->sensor_left.setZero(pressure_data->n_row, pressure_data->n_col);

    ReadPressureData(serial_port, pressure_data);
	
    pressure_data->right_pressure_prev = pressure_data->sensor_right.sum();
    pressure_data->left_pressure_prev = pressure_data->sensor_left.sum();

    time_point_prev = std::chrono::steady_clock::now();
}


/*
@brief Filter the spike (if any) when reading the raw data from foot-sensor.
When reading sensor, if the pressure-sum surges higher than threshold, 
then discard the current reading and store the previous reading as current.

If filter is triggered, the next reading will not filter. 
The 2nd-next reading will resume filtering again.

@param[in/out]	pressure_data	struct that contains 99-pixel n pressure-sum.
@param[in/out]	spike_check		flag that is true when SpikeFilter is triggered.
*/
void FootSensor::FilterSpike(PressureData* pressure_data, bool* spike_check)
{
    // Refer the pressure-threshold from the header-file

    // Calculate the pressure_threshold
    CalcPressureGradiant(pressure_data);

	// Check spikes in right foot_sensor
    if((pressure_data->right_pressure_grad > threshold) && (spike_check[0]==true))
    {
        pressure_data->right_pressure = pressure_data->right_pressure_prev;
        spike_check[0] = false;
    }
    else 
        spike_check[0] = true;
    
	// Check spikes in left foot_sensor
    if((pressure_data->left_pressure_grad > threshold) && (spike_check[1] == true))
    {
        pressure_data->left_pressure = pressure_data->left_pressure_prev;
        spike_check[1] = false;
    }
    else
        spike_check[1] = true;
}


/*
@brief	Determine when the heel-strike based if pressure-gradiant surges higher than a threshold.
Thresholds of left & right foot-sensor are different.

getHeelStrike is only triggered during Swing phase.

@param[in]	pressure_data	struct that contains the pressure-sum & COP
@param[in]	heel_check		

@return 	0->no heelstrike ; 1->left heelstrike ; 2->right heelstrike
*/
int FootSensor::getHeelStrike(PressureData* pressure_data, int* heel_check)
{
	// Calculate the AVERAGE pressure_gradiant
	CalcPressureAverGrad(pressure_data);

	if( heel_check[0] > 0 )
	{
		// 2nd-stage checking if the pressure_gradiant is greater than threshold
		if ( heel_check[1] == 2 && pressure_data->right_pressure_aver_grad > right_pressure_threshold )
		{
			heel_check[0] = 0;
			heel_check[1] = 0;
			return 2;	// same as step_status ; heel_strike == 2 -> right step
		}
		else if ( heel_check[1] == 1 && pressure_data->left_pressure_aver_grad > left_pressure_threshold )
		{
			heel_check[0] = 0;
			heel_check[1] = 0;
			return 1;	// same step_status ; heel_strike == 1 -> left step
		}

		// 1st-stage checking if the pressure_gradiant is greater than threshold
		if ( heel_check[0] == 2 && pressure_data->right_pressure_aver_grad > right_pressure_threshold )
		{
			heel_check[1] = 2;
		}
		else if ( heel_check[0] == 1 && pressure_data->left_pressure_aver_grad > left_pressure_threshold )
		{
			heel_check[1] = 1;
		}
	}
	else return 0;

	return 0;
}


