#ifndef REALTIMEFILE_IO_H_
#define REALTIMEFILE_IO_H_

#include <string>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include "variable_conversion.hpp"
#include "imu_store.hpp"

#ifdef useIMU
#include "imu_data_manipulation.hpp"
#endif

/**
* Realtime Input Output Class
* This class is built to eliminate the need of saving a big matrix or array during long runtime
*
*
* Depends on Eigen Library for matrix objects. Very simple to include that into your program
* @see http://eigen.tuxfamily.org/index.php?title=Main_Page
*
* @author Er Jie Kai (EJK)
* @bug No known bugs.
*/
class RealTimeFileIO {
private:

	VariableConversion varc;
	int num_imu = 0;
	int imu_placement = 0; //creates an int value that indicates the imu_placement order
	IMUStore* imu_store;

	/////////////////// Real time IO ////////////////////
	bool write_binary = false;
	std::ofstream realtimefile; //file to be saved in realtime
	std::string realtimefilename; // filename of the current savefile
	std::string append_filename;
	int filecount = 0; // the number of files that has been automatically generated
	int rowcount = 0;// number of rows that have been written
	int extra_col = 0; // number of extra columns to add
	std::string extra_col_header = ""; //header of the extra cols

	void newFileBinary(int datasize);
	void newFileASCII();

	void writeToFileIMUBinary(IMUStore* data, float *extra_data);
	void writeToFileIMU(IMUStore* data, float *extra_data);

	std::string getSaveString();

public:
#ifdef useIMU
	RealTimeFileIO(IMUDataManipulation &imu_manip);
#endif
	RealTimeFileIO(std::string append_filename_ = "");
	void init(bool write_binary, IMUStore* data);
	void save(IMUStore* data, float* extra_data = NULL);
	void newfile(IMUStore* data);

	~RealTimeFileIO() 
	{
		end();
	}
	
	void end();
	void addColumn(std::string header);
	std::string getFileName()
	{
		return realtimefilename;
	}

};

#endif  // REALTIMEFILE_IO_H_
