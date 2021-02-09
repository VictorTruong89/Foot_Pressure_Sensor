#include <string>
#include <istream>
#include <stdexcept>
#include <ctime> // for getting time date

#include "realtimefile_io.hpp"
#include "variable_conversion.hpp"


RealTimeFileIO::RealTimeFileIO(std::string append_filename_)
{
	this->num_imu = 0;
	this->imu_placement = 0;
	this->append_filename = append_filename_;
}

#ifdef useIMU
RealTimeFileIO::RealTimeFileIO(IMUDataManipulation& imu_manip)
{
	this->num_imu = imu_manip.getNumIMU();
	imu_store = imu_manip.getIMUStore();
}
#endif

/** @brief Add a column to the saved file
 *
 * @note extra column data must also be in float
 * @note must be run before init(), save() or newfile()
 *
 * @return returns nothing
 */
void RealTimeFileIO::addColumn(std::string header)
{
	extra_col++;
	if (extra_col == 1)
		extra_col_header = extra_col_header + header;
	else
		extra_col_header = extra_col_header + ", " + header;
}

/** @brief Open a new binary file and write the standard headers
 *
 * The binary file must be stored with the following format: \n
 * the 1st row is always the number of IMU used to form the data \n
 * the 2nd row is the datasize in bytes per IMU \n
 * All subsequent rows are raw data, with each IMU data separated by a space \n
 * The last row stores the total rows in the binary files (i.e. how many samples) \n
 * New timestamp is separated by new line (i.e. new row)
 *
 * Placeholder for new row = "\n\r"
 *
 * The input matrix must follow a certain format:
 * Each row contains one sample of all the IMU.
 * Col 1 = timestamp
 * Col 2 = Accel_x
 * Col 3 = Accel_y
 * Col 4 = Accel_z
 * Col 5 = Gyro_x
 * Col 6 = Gyro_y
 * Col 7 = Gyro_z
 * Col 8 = Quat_w
 * Col 9 = Quat_x
 * Col 10 = Quat_y
 * Col 11 = Quat_z
 *
 * @param[in] datasize the datasize of the data to be stored in the header [default=0]
 *
 * @return returns nothing
 */
void RealTimeFileIO::newFileBinary(int datasize)
{
	if (realtimefile.is_open() == true)
	{
		char temp[4];
		memcpy(temp, (char*)&rowcount, sizeof(unsigned int)); // store the row count as the last value in little endian format
		realtimefile.write(temp, 4);
		//delete[] realtimefilename;
		//realtimefilename = new char[30];
		realtimefile.close();
		rowcount = 0;
		filecount++;
		rowcount = 0;
		//sprintf(realtimefilename, "../data/data_%d.bin", filecount);
		realtimefilename = getSaveString();
		realtimefilename = realtimefilename + append_filename;
		printf("New file: %s\n", realtimefilename.c_str());
		realtimefile.open("../data/" + realtimefilename + ".bin", std::ios::out | std::ios::trunc | std::ios::binary);
	}
	else if (realtimefile.is_open() == false)
	{
		//realtimefilename = new char[30];
		filecount = 1;
		rowcount = 0;
		//sprintf(realtimefilename, "../data/data%d.bin", filecount);
		realtimefilename = getSaveString();
		realtimefilename = realtimefilename + append_filename;
		printf("New file: %s\n", realtimefilename.c_str());
		realtimefile.open("../data/" + realtimefilename + ".bin", std::ios::out | std::ios::trunc | std::ios::binary);
	}

	// write the standard header (num_imu and datasize)
	char temp[1];
	temp[0] = num_imu;
	realtimefile.write(temp, 1);
	realtimefile.write("\n\r", 2); //print interval seperator
	temp[0] = datasize; //datasize per IMU
	realtimefile.write(temp, 1);
	realtimefile.write("\n\r", 2); //print interval seperator
	temp[0] = extra_col; //number of extra columns
	realtimefile.write(temp, 1);
	realtimefile.write("\n\r", 2); //print interval seperator

	realtimefile.write((char*)&imu_placement, 4); // print imu_placement
	realtimefile.write("\n\r", 2); //print interval seperator
}


/** @brief write the IMU data to a binary file in realtime
 *
 * The binary file must be stored with the following format: \n
 * the 1st row is always the number of IMU used to form the data \n
 * the 2nd row is the datasize in bytes per IMU \n
 * All subsequent rows are raw data, with each IMU data separated by a space \n
 * The last row stores the total rows in the binary files (i.e. how many samples) \n
 * New timestamp is separated by new line (i.e. new row)
 *
 * Placeholder for new row = "\n\r"
 *
 * The input matrix must follow a certain format:
 * Each row contains one sample of all the IMU. Each row is separated by a newline ("\n\r")
 * Each col is separated by a space (" ")
 * Col 1 = timestamp
 * Col 2 = Accel_x
 * Col 3 = Accel_y
 * Col 4 = Accel_z
 * Col 5 = Gyro_x
 * Col 6 = Gyro_y
 * Col 7 = Gyro_z
 * Col 8 = Quat_w
 * Col 9 = Quat_x
 * Col 10 = Quat_y
 * Col 11 = Quat_z
 * Col 12 = extra_data_1
 * Col 13 = extra_data_2
 * ...
 * Col n = extra_data_(n-11)
 *
 * @param[in] data the IMU data to be stored in binary
 * @param[in] extra_data the data to be stored as extra columns (extra columns must be added with addColumn() before this function will store the value. input data must be float)
 *
 * @return returns nothing
 */
void RealTimeFileIO::writeToFileIMUBinary(IMUStore* data, float *extra_data)
{
	if (realtimefile.is_open() == false)
	{
		printf("Failed to open %s (maybe missing data folder). Reopening File.\n", realtimefilename.c_str());

		const int numdata = 11; // 3 for accel, 3 for gyro, 4 for quat, 1 for timestamp
		int datasize = numdata * 4 + 1; // add 1 for button state, add multiples of 4 for extra_columns
		newFileBinary(datasize);
	}

	// write IMU data
	rowcount++;
	for (int k = 0; k < num_imu; k++)
	{
		realtimefile.write(data[k].rawdata, data[k].rawdatasize);
		realtimefile.write(" ", 1);
	}
	for (int k = 0; k < extra_col; k++)
	{
		char temp[4];
		varc.Float2Char(extra_data[k], temp);
		realtimefile.write(temp, sizeof(float));
		realtimefile.write(" ", 1);
	}
	realtimefile.write("\n\r", 2);
	
}

/** @brief Open a new text file and write the standard headers
 *
 * The text file must be stored with the following format: \n
 * First few rows are comments, which always start with a hex ("#") \n
 * Subsequent rows are data for each timestamp. Each IMU and each of their variables are all separated by a space. \n
 * New row for every timestamp
 *
 * Placeholder for new row = "\n\r"
 *
 * The input matrix must follow a certain format:
 * Each row contains one sample of all the IMU.
 * Col 1 = timestamp
 * Col 2 = Accel_x
 * Col 3 = Accel_y
 * Col 4 = Accel_z
 * Col 5 = Gyro_x
 * Col 6 = Gyro_y
 * Col 7 = Gyro_z
 * Col 8 = Quat_w
 * Col 9 = Quat_x
 * Col 10 = Quat_y
 * Col 11 = Quat_z
 * Col 12 = extra_data_1
 * Col 13 = extra_data_2
 * ...
 * Col n = extra_data_(n-11)
 *
 *
 * @return returns nothing
 */
void RealTimeFileIO::newFileASCII()
{
	if (realtimefile.is_open() == true)
	{
		char temp[200];
		int num_col_per_imu = 11;
		int n = sprintf(temp, "#size(row,tot_col,num_imu,num_col_per_imu,imu_placement): %d %d %d %d %d\n", rowcount, num_col_per_imu * num_imu + extra_col, num_imu, num_col_per_imu, imu_placement);
		realtimefile.write(temp, n);
		//delete[] realtimefilename;
		//realtimefilename = new char[20];
		realtimefile.close();
		rowcount = 0;
		filecount++;
		//sprintf(realtimefilename, "../data/data%d.txt", filecount);
		realtimefilename = getSaveString();
		realtimefilename = realtimefilename + append_filename;
		printf("New file: %s\n", realtimefilename.c_str());
		realtimefile.open("../data/" + realtimefilename + ".txt", std::ios::out | std::ios::trunc);
		realtimefile.precision(8);
		realtimefile << std::scientific;
	}
	else if (realtimefile.is_open() == false) //new file count
	{
		//realtimefilename = new char[20];
		filecount = 1;
		rowcount = 0;
		//sprintf(realtimefilename, "../data/data%d.txt", filecount);
		realtimefilename = getSaveString();
		realtimefilename = realtimefilename + append_filename;
		printf("New file: %s\n", realtimefilename.c_str());
		realtimefile.open("../data/" + realtimefilename + ".txt", std::ios::out | std::ios::trunc);
		realtimefile.precision(8);
		realtimefile << std::scientific;
	}

	// write the standard header
	std::string col_header = "";
	for (int i = 0; i < num_imu; i++)
	{
		std::string num = std::to_string(imu_store[i].position);
		col_header += "timestamp" + num + " accelx" + num + " accely" + num + " accelz" + num + " gyrox" + num + " gyroy" + num + " gyroz" + num + " quatw" + num + " quatx" + num + " quaty" + num + " quatz" + num + " ";
	}
	char temp[10000];
	int n = sprintf(temp, "#imu_placement = ");
	realtimefile.write(temp, n);
	realtimefile << imu_placement;
	realtimefile.write("\n", 1);
	n = sprintf(temp, "# %d IMU stored in text file.\n#Every timestamp is stored in a different row.\n#Every variable of the IMU is separated by a space, and a space is also placed between every IMU on the same row for the same timestamp\n#%s%s\n", num_imu, col_header.c_str(), extra_col_header.c_str());
	realtimefile.write(temp, n);
}

/** @brief Open a new text file and write the IMU data in realtime
 *
 * The text file must be stored with the following format: \n
 * First few rows are comments, which always start with a hex ("#") \n
 * Subsequent rows are data for each timestamp. Each IMU and each of their variables are all separated by a space. \n
 * New row for every timestamp
 *
 * Placeholder for new row = "\n\r"
 *
 * The input matrix must follow a certain format:
 * Each row contains one sample of all the IMU.
 * Col 1 = timestamp
 * Col 2 = Accel_x
 * Col 3 = Accel_y
 * Col 4 = Accel_z
 * Col 5 = Gyro_x
 * Col 6 = Gyro_y
 * Col 7 = Gyro_z
 * Col 8 = Quat_w
 * Col 9 = Quat_x
 * Col 10 = Quat_y
 * Col 11 = Quat_z
 *
 * @param[in] data the IMU data to be stored in ascii
 * @param[in] extra_data the data to be stored as extra columns (extra columns must be added with addColumn() before this function will store the value. input data must be float)
 *
 * @return returns nothing
 */
void RealTimeFileIO::writeToFileIMU(IMUStore* data, float *extra_data)
{
	if (realtimefile.is_open() == false)
	{
		printf("Failed to open %s (maybe missing data folder). Reopening File.\n", realtimefilename.c_str());
		newFileASCII();
	}

	// write IMU data
	rowcount++;
	for (int k = 0; k < num_imu; k++)
	{
		realtimefile << data[k].interval << " ";
		realtimefile << data[k].accel(0) << " " << data[k].accel(1) << " " << data[k].accel(2) << " ";
		realtimefile << data[k].gyro(0) << " " << data[k].gyro(1) << " " << data[k].gyro(2) << " ";
		realtimefile << data[k].quat.w() << " " << data[k].quat.x() << " " << data[k].quat.y() << " " << data[k].quat.z() << " ";
		//data[k].printIMUData();
	}
	for (int k = 0; k < extra_col; k++)
	{
		realtimefile << extra_data[k] << " ";
	}
	realtimefile.write("\n", 1);
}

/** @brief Initialize the file
 *
 * Open a new file OR close the previous file, iterate the filecount and open a new file \n
 *
 * @param[in] write_binary state whether to write the data in binary or text
 * @param[in] data the IMU data address to obtain the placement order of the IMU
 *
 * @return returns nothing
 */
void RealTimeFileIO::init(bool write_binary, IMUStore* data)
{
	imu_placement = 0;
	for (int i = 0; i < num_imu; i++)
		imu_placement += data[i].position * pow(10, num_imu - 1 - i);//creates an int value that indicates the imu_placement order

	this->write_binary = write_binary;
	if (write_binary)
	{
		int datasize = 0;
		if (num_imu > 0)
		{
			const int numdata = 11; // 3 for accel, 3 for gyro, 4 for quat, 1 for timestamp
			datasize = numdata * 4 + 1; // add 1 for button state
		}
		newFileBinary(datasize);
	}
	else
	{
		newFileASCII();
	}
}

/** @brief save the IMU data
 *
 * Save the IMU data in binary or text, depending on the initialised "write_binary" variable
 *
 * @param[in] data the IMU data to be saved
 * @param[in] extra_data the extra data to be saved. (must use addColumn before the data can be saved)
 *
 * @return returns nothing
 */
void RealTimeFileIO::save(IMUStore* data, float *extra_data)
{
	if(write_binary)
		writeToFileIMUBinary(data, extra_data);
	else
		writeToFileIMU(data, extra_data);
}

/** @brief start a new file to write the next data
 *
 * Open a new file OR close the previous file, iterate the filecount and open a new file \n
 *
 * @return returns nothing
 */
void RealTimeFileIO::newfile(IMUStore* data)
{
	init(write_binary, data);
}

/** @brief Close all files
 *
 * Close the text or binary files
 *
 * @return returns nothing
 */
void RealTimeFileIO::end()
{
	if (realtimefile.is_open() == true)
	{
		if (write_binary == true)
		{
			char temp[4];
			memcpy(temp, (char*)&rowcount, sizeof(unsigned int)); // store the row count as the last value in little endian format
			realtimefile.write(temp, 4);
		}
		else
		{
			char temp[200];
			int num_col_per_imu = 11;
			int n = sprintf(temp, "#size(row,tot_col,num_imu,num_col_per_imu,imu_placement): %d %d %d %d %d\n", rowcount, num_col_per_imu * num_imu + extra_col, num_imu, num_col_per_imu, imu_placement);
			realtimefile.write(temp, n);
		}
		realtimefile.close();
		//delete[] realtimefilename;
	}
	else
	{
		printf("No file is opened or file already closed\n");
	}
}

std::string RealTimeFileIO::getSaveString()
{
	time_t now = time(0);
	tm* ltm = localtime(&now);
	std::string year = std::to_string(1900 + ltm->tm_year);
	std::string month = std::to_string(1 + ltm->tm_mon);
	std::string day = std::to_string(ltm->tm_mday);
	std::string hour = std::to_string(1 + ltm->tm_hour);
	std::string min = std::to_string(1 + ltm->tm_min);
	std::string sec = std::to_string(1 + ltm->tm_sec);

	//// print various components of tm structure.
	//cout << "Year:" << 1900 + ltm->tm_year << endl;
	//cout << "Month: " << 1 + ltm->tm_mon << endl;
	//cout << "Day: " << ltm->tm_mday << endl;
	//cout << "Time: " << 1 + ltm->tm_hour << ":";
	//cout << 1 + ltm->tm_min << ":";
	//cout << 1 + ltm->tm_sec << endl;

	std::string date_time_string = year + month + day + "_" + hour + min + sec;
	return date_time_string;
}


/*
void example() //without imu save and read
{
	RealTimeFileIO rtfile; //save binary
	RealTimeFileIO rtfile2; //save text
	rtfile.addColumn("data1");
	rtfile.addColumn("cumulative");
	rtfile2.addColumn("data1");
	rtfile2.addColumn("cumulative");

	bool use_binary = true;
	rtfile.init(use_binary, NULL);
	use_binary = false;
	rtfile2.init(use_binary, NULL);

	std::string filename1 = rtfile.getFileName();
	std::string filename2 = rtfile2.getFileName();

	int cumulative = 0;
	for (int count = 0; count < 2000; count++)
	{
		if (count == 1000)
		{
			Sleep(1000);
			rtfile.newfile(NULL);
			rtfile2.newfile(NULL);
		}

		printf("Sample %d\n", count);

		// random calculations to simulate other processes
		int sum_result = 0;
		for (int j = 0; j < 100000; j++)
			sum_result += j;

		cumulative = cumulative + count;
		float temp[2] = { count, cumulative };
		// save imu data in real time file io
		rtfile.save(NULL, temp);

		rtfile2.save(NULL, temp);

	}

	printf("Press Enter to readback file");
	getchar();

	MatrixIO mio;
	int num_imu = 0;
	int filerows = 0;
	int* imu_order;
	filename1 = "../data/" + filename1 + ".bin";
	Eigen::MatrixXf mat1 = mio.readFromIMUBinaryOrdered(&filename1, 1, &filerows, num_imu, &imu_order);
	std::cout << mat1 << std::endl;

	filename2 = "../data/" + filename2 + ".txt";
	int num_col_per_imu = 0;
	Eigen::MatrixXf mat2 = mio.readFromIMUTextOrdered(filename2, num_imu, num_col_per_imu, &imu_order);
	std::cout << mat2 <<std::endl;

}

*/