#include <string>
#include <istream>
#include <stdexcept>

#include "matrix_io.hpp"
#include "variable_conversion.hpp"


/** @brief format the raw binary data to match the format of the matrix used for analysis purposes
 *
 * Convert a raw char byte array into a vector of floats and integers of a single timestamp
 *
 * The vector will follow the following format:
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
 * @param[in] x the 4 byte char array to be converted to integer
 *
 * @return returns the swap endianness integer
 */
Eigen::VectorXf MatrixIO::formatData(char* rawdata, int numdata)
{
	VariableConversion varc;
	Eigen::VectorXf data;
	data.setZero(numdata); //+1 for buttonstate
	data(0) = varc.EndianSwap32i(rawdata);//timestamp
	for (int i = 1; i < numdata - 4; i++) //do for accel and gyro
	{
		data(i) = varc.EndianSwap32f(rawdata + (i * 4));
	}
	data(7) = varc.EndianSwap32f(rawdata + (40)); // quat.w
	data(8) = varc.EndianSwap32f(rawdata + (28)); // quat.x
	data(9) = varc.EndianSwap32f(rawdata + (32)); // quat.y
	data(10) = varc.EndianSwap32f(rawdata + (36)); // quat.z
	//data(0, numdata - 1) = m_endian_swap_4c(rawdata + (numdata * 4 - 4));

	return data;
}

/** @brief read the matrix stored in the file
 *
 * The matrix must be stored with the first row in the following format: \n
 * #size: rows cols \n
 * For example, 10 rows and 2 columns: \n
 * #size 10 2 \n
 *
 * Subsequent rows will be the 10 X 2 data
 *
 * @param[in] filename the file location with the stored matrix
 * @param[in] delimeter the delimeter that separates each data
 *
 * @return returns matrix
 */
Eigen::MatrixXf MatrixIO::readFromFile(const std::string filename, char delimeter) {
	std::ifstream input_file;
	std::string line;
	int rows = 0;
	int cols = 0;
	Eigen::MatrixXf matrix;

	std::cout << "Opening file: " << filename << std::endl;
	input_file.open(filename);

	if (input_file.fail()) {
		char buffer[1000];
		sprintf(buffer, "Impossible to open the file: %s", filename.c_str());
		throw std::runtime_error(buffer);
	}

	int row_idx = 0;

	while (!input_file.fail() && !input_file.eof())
	{
		std::getline(input_file, line);
		std::istringstream iss(line);
		std::string buffer;

		iss >> buffer;
		iss.clear();
		iss.seekg(0, std::ios::beg); //set the position of the next character to be extracted from input stream


		if (buffer.empty() || (buffer[0] == '#' && buffer != "#size:")) {
			continue;
		}

		if (buffer == "#size:" && rows == 0 && cols == 0) {
			std::string str;
			iss >> str;
			iss >> rows;
			iss >> cols;
			matrix = Eigen::MatrixXf::Zero(rows, cols);
			row_idx = 0;
		}
		else {

			if (delimeter == ' ')
			{
				double value;

				for (int i = 0; i < cols && !iss.eof(); ++i) {
					iss >> value;
					matrix(row_idx, i) = value;
				}
			}
			else //other delimeter
			{
				double value;

				for (int i = 0; i < cols && !iss.eof(); ++i) {
					std::string tmp;
					std::getline(iss, tmp, delimeter);
					std::istringstream(tmp) >> value;
					matrix(row_idx, i) = value;
				}

			}


			++row_idx;
		}
	}
	printf("Dataset: %d row X %d col\n", rows, cols);
	input_file.close();
	return matrix;
}

/** @brief read the imu data stored in the file and arrange it in proper order
 *
 * The matrix must be stored with the first row in the following format: \n
 * #size: rows cols \n
 * For example, 10 rows and 2 columns: \n
 * #size 10 2 \n
 *
 * Subsequent rows will be the 10 X 2 data
 *
 * @param[in] filename the file location with the stored matrix
 *
 * @return returns matrix
 */
Eigen::MatrixXf MatrixIO::readFromIMUTextOrdered(const std::string filename, int& num_imu, int& num_col_per_imu, int** imu_order) {
	std::ifstream input_file;
	std::string line;
	int rows = 0;
	int tot_cols = 0;
	//int num_imu = 0;
	//int num_col_per_imu = 0;
	int imu_placement = 0;
	Eigen::MatrixXf matrix;

	std::cout << "Opening file: " << filename << std::endl;
	input_file.open(filename);

	if (input_file.fail()) {
		char buffer[1000];
		sprintf(buffer, "Impossible to open the file: %s", filename.c_str());
		throw std::runtime_error(buffer);
	}

	//find the last line in the text file
	int filepos = -3;
	input_file.seekg(filepos, std::ios::end); //go to one char before end of file (EOF)
	while (1)
	{
		char ch;
		input_file.get(ch); // get the text at the current pos (filepos) of the file
		if (ch == '\n') //new line char is found.
		{
			std::getline(input_file, line);
			std::istringstream iss(line);
			std::string buffer;
			iss >> buffer;

			//printf("buffer = %s\n", buffer.c_str());

			if (buffer == "#size(row,tot_col,num_imu,num_col_per_imu,imu_placement):")
			{
				iss >> rows;
				iss >> tot_cols;
				iss >> num_imu;
				iss >> num_col_per_imu;
				iss >> imu_placement;
				printf("Rows = %d\nTotal Cols = %d\nNum IMU = %d\nNum Col per IMU = %d\nIMU Placement = %d\n", rows, tot_cols, num_imu, num_col_per_imu, imu_placement);
				break;
			}
			else
			{
				throw std::runtime_error("Proper file ending not found.\n");
				break;
			}
		}
		else
		{
			filepos--;
			input_file.seekg(filepos, std::ios::end);
		}
	}

	matrix.resize(rows, tot_cols); //form a matrix with the correct size

	// separate the IMU order into individual numbers
	//delete[] imu_order;
	*imu_order = new int[num_imu];
	for (int imu = 0; imu < num_imu; imu++)
	{
		int base = pow(10, num_imu - imu - 1);
		(*imu_order)[imu] = ceil(imu_placement / base);
		imu_placement = imu_placement - (*imu_order)[imu] * base;
		printf("imu_order[%d] = %d\n", imu, (*imu_order)[imu]);
	}

	// read the data from the file
	input_file.seekg(0, std::ios::beg); //go to first line of file
	int row_idx = 0;
	while (!input_file.fail() && !input_file.eof())
	{
		std::getline(input_file, line);
		std::istringstream iss(line);
		std::string buffer;
		iss >> buffer;
		iss.clear(); //clear any error flags

		if (buffer.empty() || buffer[0] == '#') {
			continue;
		}
		else {
			double value;
			iss.str(line);
			for (int i = 0; i < tot_cols && !iss.eof(); ++i) {
				iss >> value;
				//int imu = floor(i / num_col_per_imu);
				//if (imu < num_imu) //every col that meets this condition belongs to imu data
				//	matrix(row_idx, ((*imu_order)[imu] + i % num_col_per_imu)) = value; // this ensures that all imu are output in ascending order in the output matrix
				//else
				matrix(row_idx, i) = value;

			}

			if (row_idx % 1000 == 0)
				printf(".");
			++row_idx;
		}
	}
	printf("\nDataset: %d row X %d col\n", row_idx, tot_cols);
	input_file.close();

	return matrix;
}

/** @brief read the IMU data stored in the binary file
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
 * The function can read multiple files together and output them as a single matrix. The files must have the same column dimension
 *
 * @param[in] filename an array of all the files to be read
 * @param[in] numfiles the number of files in the array
 * @param[in] numimu the number of IMUs that are used to generate that dataset. Used for error checking
 * @param[out] filerows an output array that stores the number of rows per file
 *
 * @return returns the combined matrix of all the files
 */
Eigen::MatrixXf MatrixIO::readFromIMUBinary(const std::string* filename, int numfiles, int numimu, int* filerows) {
	// The file format of the saved data is as follows:
	// the 1st row is always the number of IMU used to form the data
	// the 2nd row is the datasize in bytes per IMU
	// All subsequent rows are raw data, with each IMU data separated by a space
	Eigen::MatrixXf matrix;
	std::ifstream input_file;
	int rows = 0;
	int cols = 0;
	const int numdata = 11; //1 for timestamp, 3 for accel, 3 for gyro, 4 for quat
	const int datasize = numdata * 4 + 1; //+1 for button state coz it is a char with only 1 byte. the rest are int and float which are 4 byte. button_state is not saved into IMU_data struct
	char data[datasize];

	std::string IMUSeparator = " ";
	std::string IntervalSeparator = "\n\r";
	//std::cout << "IMUSeparator = " << IMUSeparator.length() << std::endl;
	//std::cout << "IntervalSeparator = " << IntervalSeparator.length() << std::endl;
	int datasetIMU = 0;
	int datasetDatasize = 0;
	Eigen::MatrixXf* filematrix;
	filematrix = new Eigen::MatrixXf[numfiles];

	char dump;
	int total;

	for (int i = 0; i < numfiles; i++)
	{
		std::cout << "Opening File: " << filename[i] << std::endl;
		input_file.open(filename[i], std::ios::binary);

		if (input_file.fail()) {
			char buffer[1000];
			sprintf(buffer, "Impossible to open the file: %s", filename[i].c_str());
			throw std::runtime_error(buffer);
		}

		char temp[4];
		char temp2, temp1;
		input_file.get(temp2);
		input_file.seekg(-4, std::ios::end); // go to the last 4 chars before end of file
		for (int i = 0; i < 4; i++)
			input_file.get(temp[i]); //read in the last 4 chars
		unsigned int rowcount;
		memcpy(&rowcount, temp, sizeof(temp)); // convert the temp[4] array into unsigned int "rowcount"
		input_file.seekg(input_file.beg); // go to beginning of file
		rows = rows + rowcount;

		printf("Dataset num of rows = %d\n", rowcount);

		int row_idx = 0;
		//while (!input_file.fail() && !input_file.eof()) {
		for (row_idx = 0; row_idx < rowcount + 2; row_idx++) {

			if (row_idx == 0)
			{
				//check the number of IMU
				char temp = 0;
				input_file.get(temp);
				datasetIMU = (int)temp;
				if (datasetIMU != numimu) {
					printf("Specified number of IMU (%d) is different from dataset specified IMU (%d, %c)!!!!!\n\n", numimu, datasetIMU, temp);
					getchar();
					datasetIMU = numimu;
				}
				else
				{
					printf("Dataset num of IMU = %d\n", datasetIMU);
					//matrix.conservativeResize(rows, numdata * datasetIMU);
					filematrix[i].setZero(rowcount, numdata * datasetIMU);// set the size of matrix. button state is not stored in the matrix
				}
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
				{
					input_file.get(dump); // dump the sample separator
					//printf("dump = %c\n", dump);
				}
				//row_idx++;
			}
			else if (row_idx == 1)
			{
				//check the number of rows
				char temp = 0;
				input_file.get(temp);
				datasetDatasize = (int)temp;
				if (datasetDatasize != datasize) {
					printf("Specified datasize (%d) is different from dataset specified datasize (%d, %c)!!!!!\n\n", datasize, datasetDatasize, temp);
					getchar();
					datasetDatasize = datasize;
				}
				else
					printf("Dataset datasize = %d\n", datasetDatasize);
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
				{
					input_file.get(dump); // dump the sample separator
					//printf("dump = %c\n", dump);
				}
				//row_idx++;
			}
			else
			{
				/////////////////// read data from 1 interval of data ///////////////////
				for (int imu = 0; imu < numimu; imu++)
				{
					for (int byte = 0; byte < datasize; byte++)
						input_file.get(data[byte]);
					for (int byte = 0; byte < IMUSeparator.length(); byte++)
						input_file.get(dump); // dump the imu separator
					Eigen::MatrixXf temp = formatData(data, numdata);

					//matrix.block(row_idx - 2 + rows - rowcount, imu*numdata, 1, numdata) = temp;
					filematrix[i].block(row_idx - 2, imu * numdata, 1, numdata) = temp.transpose();
				}
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

				//row_idx++;
				//matrix.conservativeResize(row_idx + 1, numdata * numimu);
				//std::cout << "row: " << row_idx - 2 << std::endl;

				if (row_idx % 100 == 0)
					printf(" .");
			}
		}
		printf("\n");
		//std::cout << "matrix: " << matrix << std::endl;
		//getchar();
		printf("Finished reading datafile with %d IMU and %d datasize per IMU\n", datasetIMU, datasetDatasize);
		//std::cout << "matrix: " << filematrix[i].row(rowcount - 1) << std::endl;
		//getchar();
		//std::cout << "matrix: " << filematrix[i].row(rowcount) << std::endl;
		//getchar();
		input_file.close();
	}

	matrix.conservativeResize(rows, numdata * datasetIMU);
	for (int i = 0; i < numfiles; i++)
	{
		int startrow;
		if (i == 0)
			startrow = 0;
		else
			startrow += filematrix[i - 1].rows();

		matrix.block(startrow, 0, filematrix[i].rows(), filematrix[i].cols()) = filematrix[i];
		filerows[i] = filematrix[i].rows();
	}

	delete[] filematrix;
	return matrix;
}

/** @brief read single file version of readFromIMUBinary */
Eigen::MatrixXf MatrixIO::readFromIMUBinary(const std::string filename, int numimu)
{
	int rows;
	return readFromIMUBinary(&filename, 1, numimu, &rows);
}

/** @brief read the IMU data stored in the binary file. Sort the data into imu of ascending order
 *
 * Compatible with files generated from RealTimeFileIO
 *
 * The binary file must be stored with the following format: \n
 * the 1st row is always the number of IMU used to form the data \n
 * the 2nd row is the datasize in bytes per IMU \n
 * the 3rd row is the number of extra columns (data in extra cols are always 4 bytes (float)) \n
 * the 4th row is the order of imu. (eg: imu array when input is 3 first, then 1, then 0, then 2 and lastly 4. Then 31024 will be saved.)
 *		- if the zeroth imu is stored first, then the saved imu_placement will have one less significant bit. (eg: 0132 --> 132, because 0 will not be showing)
 * All subsequent rows are raw data, with each IMU data separated by a space \n
 * The last row stores the total rows in the binary files (i.e. how many samples) \n
 * New timestamp is separated by new line (i.e. new row)
 *
 * Placeholder for new row = "\n\r"
 *
 * The function can read multiple files together and output them as a single matrix. The files must have the same column dimension
 *
 * @note the difference between this function and "readFromIMUBinary" is the lack of imu_order in the input data files
 * @note this function cannot be used with datafiles without the imu_order information
 *
 * @param[in] filename an array of all the files to be read
 * @param[in] numfiles the number of files in the array
 * @param[in] numimu the number of IMUs that are used to generate that dataset. Used for error checking
 * @param[out] filerows an output array that stores the number of rows per file
 * @param[out] imu_order an output array that stores the position of each imu on the body (look at IMUPosition enum in imu_store.hpp)
 *
 * @return returns the combined matrix of all the files
 */
Eigen::MatrixXf MatrixIO::readFromIMUBinaryOrdered(const std::string* filename, int numfiles, int* filerows, int& num_imu, int** imu_order) {
	// The file format of the saved data is as follows:
	// the 1st row is always the number of IMU used to form the data
	// the 2nd row is the datasize in bytes per IMU
	// All subsequent rows are raw data, with each IMU data separated by a space
	Eigen::MatrixXf concatenate_datamatrix;
	std::ifstream input_file;
	//int num_imu;
	char* data; // variable for single imu data
	char extra_col_data[4]; //variable for each extra col's data. it is always a float, hence 4 bytes/chars
	//int* imu_order;
	int rows = 0;
	int num_cols = 11;
	int imu_datasize = 0;
	int* extra_col = new int[numfiles];
	int num_header_info = 4;// num_imu, datasize per imu, num extra cols, num rows

	std::string IMUSeparator = " ";
	std::string IntervalSeparator = "\n\r";

	Eigen::MatrixXf* filematrix;
	filematrix = new Eigen::MatrixXf[numfiles];

	char dump;
	int total;

	for (int i = 0; i < numfiles; i++)
	{
		std::cout << "Opening File: " << filename[i] << std::endl;
		input_file.open(filename[i], std::ios::binary);

		if (input_file.fail()) {
			char buffer[1000];
			sprintf(buffer, "Impossible to open the file: %s", filename[i].c_str());
			throw std::runtime_error(buffer);
		}

		char temp[4];
		char temp2, temp1;
		input_file.get(temp2);
		input_file.seekg(-4, std::ios::end); // go to the last 4 chars before end of file
		for (int i = 0; i < 4; i++)
			input_file.get(temp[i]); //read in the last 4 chars
		unsigned int rowcount;
		memcpy(&rowcount, temp, sizeof(temp)); // convert the temp[4] array into unsigned int "rowcount"
		input_file.seekg(input_file.beg); // go to beginning of file
		rows = rows + rowcount;

		printf("Dataset num of rows = %d\n", rowcount);

		int row_idx = 0;
		//while (!input_file.fail() && !input_file.eof()) {

		for (row_idx = 0; row_idx < rowcount + num_header_info; row_idx++) {

			if (row_idx == 0) //check the number of IMU
			{
				char temp = 0;
				input_file.get(temp);
				num_imu = (int)temp;

				printf("Dataset num of IMU = %d\n", num_imu);

				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

			}
			else if (row_idx == 1) //check the datasize
			{
				char temp = 0;
				input_file.get(temp);
				imu_datasize = (int)temp;
				printf("Datasize per IMU = %d\n", imu_datasize);
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

				//filematrix[i].setZero(rowcount, num_imu * imu_datasize);// set the size of matrix. button state is not stored in the matrix
				data = new char[imu_datasize];
			}
			else if (row_idx == 2) //check the number of extra columns
			{
				char temp = 0;
				input_file.get(temp);
				extra_col[i] = (int)temp;
				printf("Extra columns = %d\n", extra_col[i]);

				if (i > 0 && (extra_col[i] != extra_col[0]))
					perror("Datafiles with different number of extra columns cannot be combined!!\n");

				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator
			}
			else if (row_idx == 3) //check the stored imu order
			{
				VariableConversion varc;

				*imu_order = new int[num_imu];

				char temp[4];
				for (int byte = 0; byte < 4; byte++)
					input_file.get(temp[byte]);

				int col = varc.Char2Int(temp);
				printf("IMU order = %d\n", col);
				for (int imu = 0; imu < num_imu; imu++)
				{
					int base = pow(10, num_imu - imu - 1);
					(*imu_order)[imu] = ceil(col / base);
					col = col - (*imu_order)[imu] * base;
					printf("imu_order[%d] = %d\n", imu, (*imu_order)[imu]);
				}

				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

				filematrix[i].setZero(rowcount, num_imu * num_cols + extra_col[i]);// set the size of matrix. button state is not stored in the matrix. 
			}
			else
			{
				/////////////////// read IMU data from 1 interval of data ///////////////////
				for (int imu = 0; imu < num_imu; imu++)
				{
					for (int byte = 0; byte < imu_datasize; byte++)
						input_file.get(data[byte]);
					for (int byte = 0; byte < IMUSeparator.length(); byte++)
						input_file.get(dump); // dump the imu separator
					Eigen::MatrixXf temp = formatData(data, num_cols); //11 -> 1 timestamp, 3 accel, 3 gyro, 4 quat

					//std::cout << temp;

					//filematrix[i].block(row_idx - 2, imu_order[imu] * num_cols, 1, num_cols) = temp.transpose();
					filematrix[i].block(row_idx - num_header_info, imu * num_cols, 1, num_cols) = temp.transpose();
				}

				/////////////////// read extra_col data from 1 interval of data ///////////////////
				for (int ecol = 0; ecol < extra_col[i]; ecol++)
				{
					for (int byte = 0; byte < 4; byte++) //4 bytes make one float
						input_file.get(extra_col_data[byte]);
					for (int byte = 0; byte < IMUSeparator.length(); byte++)
						input_file.get(dump); // dump the imu separator

					VariableConversion varc;


					filematrix[i](row_idx - num_header_info, num_imu* num_cols + ecol) = varc.EndianSwap32f(extra_col_data);
				}

				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

				if (row_idx % 100 == 0)
					printf(" .");
			}
		}
		printf("\n");

		printf("Finished reading datafile with %d IMU and %d datasize per IMU\n", num_imu, imu_datasize);
		//delete[] imu_order;
		delete[] data;
		input_file.close();
	}

	concatenate_datamatrix.conservativeResize(rows, num_imu * num_cols + extra_col[0]);
	for (int i = 0; i < numfiles; i++)
	{
		int startrow;
		if (i == 0)
			startrow = 0;
		else
			startrow += filematrix[i - 1].rows();

		concatenate_datamatrix.block(startrow, 0, filematrix[i].rows(), filematrix[i].cols()) = filematrix[i];
		filerows[i] = filematrix[i].rows();
	}

	delete[] filematrix;
	return concatenate_datamatrix;
}

/** @brief read single file version of readFromIMUBinaryOrdered */
Eigen::MatrixXf MatrixIO::readFromIMUBinaryOrdered(const std::string filename, int& num_imu, int** imu_order)
{
	int rows;
	return readFromIMUBinaryOrdered(&filename, 1, &rows, num_imu, imu_order);
}

/** @brief read the matrix data stored in the binary file
 *
 * The binary file must be stored with the following format: \n
 * the 1st row is the number of rows in the matrix \n
 * the 2nd row is the number of columns in the matrix \n
 * All subsequent rows are raw data. Each data is not separated by anything since the datasize is set at 4 bytes each. \n
 * Each row of data is separated by the new line placeholder "\n\r:
 *
 * Placeholder for new row = "\n\r"
 *
 * The function can read multiple files together and output them as a single matrix. The files must have the same column dimension
 *
 * @param[in] filename an array of all the files to be read
 * @param[in] numfiles the number of files in the array
 * @param[out] filerows an output array that stores the number of rows per file
 *
 * @return returns the combined matrix of all the files
 */
Eigen::MatrixXf MatrixIO::readFromFileBinary(const std::string* filename, int numfiles, int* filerows)
{
	// The file format of the saved data is as follows:
	// the 1st row is the number of rows
	// the 2nd row is the number of columns
	// All subsequent rows are raw data, nothing separates one data from another data since each datasize is fixed at 4 bytes

	Eigen::MatrixXf matrix;
	std::ifstream input_file;
	int totalrows = 0;
	int totalcols = 0;
	char* rowdata;

	std::string IntervalSeparator = "\n\r";
	//std::cout << "IMUSeparator = " << IMUSeparator.length() << std::endl;
	//std::cout << "IntervalSeparator = " << IntervalSeparator.length() << std::endl;
	int datasetrows = 0;
	int datasetcols = 0;
	Eigen::MatrixXf* filematrix;
	filematrix = new Eigen::MatrixXf[numfiles];

	char dump;

	for (int i = 0; i < numfiles; i++)
	{
		std::cout << "Reading File from " << filename[i] << std::endl;
		input_file.open(filename[i], std::ios::binary);

		if (input_file.fail()) {
			char buffer[1000];
			sprintf(buffer, "Impossible to open the file: %s", filename[i].c_str());
			printf("Fail!! %s\n", buffer);
			throw std::runtime_error(buffer);
		}

		//read rows
		char temp[4];
		for (int k = 0; k < 4; k++)
			input_file.get(temp[k]);
		memcpy(&datasetrows, temp, 4);
		//dump separator
		for (int byte = 0; byte < IntervalSeparator.length(); byte++)
			input_file.get(dump); // dump the sample separator

		//read cols
		for (int k = 0; k < 4; k++)
			input_file.get(temp[k]);
		memcpy(&datasetcols, temp, 4);
		//dump separator
		for (int byte = 0; byte < IntervalSeparator.length(); byte++)
			input_file.get(dump); // dump the sample separator

		totalrows += datasetrows;
		totalcols = datasetcols;
		rowdata = new char[datasetcols * 4];
		filematrix[i].setZero(datasetrows, datasetcols);

		printf("File has %d rows, %d cols\n", datasetrows, datasetcols);

		int row_idx = 0;
		for (row_idx = 0; row_idx < datasetrows; row_idx++) {

			for (int byte = 0; byte < datasetcols * 4; byte++)
				input_file.get(rowdata[byte]);

			for (int j = 0; j < datasetcols; j++)
			{
				float temp;
				memcpy(&temp, rowdata + (j * 4), 4);
				filematrix[i](row_idx, j) = temp;
			}

			for (int byte = 0; byte < IntervalSeparator.length(); byte++)
				input_file.get(dump); // dump the sample separator
							//std::cout << "row: " << row_idx-2 << std::endl;
			//std::cout << "matrix: " << matrix.row(row_idx - 2) << std::endl;
			//getchar();
			if (row_idx % 100 == 0)
				printf(" .");
		}

		printf("\n");
		//std::cout << "matrix: " << matrix << std::endl;
		//getchar();
		printf("Finished reading datafile with %d rows and %d columns\n", datasetrows, datasetcols);

		input_file.close();
	}

	matrix.conservativeResize(totalrows, datasetcols);
	for (int i = 0; i < numfiles; i++)
	{
		int startrow;
		if (i == 0)
			startrow = 0;
		else
			startrow += filematrix[i - 1].rows();

		matrix.block(startrow, 0, filematrix[i].rows(), filematrix[i].cols()) = filematrix[i];
		filerows[i] = filematrix[i].rows();
	}

	delete[] rowdata;
	return matrix;
}

/** @brief read the matrix data from a single binary file
 *
 * Look at Eigen::MatrixXf MatrixIO::readFromFileBinary(const std::string* filename, int numfiles, int* filerows)
 *
 * @param[in] filename an array of all the files to be read
 * @param[out] filerows an output array that stores the number of rows per file
 *
 * @return returns the combined matrix of all the files
 */
Eigen::MatrixXf MatrixIO::readFromFileBinary(const std::string filename)
{
	int rows;
	return readFromFileBinary(&filename, 1, &rows);
}

/** @brief write the matrix data to a binary file
 *
 * The binary file must be stored with the following format: \n
 * the 1st row is the number of rows in the matrix \n
 * the 2nd row is the number of columns in the matrix \n
 * All subsequent rows are raw data. Each data is not separated by anything since the datasize is set at 4 bytes each. \n
 * Each row of data is separated by the new line placeholder "\n\r:
 *
 * Placeholder for new row = "\n\r"
 *
 * The function can only write one matrix into the file, but you can concatenate mulitple files by using the "concat" input variable
 *
 * @param[in] outputfile the file location to save the matrix
 * @param[in] mat the matrix to be saved
 * @param[out] concat choose whether to overwrite or concatenate the data
 *
 * @return returns nothing
 */
void MatrixIO::writeToFileBinary(std::string outputfile, const Eigen::MatrixXf& mat, bool concat)
{
	std::cout << "Saving matrix to " << outputfile << std::endl;
	std::ofstream myfile;
	if (concat == false)
		myfile.open(outputfile, std::ios::out | std::ios::binary); //| std::ios::app
	else
		myfile.open(outputfile, std::ios::out | std::ios::app | std::ios::binary);

	char temp[4];
	int rows = mat.rows();
	int cols = mat.cols();
	//write rows
	memcpy(temp, &(rows), sizeof(rows));
	myfile.write(temp, sizeof(rows));
	myfile.write("\n\r", 2); //print interval seperator
	//write cols
	memcpy(temp, &(cols), sizeof(cols));
	myfile.write(temp, sizeof(cols));
	myfile.write("\n\r", 2); //print interval seperator


	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			char temp[4];
			float data = mat(i, j);
			memcpy(temp, &(data), sizeof(data));
			myfile.write(temp, sizeof(data));
		}
		myfile.write("\n\r", 2);
	}

	myfile.close();
}

/** @brief write the matrix data to a text file
 *
 * The matrix must be stored with the first row in the following format: \n
 * #size: rows cols \n
 * For example, 10 rows and 2 columns: \n
 * #size 10 2 \n
 *
 * Subsequent rows will be the 10 X 2 data
 *
 * You can set the precision of the matrix by adjusting the precision class member within the function.
 *
 * @param[in] filename the file location with the stored matrix
 * @param[in] matrix the matrix to be saved
 *
 * @return returns nothing
 */
void MatrixIO::writeToFile(const std::string filename, const Eigen::MatrixXf& matrix) {
	std::ofstream output_file;

	output_file.open(filename);

	if (output_file.fail()) {
		throw std::runtime_error("Impossible to open the file");
	}

	output_file << "#file automatically generated" << std::endl;
	output_file << "#size: " << matrix.rows() << " " << matrix.cols() << std::endl;
	output_file.precision(8);
	output_file << std::scientific;

	output_file << matrix << std::endl;

	output_file.close();
}

/** @brief write the matrix data to a text file with the left most column numbered with row index
 *
 * The matrix must be stored with the first row in the following format: \n
 * #size: rows cols \n
 * For example, 10 rows and 2 columns: \n
 * #size 10 2 \n
 *
 * Subsequent rows will be the 10 X 2 data
 *
 * @note the number of cols stated in the 1st row will include the row index column
 *
 * You can set the precision of the matrix by adjusting the precision class member within the function.
 *
 * @param[in] filename the file location with the stored matrix
 * @param[in] matrix the matrix to be saved
 *
 * @return returns nothing
 */
void MatrixIO::writeToFileNumbered(const std::string filename, const Eigen::MatrixXf& matrix) {
	std::ofstream output_file;

	output_file.open(filename);

	if (output_file.fail()) {
		throw std::runtime_error("Impossible to open the file");
	}

	output_file << "#file automatically generated" << std::endl;
	output_file << "#size: " << matrix.rows() << " " << matrix.cols() + 1 << std::endl;
	output_file.precision(8);
	output_file << std::scientific;

	Eigen::MatrixXf newmatrix(matrix.rows(), matrix.cols() + 1);
	newmatrix.block(0, 1, matrix.rows(), matrix.cols()) = matrix;
	for (int i = 0; i < matrix.rows(); i++)
		newmatrix(i, 0) = i;

	output_file << newmatrix << std::endl;

	output_file.close();
}


/** @brief write the converted IMU data matrix to a binary file
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
 * @param[in] outputfile an array of all the files to be read
 * @param[in] mat the matrix containing the IMU data
 *
 * @return returns nothing
 */
void MatrixIO::writeToFileIMUBinary(std::string outputfile, const Eigen::MatrixXf& mat, int num_imu)
{
	// save as binary file
	std::ofstream myfile;
	myfile.open(outputfile, std::ios::out | std::ios::trunc | std::ios::binary);
	char temp[1];

	VariableConversion varc;

	const int numdata = 11; // 3 for accel, 3 for gyro, 4 for quat, 1 for timestamp
	int datasize = numdata * 4 + 1;

	temp[0] = num_imu;
	myfile.write(temp, 1); //writes the number of imu
	myfile.write("\n\r", 2); //print interval seperator
	temp[0] = datasize;
	myfile.write(temp, 1); //writes the datasize per imu
	myfile.write("\n\r", 2); //print interval seperator

	unsigned int rows = mat.rows();
	for (int k = 0; k < rows; k++)
	{
		for (int i = 0; i < num_imu; i++)
		{
			char temp[4];
			//timestamp
			varc.Int2Char(mat(k, i * 11), temp);
			myfile.write(temp, 4);

			//Accel_x
			varc.Float2Char(mat(k, i * 11 + 1), temp);
			myfile.write(temp, 4);
			//Accel_y
			varc.Float2Char(mat(k, i * 11 + 2), temp);
			myfile.write(temp, 4);
			//Accel_z
			varc.Float2Char(mat(k, i * 11 + 3), temp);
			myfile.write(temp, 4);

			//Gyro_x
			varc.Float2Char(mat(k, i * 11 + 4), temp);
			myfile.write(temp, 4);
			//Gyro_y
			varc.Float2Char(mat(k, i * 11 + 5), temp);
			myfile.write(temp, 4);
			//Gyro_z
			varc.Float2Char(mat(k, i * 11 + 6), temp);
			myfile.write(temp, 4);

			//Quat_w
			varc.Float2Char(mat(k, i * 11 + 7), temp);
			myfile.write(temp, 4);
			//Quat_x
			varc.Float2Char(mat(k, i * 11 + 8), temp);
			myfile.write(temp, 4);
			//Quat_y
			varc.Float2Char(mat(k, i * 11 + 9), temp);
			myfile.write(temp, 4);
			//Quat_z
			varc.Float2Char(mat(k, i * 11 + 10), temp);
			myfile.write(temp, 4);

			//button
			char button = 0; //since button state is not converted to IMU data, so we just set the button state to constant 0
			myfile.write(&button, 1);

			//IMU separator
			myfile.write(" ", 1);
		}
		//Timestamp (sample) separator
		myfile.write("\n\r", 2);
	}

	char temp4[4];
	memcpy(temp4, (char*)&rows, sizeof(unsigned int)); // store the row count as the last value in little endian format
	myfile.write(temp4, 4);
	myfile.close();

}


/** @brief duplicate a file byte by byte
 *
 * @param[in] source the source file to be copied
 * @param[out] destination the destination file to save the copied data
 *
 * @return returns nothing
 */
void MatrixIO::duplicateFile(const std::string source, const std::string destination)
{
	std::cout << "copying from: " << source << " to " << destination << std::endl;

	std::ifstream src;
	size_t size = 0; // here

	src.open(source, std::ios::in | std::ios::binary);
	char* oData = 0;

	src.seekg(0, std::ios::end); // set the pointer to the end
	size = src.tellg(); // get the length of the file
	std::cout << "Size of src: " << size << std::endl;
	src.seekg(0, std::ios::beg); // set the pointer to the beginning

	oData = new char[size]; //  for the '\0'
	src.read(oData, size); // may have problem if size of source is larger than the size of the computer's ram

	//print data
	std::ofstream dst;
	dst.open(destination, std::ios::out | std::ios::binary | std::ios::trunc);
	//for (size_t i = 0; i < strlen(oData); i++)
	dst.write(oData, size);
	std::cout << "Finish Duplicating file to " << destination << std::endl;
	delete[] oData;

	src.close();
	dst.close();
}

/*
void MatrixIO::rearrangeIMUBinary(const std::string infilename, const std::string outfilename)
{
	// When the IMU is stored in a different order than what is intended, we need to rearrange the IMU stored axis
	// The file format of the saved data is as follows:
	// the 1st row is always the number of IMU used to form the data
	// the 2nd row is the datasize in bytes per IMU
	// All subsequent rows are raw data, with each IMU data separated by a space

#if num_imu == 3
	int rearrange_order[num_imu] = { 0,2,1};
#elif num_imu == 5
	int rearrange_order[num_imu] = { 0,2,3,1,4 }; //{ 0,1,3,4,2 };
#endif

	int rows = 0;
	int cols = 0;
	const int numdata = 11; //1 for timestamp, 3 for accel, 3 for gyro, 4 for quat
	const int datasize = numdata * 4 + 1; //+1 for button state coz it is a char with only 1 byte. the rest are int and float which are 4 byte. button_state is not saved into IMU_data struct
	char data[num_imu][datasize];

	std::string IMUSeparator = " ";
	std::string IntervalSeparator = "\n\r";
	//std::cout << "IMUSeparator = " << IMUSeparator.length() << std::endl;
	//std::cout << "IntervalSeparator = " << IntervalSeparator.length() << std::endl;
	int datasetIMU = 0;
	int datasetDatasize = 0;
	Eigen::MatrixXf *filematrix;

	char dump;
	int total;


		std::cout << "Opening File: " << infilename << std::endl;
		std::ifstream input_file;
		input_file.open(infilename, std::ios::binary);
		if (input_file.fail()) {
			char buffer[1000];
			sprintf(buffer, "Impossible to open the input file: %s", infilename.c_str());
			throw std::runtime_error(buffer);
		}

		std::ofstream output_file;
		output_file.open(outfilename, std::ios::out | std::ios::trunc | std::ios::binary);
		if (output_file.fail()) {
			char buffer[1000];
			sprintf(buffer, "Impossible to open the output file: %s", infilename.c_str());
			throw std::runtime_error(buffer);
		}


		char temp[4];
		char temp2, temp1;
		input_file.get(temp2);
		input_file.seekg(-4, std::ios::end); // go to the last 4 chars before end of file
		for (int i = 0; i < 4; i++)
			input_file.get(temp[i]); //read in the last 4 chars (row count)
		unsigned int rowcount;
		memcpy(&rowcount, temp, sizeof(temp)); // convert the temp[4] array into unsigned int "rowcount"
		input_file.seekg(input_file.beg); // go to beginning of file

		printf("Dataset num of rows = %d\n", rowcount);

		int row_idx = 0;
		//while (!input_file.fail() && !input_file.eof()) {
		for (row_idx = 0; row_idx < rowcount + 2; row_idx++) {

			if (row_idx == 0)
			{
				//check the number of IMU
				char temp = 0;
				input_file.get(temp);
				datasetIMU = (int)temp;
				if (datasetIMU != num_imu) {
					printf("Specified number of IMU (%d) is different from dataset specified IMU (%d, %c)!!!!!\n\n", num_imu, datasetIMU, temp);
					getchar();
					datasetIMU = num_imu;
				}
				else
				{
					printf("Dataset num of IMU = %d\n", datasetIMU);
					//write to outputfile
					char temp[1] = { datasetIMU }; //add 48 because ascii 0 is 48
					output_file.write(temp, 1);
					output_file.write("\n\r", 2); //print interval seperator

				}
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
				{
					input_file.get(dump); // dump the sample separator
										  //printf("dump = %c\n", dump);
				}
				//row_idx++;
			}
			else if (row_idx == 1)
			{
				//check the number of rows
				char temp = 0;
				input_file.get(temp);
				datasetDatasize = (int)temp;
				if (datasetDatasize != datasize) {
					printf("Specified datasize (%d) is different from dataset specified datasize (%d, %c)!!!!!\n\n", datasize, datasetDatasize, temp);
					getchar();
					datasetDatasize = datasize;
				}
				else
				{
					printf("Dataset datasize = %d\n", datasetDatasize);
					//write to outputfile
					char temp[1] = { datasetDatasize }; //48 because ascii value of 0 is 48.
					output_file.write(temp, 1);
					output_file.write("\n\r", 2); //print interval seperator
				}
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
				{
					input_file.get(dump); // dump the sample separator
										  //printf("dump = %c\n", dump);
				}
				//row_idx++;
			}
			else
			{

				// Read data from File
				for (int imu = 0; imu < datasetIMU; imu++)
				{
					for (int byte = 0; byte < datasize; byte++)
						input_file.get(data[imu][byte]);
					for (int byte = 0; byte < IMUSeparator.length(); byte++)
						input_file.get(dump); // dump the imu separator

				}
				for (int byte = 0; byte < IntervalSeparator.length(); byte++)
					input_file.get(dump); // dump the sample separator

				// Write data to output file
				for (int imu = 0; imu < datasetIMU; imu++)
				{
					output_file.write(data[rearrange_order[imu]], datasize);
					output_file.write(" ", 1);
				}
				output_file.write("\n\r", 2);

				if (row_idx % 100 == 0)
					printf(" .");
			}
		}

		memcpy(temp, (char*)&rowcount, sizeof(unsigned int)); // store the row count as the last value in little endian format
		output_file.write(temp, 4);
		output_file.close();

		printf("\n");
		//std::cout << "matrix: " << matrix << std::endl;
		//getchar();
		printf("Finished rearranging datafile with %d IMU and %d datasize per IMU\n", datasetIMU, datasetDatasize);
		//std::cout << "matrix: " << filematrix[i].row(rowcount - 1) << std::endl;
		//getchar();
		//std::cout << "matrix: " << filematrix[i].row(rowcount) << std::endl;
		//getchar();
		input_file.close();

}*/
