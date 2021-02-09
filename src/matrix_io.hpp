#ifndef _MATRIX_IO_HPP_
#define _MATRIX_IO_HPP_

#include <string>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include "main.hpp"


/**
* Matrix Input Output Class
*
* This class is edited from another matrix_io class from "gaussian_mixture_models" project by Roberto Capobianco <capobianco@dis.uniroma1.it>
* @see https://github.com/webrot9/gaussian_mixture_models/
*
* Depends on Eigen Library for matrix objects. Very simple to include that into your program
* @see http://eigen.tuxfamily.org/index.php?title=Main_Page
*
* @author Er Jie Kai (EJK)
* @bug No known bugs.
*/
class MatrixIO {
private:
	int num_imu;
public:
	MatrixIO() {}
	~MatrixIO()
	{
	}

	Eigen::MatrixXf readFromFile(const std::string filename, char delimeter = ' ');
	Eigen::MatrixXf readFromIMUTextOrdered(const std::string filename, int& num_imu, int& num_col_per_imu, int** imu_order);
	Eigen::MatrixXf readFromIMUBinary(const std::string* filename, int numfiles, int numimu, int* filerows);
	Eigen::MatrixXf readFromIMUBinary(const std::string filename, int numimu);
	Eigen::MatrixXf readFromIMUBinaryOrdered(const std::string* filename, int numfiles, int* filerows, int& num_imu, int** imu_order);
	Eigen::MatrixXf readFromIMUBinaryOrdered(const std::string filename, int& num_imu, int** imu_order);
	Eigen::MatrixXf readFromFileBinary(const std::string* filename, int numfiles, int* filerows);
	Eigen::MatrixXf readFromFileBinary(const std::string filename);
	void writeToFile(const std::string filename, const Eigen::MatrixXf& matrix);
	void writeToFileBinary(std::string outputfile, const Eigen::MatrixXf& mat, bool concat = false);
	void writeToFileNumbered(const std::string filename, const Eigen::MatrixXf& matrix);
	void writeToFileIMUBinary(std::string outputfile, const Eigen::MatrixXf& mat, int num_imu);

	void duplicateFile(const std::string source, const std::string destination);
	Eigen::VectorXf formatData(char* rawdata, int numdata);
};

#endif  // _MATRIX_IO_HPP_
