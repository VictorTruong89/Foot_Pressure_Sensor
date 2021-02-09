#ifndef VARIABLECONVERSION_HPP_
#define VARIABLECONVERSION_HPP_

#include <cstring> //for memcpy

/** A class that performs variable conversions
*
* 1) Changes the endian of the variable
* 2) Changes int to char array
* 3) Changes float to char array
*
* Created by:
* @author Er Jie Kai (EJK)
 */
class VariableConversion
{
public:
	/** @brief Swap the endianness of a 16 bit variable
	*
	* @param[in/out] x the 16 byte char array to be swapped
	*
	* @return returns nothing [output via pointer]
	*/
	void EndianSwap16(unsigned short* x)
	{
		*x = (*x >> 8) |
			(*x << 8);
	}

	/** @brief Swap the endianness of a 32 bit variable
	*
	* @param[in/out] x the 32 byte char array to be swapped
	*
	* @return returns nothing [output via pointer]
	*/
	void EndianSwap32(unsigned int* x)
	{
		*x = (*x >> 24) |
			((*x << 8) & 0x00FF0000) |
			((*x >> 8) & 0x0000FF00) |
			(*x << 24);
	}

	/** @brief Swap the endianness of a float
	*
	* Convert your input float into a 4 element char array before using this function
	*
	* @param[in] x the 4 byte char array to be converted to float
	*
	* @return returns the swap endianness float
	*/
	float EndianSwap32f(const char* x)
	{
		char temp[4];
		temp[0] = x[3];
		temp[1] = x[2];
		temp[2] = x[1];
		temp[3] = x[0];
		float f;
		memcpy(&f, temp, sizeof(temp));
		return f;
	}

	/** @brief Swap the endianness of an integer
	*
	* Convert your input integer into a 4 element char array before using this function
	*
	* @param[in] x the 4 byte char array to be converted to integer
	*
	* @return returns the swap endianness integer
	*/
	int EndianSwap32i(char* x)
	{
		char temp[4];
		temp[0] = x[3];
		temp[1] = x[2];
		temp[2] = x[1];
		temp[3] = x[0];
		int f;
		memcpy(&f, temp, sizeof(temp));
		return f;
	}

	/** @brief Convert any 4 byte char array into a float
	*
	* @param[in] x the 4 byte char array to be converted to float
	*
	* @return returns the converted float
	*/
	float Char2Float(char* array)
	{
		float f;
		memcpy(&f, array, sizeof(f));
		return f;
	}

	/** @brief Convert any 4 byte char array into a int
	*
	* @param[in] x the 4 byte char array to be converted to int
	*
	* @return returns the converted int
	*/
	int Char2Int(char* array)
	{
		int f;
		memcpy(&f, array, sizeof(f));
		return f;
	}

	/** @brief Converts an integer to character array in little endian
	*
	* @param[in] input the integer to be converted
	* @param[out] output the converted character array
	*
	* @return returns nothing [output via pointer]
	*/
	void Int2Char(int input, char output[])
	{
		char temp[4];
		memcpy(temp, (char*)&input, sizeof(int));
		output[0] = temp[3];
		output[1] = temp[2];
		output[2] = temp[1];
		output[3] = temp[0];
	}

	/** @brief Converts a float to character array in little endian
	*
	* @param[in] input the float to be converted
	* @param[out] output the converted character array
	*
	* @return returns nothing [output via pointer]
	*/
	void Float2Char(float input, char output[])
	{
		char temp[4];
		memcpy(temp, (char*)&input, sizeof(float));
		output[0] = temp[3];
		output[1] = temp[2];
		output[2] = temp[1];
		output[3] = temp[0];
	}

	/** @brief Converts a short to character array in little endian
	*
	* @param[in] input the float to be converted
	* @param[out] output the converted character array
	*
	* @return returns nothing [output via pointer]
	*/
	void Short2Char(unsigned short input, char output[])
	{
		char temp[2];
		memcpy(temp, (char*)&input, sizeof(unsigned short));
		output[0] = temp[1];
		output[1] = temp[0];
	}

};
#endif /*VARIABLECONVERSION_HPP_*/
