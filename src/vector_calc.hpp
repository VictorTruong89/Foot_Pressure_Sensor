#ifndef VECTORCALC_HPP_
#define VECTORCALC_HPP_

// c++ headers
#include <typeinfo> //for comparing variable type

// Other libraries headers
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "quaternion.hpp"

#define vartype mFLOAT

#if mprecision == mFLOAT
#define var float
#define vtype Eigen::Vector3f
#define qtype Eigen::Quaternionf
#elif mprecision == mDOUBLE
#define var double
#define vtype Eigen::Vector3d
#define qtype Eigen::Quaterniond
#endif

/**
 * A class that performs all 3D vector calculations
*
* @note define precision as need via FLOAT or DOUBLE in the precompile variable
* @note This will change the eigen quaternion and vector type to float or double
*
* Dependencies:
* Eigen library
* @see http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
*
* Created by:
* @author Er Jie Kai (EJK)
 */
 //template <class var, class vtype, class qtype> // var = float/double, vtype = Eigen::Vector3f/Vector3d, qtype = Eigen::Quaternionf/Quaterniond
class VectorCalc
{
private:


public:

	/** @brief Project a vector onto another vector
	*
	* @param[in] v1 vector to be projected onto v2
	* @param[in] v2 the base vector
	*
	* @warning Projection of v1 onto v2
	*
	* @return returns the projected vector
	*/
	vtype Project2Vector(const vtype& v1, const vtype& v2)
	{
		var magnitude = v1.dot(v2) / v2.norm();
		vtype projected = magnitude * (v2 / v2.norm());

		return projected;
	}

	/** @brief Project a vector (v1) onto a plane orthogonal to v2
	*
	* @param[in] v1 vector to be projected onto a plane orthogonal to v2
	* @param[in] v2 the orthogonal vector
	*
	*
	* @return returns the projected vector
	*/
	vtype Project2Plane(const vtype& v1, const vtype& v2)
	{
		vtype v1_project_to_v2 = Project2Vector(v1, v2); // project v1 onto v2
		vtype projected = v1 - v1_project_to_v2; // minus the orthogonal vector (v1ProjectTov2) from original v1 will project v1 onto a plane orthogonal to v1ProjectTov2

		return projected;
	}

	/** @brief Project a vector (v1) onto a plane formed by 2 other vectors
	*
	* @param[in] v1 vector to be projected onto a plane formed by v2 and v3
	* @param[in] v2 the first vector of the plane
	* @param[in] v3 the second vector of the plane
	*
	* @warning The orthogonal of the plane is formed by cross-multiplying v2 to v3
	*
	* @return returns the projected vector
	*/
	vtype Project2Plane(const vtype& v1, const vtype& v2, const vtype& v3)
	{
		vtype orthogonal = v2.cross(v3);
		vtype v1_project_to_orthogonal = Project2Vector(v1, orthogonal); // project v1 onto v2
		vtype projected = v1 - v1_project_to_orthogonal; // minus the orthogonal vector (v1ProjectToOrthogonal) from original v1 will project v1 onto a plane formed by v2 and v3

		return projected;
	}

	/** @brief Calculate angle between 2 vectors
	*
	* if angle positive, it means v1 rotates CCW to v2. Polarity requires a reference up vector to specify which direction is CW or CCW
	*
	* @param[in] v1 the first vector
	* @param[in] v2 the second vector
	* @param[in] up_vec the reference up vector. (CCW rotation about the up_vec is positive; right hand rule)
	*
	* @return returns the angle
	*/
	var AngleBetween2Vectors(const vtype& v1, const vtype& v2, const vtype& up_vec)
	{
		var result = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

		vtype cross = v1.cross(v2);
		if (cross.dot(up_vec) > 0)
			return result;
		else
			return -result;
	}

	/** @brief Project a vector (v1) onto a plane formed by 2 other vectors
	*
	* @param[in] v1 vector to be projected onto a plane formed by v2 and v3
	* @param[in] v2 the first vector of the plane
	* @param[in] v3 the second vector of the plane
	*
	* @warning The orthogonal of the plane is formed by cross-multiplying v2 to v3
	*
	* @return returns the projected vector
	*/
	vtype Rotate2Plane(const vtype& v1, const vtype& v2, const vtype& v3)
	{
		vtype unitY = { 0,1,0 };
		vtype vector_projected = Project2Plane(v1, v2, v3);
		var vector_heading_angle = AngleBetween2Vectors(v1, vector_projected, unitY);

		vtype rot_axis = v1.cross(vector_projected);
		rot_axis.normalize();

		Eigen::Quaternionf mquat;
		mquat.w() = cos(vector_heading_angle / 2);
		mquat.vec() = rot_axis * sin(vector_heading_angle / 2);

		Quaternion quat;
		Eigen::Quaternionf v1_quat = { 0, v1.x(), v1.y(), v1.z() };
		vtype rotated = quat.mult(quat.mult(mquat, v1_quat), mquat.conjugate()).vec();

		return rotated;
	
	}

};



#endif /*VECTORCALC_HPP_*/
