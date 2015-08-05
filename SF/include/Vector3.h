/*
 *  Vector3.h
 *  SF-3D Library.
 */

/*!
 *  @file		Vector3.h
 *  @brief		Contains the Vector3 class.
 */
#ifndef VECTOR_3_H
#define VECTOR_3_H

#include <cmath>
#include <ostream>

#include "SimpleMatrix.h"

namespace SF
{
/*!
 *  @brief		Defines a three-dimensional vector.
 */
__declspec(dllexport) class Vector3
{
public:
	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector instance to zero.
	 */
	inline Vector3()
	{
		val_[0] = 0.0f;
		val_[1] = 0.0f;
		val_[2] = 0.0f;
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector containing the xyz-coordinates.
	 */
	inline Vector3(const Vector3& vector)
	{
		val_[0] = vector[0];
		val_[1] = vector[1];
		val_[2] = vector[2];
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified three-element array.
	 *  @param		val		The three-element array containing the xyz-coordinates.
	 */
	inline explicit Vector3(const float val[3])
	{
		val_[0] = val[0];
		val_[1] = val[1];
		val_[2] = val[2];
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified xyz-coordinates.
	 *  @param		vx		The x-coordinate of the three-dimensional vector.
	 *  @param		vy		The y-coordinate of the three-dimensional vector.
	 *  @param		vz		The z-coordinate of the three-dimensional vector.
	 */
	inline Vector3(float vx, float vy, float vz)
	{
		val_[0] = vx;
		val_[1] = vy;
		val_[2] = vz;
	}

	inline float getLength()
	{
		return sqrt(pow(x(), 2) + pow(y(), 2) + pow(z(), 2));
	}

	/*!
	 *  @brief		Returns the x-coordinate of this three-dimensional vector.
	 *  @returns	The x-coordinate of the three-dimensional vector.
	 */
	inline float x() const
	{
		return val_[0];
	}

	/*!
	 *  @brief		Returns the y-coordinate of this three-dimensional vector.
	 *  @returns	The y-coordinate of the three-dimensional vector.
	 */
	inline float y() const
	{
		return val_[1];
	}

	/*!
	 *  @brief		Returns the z-coordinate of this three-dimensional vector.
	 *  @returns	The z-coordinate of the three-dimensional vector.
	 */
	inline float z() const
	{
		return val_[2];
	}

	/*!
	 *  @brief		Returns the specified coordinate of this three-dimensional vector.
	 *  @param		i		The coordinate that should be returned (0 <= i < 3).
	 *  @returns    The specified coordinate of the three-dimensional vector.
	 */
	inline float operator[](size_t i) const
	{
		return val_[i];
	}

	/*!
	 *  @brief		Returns a reference to the specified coordinate of this three-dimensional vector.
	 *  @param		i		The coordinate to which a reference should be returned (0 <= i < 3).
	 *  @returns	A reference to the specified coordinate of the three-dimensional vector.
	 */
	inline float& operator[](size_t i)
	{
		return val_[i];
	}

	/*!
	 *  @brief		Computes the negation of this three-dimensional vector.
	 *  @returns	The negation of this three-dimensional vector.
	 */
	inline Vector3 operator-() const
	{
		return Vector3(-val_[0], -val_[1], -val_[2]);
	}

	/*!
	 *  @brief		Computes the dot product of this three-dimensional vector with the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector with which the dot product should be computed.
	 *  @returns	The dot product of this three-dimensional vector with a specified three-dimensional vector.
	 */
	inline float operator*(const Vector3& vector) const
	{
		return val_[0] * vector[0] + val_[1] * vector[1] + val_[2] * vector[2];
	}

	/*!
	 *  @brief		Computes the scalar multiplication of this three-dimensional vector with the specified scalar value.
	 *  @param		scalar		The scalar value with which the scalar multiplication should be computed.
	 *  @returns	The scalar multiplication of this three-dimensional vector with a specified scalar value.
	 */
	inline Vector3 operator*(float scalar) const
	{
		return Vector3(val_[0] * scalar, val_[1] * scalar, val_[2] * scalar);
	}

	/*!
	 *  @brief		Computes the scalar division of this three-dimensional vector
	 *              with the specified scalar value.
	 *  @param		scalar		The scalar value with which the scalar division should be computed.
	 *  @returns	The scalar division of this three-dimensional vector with a specified scalar value.
	 */
	inline Vector3 operator/(float scalar) const
	{
		const float invScalar = 1.0f / scalar;

		return Vector3(val_[0] * invScalar, val_[1] * invScalar, val_[2] * invScalar);
	}

	/*!
	 *  @brief		Computes the vector sum of this three-dimensional vector with the specified three-dimensional vector.
	 *  @param		vector	The three-dimensional vector with which the vector sum should be computed.
	 *  @returns	The vector sum of this three-dimensional vector with a specified three-dimensional vector.
	 */
	inline Vector3 operator+(const Vector3& vector) const
	{
		return Vector3(val_[0] + vector[0], val_[1] + vector[1], val_[2] + vector[2]);
	}

	/*!
	 *  @brief		Computes the vector difference of this three-dimensional vector with the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector with which the vector difference should be computed.
	 *  @returns	The vector difference of this three-dimensional vector with a specified three-dimensional vector.
	 */
	inline Vector3 operator-(const Vector3& vector) const
	{
		return Vector3(val_[0] - vector[0], val_[1] - vector[1], val_[2] - vector[2]);
	}

	/*!
	 *  @brief		Tests this three-dimensional vector for equality with the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector with which to test for equality.
	 *  @returns	True if the three-dimensional vectors are equal.
	 */
	inline bool operator==(const Vector3& vector) const
	{
		return val_[0] == vector[0] && val_[1] == vector[1] && val_[2] == vector[2];
	}

	/*!
	 *  @brief		Tests this three-dimensional vector for inequality with the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector with which to test for inequality.
	 *  @returns	True if the three-dimensional vectors are not equal.
	 */
	inline bool operator!=(const Vector3& vector) const
	{
		return val_[0] != vector[0] || val_[1] != vector[1] || val_[2] != vector[2];
	}

	/*!
	 *  @brief		Sets the value of this three-dimensional vector to the scalar multiplication of itself with the specified scalar value.
	 *  @param		scalar		The scalar value with which the scalar multiplication should be computed.
	 *  @returns	A reference to this three-dimensional vector.
	 */
	inline Vector3& operator*=(float scalar)
	{
		val_[0] *= scalar;
		val_[1] *= scalar;
		val_[2] *= scalar;

		return *this;
	}

	/*!
	 *  @brief		Sets the value of this three-dimensional vector to the scalar division of itself with the specified scalar value.
	 *  @param		scalar		The scalar value with which the scalar division should be computed.
	 *  @returns	A reference to this three-dimensional vector.
	 */
	inline Vector3& operator/=(float scalar)
	{
		const float invScalar = 1.0f / scalar;

		val_[0] *= invScalar;
		val_[1] *= invScalar;
		val_[2] *= invScalar;

		return *this;
	}

	/*!
	 *  @brief		Sets the value of this three-dimensional vector to the vector
	 *              sum of itself with the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector with which the vector sum should be computed.
	 *  @returns	A reference to this three-dimensional vector.
	 */
	inline Vector3& operator+=(const Vector3& vector)
	{
		val_[0] += vector[0];
		val_[1] += vector[1];
		val_[2] += vector[2];

		return *this;
	}

	/*!
	 *  @brief		Sets the value of this three-dimensional vector to the vector difference of itself with the specified three-dimensional vector.
	 *  @param		vector	The three-dimensional vector with which the vector difference should be computed.
	 *  @returns	A reference to this three-dimensional vector.
	 */
	inline Vector3& operator-=(const Vector3& vector)
	{
		val_[0] -= vector[0];
		val_[1] -= vector[1];
		val_[2] -= vector[2];

		return *this;
	}

	inline static Vector3& transformCoordinate(Vector3 coordinate, SimpleMatrix transform)
	{
		int 
			X,
			Y,
			Z,
			W;

		X = (coordinate.x() * transform.m11) + (coordinate.y() * transform.m21) + (coordinate.z() * transform.m31) + transform.m41;
		Y = (coordinate.x() * transform.m12) + (coordinate.y() * transform.m22) + (coordinate.z() * transform.m32) + transform.m42;
        Z = (coordinate.x() * transform.m13) + (coordinate.y() * transform.m23) + (coordinate.z() * transform.m33) + transform.m43;
        W = 1 / ((coordinate.x() * transform.m14) + (coordinate.y() * transform.m24) + (coordinate.z() * transform.m34) + transform.m44);

        return Vector3(X * W, Y * W, Z * W);
	}

	inline static Vector3& transformNormal(Vector3 normal, SimpleMatrix transform)
	{
		int X,
			Y,
			Z;

		X = (normal.x() * transform.m11) + (normal.y() * transform.m21) + (normal.z() * transform.m31);
		Y = (normal.x() * transform.m12) + (normal.y() * transform.m22) + (normal.z() * transform.m32);
		Z = (normal.x() * transform.m13) + (normal.y() * transform.m23) + (normal.z() * transform.m33);

		return Vector3(X, Y, Z);
	}


private:
	float val_[3];
};


/*!
 *  @brief		Computes the scalar multiplication of the specified three-dimensional vector with the specified scalar value.
 *  @param		scalar		The scalar value with which the scalar multiplication should be computed.
 *  @param      vector		The three-dimensional vector with which the scalar multiplication should be computed.
 *  @returns    The scalar multiplication of the three-dimensional vector with the scalar value.
 */
inline Vector3 operator*(float scalar, const Vector3& vector)
{
	return Vector3(scalar * vector[0], scalar * vector[1], scalar * vector[2]);
}

/*!
 *  @brief		Computes the cross product of the specified three-dimensional vectors.
 *  @param		vector1		The first vector with which the cross product should be computed.
 *  @param		vector2		The second vector with which the cross product should be computed.
 *  @returns	The cross product of the two specified vectors.
 */
inline Vector3 cross(const Vector3& vector1, const Vector3& vector2)
{
	return Vector3(vector1[1] * vector2[2] - vector1[2] * vector2[1], vector1[2] * vector2[0] - vector1[0] * vector2[2], vector1[0] * vector2[1] - vector1[1] * vector2[0]);
}

/*!
 *  @brief		Inserts the specified three-dimensional vector into the specified output stream.
 *  @param		os			The output stream into which the three-dimensional vector should be inserted.
 *  @param		vector		The three-dimensional vector which to insert into the output stream.
 *  @returns	A reference to the output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const Vector3& vector)
{
	os << "(" << vector[0] << "," << vector[1] << "," << vector[2] << ")";

	return os;
}

/*!
 *  @brief		Computes the length of a specified three-dimensional vector.
 *  @param		vector		The three-dimensional vector whose length is to be computed.
 *  @returns	The length of the three-dimensional vector.
 */
inline float abs(const Vector3& vector)
{
	return std::sqrt(vector * vector);
}

/*!
 *  @brief		Computes the squared length of a specified three-dimensional vector.
 *  @param      vector		The three-dimensional vector whose squared length is to be computed.
 *  @returns    The squared length of the three-dimensional vector.
 */
inline float absSq(const Vector3& vector)
{
	return vector * vector;
}

/*!
 *  @brief		Computes the normalization of the specified three-dimensional vector.
 *  @param		vector		The three-dimensional vector whose normalization is to be computed.
 *  @returns	The normalization of the three-dimensional vector.
 */
inline Vector3 normalize(const Vector3& vector)
{
	return vector / abs(vector);
}
}

#endif
