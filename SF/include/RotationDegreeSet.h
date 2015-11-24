/*
 *  RotationDegreeSet.h
 *  SF-3D Library.
 */

/*!
 *  @file		Vector3.h
 *  @brief		Contains the Vector3 class.
 */
#pragma once

#include <cmath>
#include <ostream>
#include "Vector3.h"

namespace SF
{
/*!
 *  @brief		Defines a rotaion degrees.
 */
class RotationDegreeSet
{
public:
	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector instance to zero.
	 */
	inline RotationDegreeSet()
	{
		val_[0] = 0.0f;
		val_[1] = 0.0f;
		val_[2] = 0.0f;

		center_ = Vector3();
	}

	RotationDegreeSet& operator=(const RotationDegreeSet &right)
	{
		if (this == &right)
			return *this;

		val_[0] = right.val_[0];
		val_[1] = right.val_[1];
		val_[2] = right.val_[2];

		center_ = right.center_;

		return *this;
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified three-dimensional vector.
	 *  @param		vector		The three-dimensional vector containing the xyz-coordinates.
	 */
	inline RotationDegreeSet(const RotationDegreeSet& set)
	{
		val_[0] = set.getRotationOX();
		val_[1] = set.getRotationOY();
		val_[2] = set.getRotationOZ();

		center_ = set.getCenter();
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified three-element array.
	 *  @param		val		The three-element array containing the xyz-coordinates.
	 */
	inline explicit RotationDegreeSet(const float val[3], const Vector3 &c)
	{
		val_[0] = val[0];
		val_[1] = val[1];
		val_[2] = val[2];

		center_ = c;
	}

	/*!
	 *  @brief		Constructs and initializes a three-dimensional vector from the specified xyz-coordinates.
	 *  @param		vx		The x-coordinate of the three-dimensional vector.
	 *  @param		vy		The y-coordinate of the three-dimensional vector.
	 *  @param		vz		The z-coordinate of the three-dimensional vector.
	 */
	inline RotationDegreeSet(float ox, float oy, float oz, Vector3 c)
	{
		val_[0] = ox;
		val_[1] = oy;
		val_[2] = oz;

		center_ = c;
	}

	inline float getRotationOX() const
	{
		return val_[0];
	}

	inline float getRotationOY() const
	{
		return val_[1];
	}

	inline float getRotationOZ() const
	{
		return val_[2];
	}

	inline Vector3 getCenter() const
	{
		return center_;
	}

private:
	float val_[3];
	Vector3 center_;
};
}