#pragma once

#include <cmath>
#include <ostream>

#include "Vector3.h"

namespace SF
{
	/// <summary> Defines a rotaion degrees </summary>
	class RotationDegreeSet
	{
	public:
		/// <summary> Constructs and initializes a three-dimensional vector instance to zero </summary>
		inline RotationDegreeSet()
		{
			val_[0] = 0.0f;
			val_[1] = 0.0f;
			val_[2] = 0.0f;

			center_ = Vector3();
		}

		/// <summary> Defines operator = </summary>
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

		/// <summary> Constructs and initializes a three-dimensional vector from the specified three-dimensional vector </summary>
		/// <param name="set"> The three-dimensional vector containing the xyz-coordinates </param>
		inline RotationDegreeSet(const RotationDegreeSet& set)
		{
			val_[0] = set.getRotationOX();
			val_[1] = set.getRotationOY();
			val_[2] = set.getRotationOZ();

			center_ = set.getCenter();
		}

		/// <summary> Constructs and initializes a three-dimensional vector from the specified three-element array </summary>
		/// <param name="val"> The three-element array containing the xyz-coordinates </param>
		inline explicit RotationDegreeSet(const float val[3], const Vector3 &c)
		{
			val_[0] = val[0];
			val_[1] = val[1];
			val_[2] = val[2];

			center_ = c;
		}

		/// <summary> Constructs and initializes a three-dimensional vector from the specified xyz-coordinates </summary>
		/// <param name="ox"> The x-coordinate of the three-dimensional vector </param>
		/// <param name="oy"> The y-coordinate of the three-dimensional vector </param>
		/// <param name="oz"> The z-coordinate of the three-dimensional vector </param>
		inline RotationDegreeSet(float ox, float oy, float oz, Vector3 c)
		{
			val_[0] = ox;
			val_[1] = oy;
			val_[2] = oz;

			center_ = c;
		}

		/// <summary> Returns OX coord </summary>
		inline float getRotationOX() const
		{
			return val_[0];
		}

		/// <summary> Returns OY coord </summary>
		inline float getRotationOY() const
		{
			return val_[1];
		}

		/// <summary> Returns OZ coord </summary>
		inline float getRotationOZ() const
		{
			return val_[2];
		}

		/// <summary> Returns center </summary>
		inline Vector3 getCenter() const
		{
			return center_;
		}

	private:
		float val_[3];		// vector component array
		Vector3 center_;	// center
	};
}