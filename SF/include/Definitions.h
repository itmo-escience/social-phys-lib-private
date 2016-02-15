/// <summary> Contains functions and constants used in multiple classes </summary>

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <cmath>
#include <limits>
#include <vector>

#include "Vector2.h"

static const float SF_EPSILON = 0.00001f;	// A sufficiently small positive number.

namespace SF
{
	class Agent;
	class SFSimulator;
	class Obstacle;
	class AgentPropertyConfig;

	/// <summary> Computes the squared distance from a line segment with the specified endpoints to a specified point </summary>
	/// <param name="a"> Position of start of line </param>
	/// <param name="b"> Position of end of line </param>
	/// <param name="c"> Selected point </param>
	/// <returns> The nearest point </returns>
	inline float distSqPointLineSegment(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		const float r = ((c - a) * (b - a)) / absSq(b - a);

		if (r < 0.0f) 
			return absSq(c - a);
		else if (r > 1.0f) 
			return absSq(c - b);
		else
			return absSq(c - (a + r * (b - a)));
	}

	/// <summary> Computes the signed distance from a line connecting the specified points to a specified point </summary>
	/// <param name="a"> Position of start of line </param>
	/// <param name="b"> Position of end of line </param>
	/// <param name="c"> The point to which the signed distance is to be calculated </param>
	/// <returns> Positive when the point c lies to the left of the line ab </returns>
	inline float leftOf(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		return det(a - c, b - a);
	}

	/// <summary> Computes the square of a float </summary>
	/// <param name="a"> The float to be squared </param>
	/// <returns> The square of the float </returns>
	inline float sqr(float a)
	{
		return a * a;
	}
}

#endif
