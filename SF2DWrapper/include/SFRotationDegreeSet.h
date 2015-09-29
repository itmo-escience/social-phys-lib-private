#include "SFVector3.h"

namespace SF3D 
{
	public value class SFRotationDegreeSet
	{
	public:
		float OX;
		float OY;
		float OZ;
		SFVector3 center;

		SFRotationDegreeSet(float x, float y, float z, SFVector3 c);
	};
}