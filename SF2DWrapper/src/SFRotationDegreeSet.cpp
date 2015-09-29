#include "../include/SFRotationDegreeSet.h"

using namespace SF3D;

SFRotationDegreeSet::SFRotationDegreeSet(float rotationOX, float rotationOY, float rotationOZ, SFVector3 c)
{
	OX = rotationOX;
	OY = rotationOY;
	OZ = rotationOZ;
	center = c;
}
