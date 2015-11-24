#include "../include/SFVector3.h"

using namespace SF3D;

SFVector3::SFVector3(float x, float y, float z)
{
	X = x;
	Y = y;
	Z = z;
}

SFVector3 SFVector3::operator + (const SFVector3 &other)
{
	return SFVector3(this->X + other.X, this->Y + other.Y, this->Z + other.Z);
}

SFVector3 SFVector3::operator * (float num)
{
	return SFVector3(this->X * num, this->Y * num, this->Z * num);
}

SFVector3 SFVector3::operator +(float num)
{
	return SFVector3(this->X + num, this->Y + num, this->Z + num);
}
