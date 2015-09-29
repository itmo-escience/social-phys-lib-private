#include "../include/SFVector2.h"

using namespace SF2D;

SFVector2::SFVector2(float x, float y)
{
	X = x;
	Y = y;
}

SFVector2 SFVector2::operator + (SFVector2 other)
{
	return SFVector2(this->X + other.X, this->Y + other.Y);
}

SFVector2 SFVector2::operator * (float num)
{
	return SFVector2(this->X * num, this->Y * num);
}

SFVector2 SFVector2::operator +(float num)
{
	return SFVector2(this->X + num, this->Y + num);
}