#include "../include/SimpleMatrix.h"


SimpleMatrix::SimpleMatrix(void)
{
	m11 = m22 = m33 = m44 = 1;
}


SimpleMatrix::~SimpleMatrix(void)
{
}

SimpleMatrix SimpleMatrix::getRotationX(float angle)
{
	SimpleMatrix result = SimpleMatrix();

	float c = cos(angle);
    float s = sin(angle);

    result.m22 = c;
    result.m23 = s;
    result.m32 = -s;
    result.m33 = c;
}
