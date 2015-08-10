#include "../include/SimpleMatrix.h"


SimpleMatrix::SimpleMatrix(void)
{
	m11 = m22 = m33 = m44 = 1;
}

SimpleMatrix::SimpleMatrix(float value)
{
	m11 = m12 = m13 = m14 =
    m21 = m22 = m23 = m24 =
    m31 = m32 = m33 = m34 =
    m41 = m42 = m43 = m44 = value;
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

void SimpleMatrix::getInvert()
{
	
}
