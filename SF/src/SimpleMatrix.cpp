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
	SimpleMatrix result = SimpleMatrix();

	float b0 = (m31 * m42) - (m32 * m41);
    float b1 = (m31 * m43) - (m33 * m41);
    float b2 = (m34 * m41) - (m31 * m44);
    float b3 = (m32 * m43) - (m33 * m42);
    float b4 = (m34 * m42) - (m32 * m44);
    float b5 = (m33 * m44) - (m34 * m43);

    float d11 = m22 * b5 + m23 * b4 + m24 * b3;
    float d12 = m21 * b5 + m23 * b2 + m24 * b1;
    float d13 = m21 * -b4 + m22 * b2 + m24 * b0;
    float d14 = m21 * b3 + m22 * -b1 + m23 * b0;

    float det = m11 * d11 - m12 * d12 + m13 * d13 - m14 * d14;
    if (fabs(det) < 0.0001f)
    {
        result = SimpleMatrix(0.0f);
        return;
    }

    det = 1.0f / det;

    float a0 = (m11 * m22) - (m12 * m21);
    float a1 = (m11 * m23) - (m13 * m21);
    float a2 = (m14 * m21) - (m11 * m24);
    float a3 = (m12 * m23) - (m13 * m22);
    float a4 = (m14 * m22) - (m12 * m24);
    float a5 = (m13 * m24) - (m14 * m23);

    float d21 = m12 * b5 + m13 * b4 + m14 * b3;
    float d22 = m11 * b5 + m13 * b2 + m14 * b1;
    float d23 = m11 * -b4 + m12 * b2 + m14 * b0;
    float d24 = m11 * b3 + m12 * -b1 + m13 * b0;

    float d31 = m42 * a5 + m43 * a4 + m44 * a3;
    float d32 = m41 * a5 + m43 * a2 + m44 * a1;
    float d33 = m41 * -a4 + m42 * a2 + m44 * a0;
    float d34 = m41 * a3 + m42 * -a1 + m43 * a0;

    float d41 = m32 * a5 + m33 * a4 + m34 * a3;
    float d42 = m31 * a5 + m33 * a2 + m34 * a1;
    float d43 = m31 * -a4 + m32 * a2 + m34 * a0;
    float d44 = m31 * a3 + m32 * -a1 + m33 * a0;

    m11 = +d11 * det; m12 = -d21 * det; m13 = +d31 * det; m14 = -d41 * det;
    m21 = -d12 * det; m22 = +d22 * det; m23 = -d32 * det; m24 = +d42 * det;
    m31 = +d13 * det; m32 = -d23 * det; m33 = +d33 * det; m34 = -d43 * det;
    m41 = -d14 * det; m42 = +d24 * det; m43 = -d34 * det; m44 = +d44 * det;
}
