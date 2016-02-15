#include "../include/SimpleMatrix.h"

/// <summary> Sets an identity matrix </summary>
SimpleMatrix::SimpleMatrix(void)
{
    m11 = m22 = m33 = m44 = 1;
    m12 = m13 = m14 =
    m21 = m23 = m24 =
    m31 = m32 = m34 =
    m41 = m42 = m43 = 0.0f;
}

/// <summary> Sets a determined matrix </summary>
SimpleMatrix::SimpleMatrix(float value)
{
    m11 = m12 = m13 = m14 =
    m21 = m22 = m23 = m24 =
    m31 = m32 = m33 = m34 =
    m41 = m42 = m43 = m44 = value;
}

/// <summary> Sets a determined matrix </summary>
SimpleMatrix::SimpleMatrix(
        float out11, float out12, float out13, float out14,
        float out21, float out22, float out23, float out24,
        float out31, float out32, float out33, float out34,
        float out41, float out42, float out43, float out44)
{
    m11 = out11; m12 = out12; m13 = out13; m14 = out14; 
    m21 = out21; m22 = out22; m23 = out23; m24 = out24; 
    m31 = out31; m32 = out32; m33 = out33; m34 = out34; 
    m41 = out41; m42 = out42; m43 = out43; m44 = out44; 
}

/// <summary> Destructor </summary>
SimpleMatrix::~SimpleMatrix(void)
{
}

/// <summary> Gets an inverted matrix </summary>
/// <returns> Inverted matrix </returns>
SimpleMatrix SimpleMatrix::getInvert()
{
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
        return SimpleMatrix(0.0f);

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


    return SimpleMatrix(
        d11 * det, -d21 * det, d31 * det, -d41 * det,
        -d12 * det, d22 * det, -d32 * det, d42 * det,
        d13 * det, -d23 * det, d33 * det, -d43 * det,
        -d14 * det, d24 * det, -d34 * det, d44 * det);
}
