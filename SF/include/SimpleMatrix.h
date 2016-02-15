#include "Definitions.h"

/// <summary> Defines a matrix in the simulation </summary>
class SimpleMatrix
{
public:
    float 
        m11,
        m12,
        m13,
        m14,
        m21,
        m22,
        m23,
        m24,
        m31,
        m32,
        m33,
        m34,
        m41,
        m42,
        m43,
        m44;

	/// <summary> Sets an identity matrix </summary>
    SimpleMatrix(void);

	/// <summary> Sets a determined matrix </summary>
    SimpleMatrix(float value);

	/// <summary> Sets a determined matrix </summary>
    SimpleMatrix(
        float out11, float out12, float out13, float out14,
        float out21, float out22, float out23, float out24,
        float out31, float out32, float out33, float out34,
        float out41, float out42, float out43, float out44);

	/// <summary> Destructor </summary>
    ~SimpleMatrix(void);

	/// <summary> Gets an inverted matrix </summary>
	/// <returns> Inverted matrix </returns>
    SimpleMatrix getInvert();
};

