#include "Definitions.h"

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

	SimpleMatrix(void);
	SimpleMatrix(float value);
	SimpleMatrix(
		float out11, float out12, float out13, float out14,
		float out21, float out22, float out23, float out24,
		float out31, float out32, float out33, float out34,
		float out41, float out42, float out43, float out44);

	~SimpleMatrix(void);

	static SimpleMatrix getRotationX(float angle);
	static SimpleMatrix getRotationY(float angle);
	static SimpleMatrix getRotationZ(float angle);

	SimpleMatrix getInvert();
};

