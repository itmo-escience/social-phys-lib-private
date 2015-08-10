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

	~SimpleMatrix(void);

	static SimpleMatrix getRotationX(float angle);
	void getInvert();
};

