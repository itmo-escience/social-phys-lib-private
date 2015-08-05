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
}