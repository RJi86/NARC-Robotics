#pragma once

#include "vector.h"

class vector3
{
public:
	vector3(vector_type scalar = 0);
	vector3(vector_type vec[3]);
	vector3(vector_type x, vector_type y, vector_type z);

	vector3 operator+(vector3 vec) const;
	vector3 operator*(vector_type scalar) const;

	vector_type operator[](int index);
private:
	vector_type m_vec[3] = { 0, 0, 0 };
};