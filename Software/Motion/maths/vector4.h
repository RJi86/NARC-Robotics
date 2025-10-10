#pragma once

#include "vector.h"

class vector3;

class vector4
{
public:
	vector4(vector_type scalar = 0);
	vector4(vector_type vec[4]);
	vector4(vector3 vec3, vector_type w = 0);
	vector4(vector_type x, vector_type y, vector_type z, vector_type w);

	auto x() const -> vector_type;
	auto y() const -> vector_type;
	auto z() const -> vector_type;
	auto w() const -> vector_type;

	static vector4 cross(const vector4& x, const vector4& y);
	static vector_type dot(const vector4& a, const vector4& b);

	vector_type& operator[](size_t index);
private:
	vector_type m_vec[4] = { 0, 0, 0, 0 };
};