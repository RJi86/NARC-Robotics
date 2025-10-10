#pragma once

#include "matrix.h"

class matrix_3x4;
class matrix_3x3;
class vector4;

class matrix_4x3
{
public:
	matrix_4x3(matrix_type value = 1);
	matrix_4x3(matrix_type matrix[4][3]);

	matrix_3x3 operator*(const matrix_3x4& mat_3x4);
	vector4 operator*(const vector4& vec);

	static matrix_3x4 transpose(matrix_4x3& matrix);
private:
	matrix_type m_matrix[4][3];
	friend class matrix_3x4;
	friend class matrix_3x3;
};
