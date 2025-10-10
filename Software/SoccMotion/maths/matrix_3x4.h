#pragma once

#include "matrix.h"

class matrix_4x3;
class matrix_3x3;

class matrix_3x4
{
public:
	matrix_3x4(matrix_type value = 1);
	matrix_3x4(matrix_type matrix[3][4]);

	matrix_3x4 operator*(const matrix_3x3& mat3x3);

	static matrix_4x3 transpose(matrix_3x4& matrix);
private:
	matrix_type m_matrix[3][4];
	friend class matrix_4x3;
	friend class matrix_3x3;
};
