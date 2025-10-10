#pragma once

#include "matrix.h"
#include "vector.h"

class matrix_4x3;
class matrix_3x4;
class vector3;

class matrix_3x3
{
public:
	matrix_3x3(matrix_type value = 1);

	static matrix_3x3 inverse(const matrix_3x3& matrix);
	static matrix_3x3 transpose(const matrix_3x3& matrix);

	matrix_3x3 operator*(const vector_type scalar);

	vector3 operator[](size_t index) const;
private:
	matrix_type m_matrix[3][3];
	friend class matrix_4x3;
	friend class matrix_3x4;
};