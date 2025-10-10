#include "matrix_3x4.h"

#include "matrix_4x3.h"
#include "matrix_3x3.h"
#include "vector3.h"

matrix_3x4::matrix_3x4(matrix_type value)
{
	m_matrix[0][0] = value; m_matrix[0][1] = 0;		m_matrix[0][2] = 0;		m_matrix[0][3] = 0;
	m_matrix[1][0] = 0;		m_matrix[1][1] = value; m_matrix[1][2] = 0;		m_matrix[1][3] = 0;
	m_matrix[2][0] = 0;		m_matrix[2][1] = 0;		m_matrix[2][2] = value; m_matrix[2][3] = 0;
}

matrix_3x4::matrix_3x4(matrix_type matrix[3][4])
{
	for (size_t row = 0; row < 3; row++)
	{
		for (size_t col = 0; col < 4; col++)
		{
			m_matrix[row][col] = matrix[row][col];
		}
	}
}

matrix_3x4 matrix_3x4::operator*(const matrix_3x3& mat3x3)
{
	matrix_type m[3][4] =
	{
		{
			m_matrix[0][0] * mat3x3[0][0] + m_matrix[1][0] * mat3x3[0][1] + m_matrix[2][0] * mat3x3[0][2],
			m_matrix[0][1] * mat3x3[0][0] + m_matrix[1][1] * mat3x3[0][1] + m_matrix[2][1] * mat3x3[0][2],
			m_matrix[0][2] * mat3x3[0][0] + m_matrix[1][2] * mat3x3[0][1] + m_matrix[2][2] * mat3x3[0][2],
			m_matrix[0][3] * mat3x3[0][0] + m_matrix[1][3] * mat3x3[0][1] + m_matrix[2][3] * mat3x3[0][2]
		},
		{
			m_matrix[0][0] * mat3x3[1][0] + m_matrix[1][0] * mat3x3[1][1] + m_matrix[2][0] * mat3x3[1][2],
			m_matrix[0][1] * mat3x3[1][0] + m_matrix[1][1] * mat3x3[1][1] + m_matrix[2][1] * mat3x3[1][2],
			m_matrix[0][2] * mat3x3[1][0] + m_matrix[1][2] * mat3x3[1][1] + m_matrix[2][2] * mat3x3[1][2],
			m_matrix[0][3] * mat3x3[1][0] + m_matrix[1][3] * mat3x3[1][1] + m_matrix[2][3] * mat3x3[1][2]
		},
		{
			m_matrix[0][0] * mat3x3[2][0] + m_matrix[1][0] * mat3x3[2][1] + m_matrix[2][0] * mat3x3[2][2],
			m_matrix[0][1] * mat3x3[2][0] + m_matrix[1][1] * mat3x3[2][1] + m_matrix[2][1] * mat3x3[2][2],
			m_matrix[0][2] * mat3x3[2][0] + m_matrix[1][2] * mat3x3[2][1] + m_matrix[2][2] * mat3x3[2][2],
			m_matrix[0][3] * mat3x3[2][0] + m_matrix[1][3] * mat3x3[2][1] + m_matrix[2][3] * mat3x3[2][2]
		}
	};
	return matrix_3x4(m);
}

matrix_4x3 matrix_3x4::transpose(matrix_3x4& matrix)
{
	matrix_4x3 result(1);
	result.m_matrix[0][0] = matrix.m_matrix[0][0];
	result.m_matrix[0][1] = matrix.m_matrix[1][0];
	result.m_matrix[0][2] = matrix.m_matrix[2][0];
	result.m_matrix[1][0] = matrix.m_matrix[0][1];
	result.m_matrix[1][1] = matrix.m_matrix[1][1];
	result.m_matrix[1][2] = matrix.m_matrix[2][1];
	result.m_matrix[2][0] = matrix.m_matrix[0][2];
	result.m_matrix[2][1] = matrix.m_matrix[1][2];
	result.m_matrix[2][2] = matrix.m_matrix[2][2];
	result.m_matrix[3][0] = matrix.m_matrix[0][3];
	result.m_matrix[3][1] = matrix.m_matrix[1][3];
	result.m_matrix[3][2] = matrix.m_matrix[2][3];
	return result;
}