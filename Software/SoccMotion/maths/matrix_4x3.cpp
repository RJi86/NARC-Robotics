#include "matrix_4x3.h"

#include "matrix_3x4.h"
#include "matrix_3x3.h"
#include "vector3.h"
#include "vector4.h"

matrix_4x3::matrix_4x3(matrix_type value)
{
	m_matrix[0][0] = value; m_matrix[0][1] = 0;		m_matrix[0][2] = 0;
	m_matrix[1][0] = 0;		m_matrix[1][1] = value; m_matrix[1][2] = 0;
	m_matrix[2][0] = 0;		m_matrix[2][1] = 0;		m_matrix[2][2] = value;
	m_matrix[3][0] = 0;		m_matrix[3][1] = 0;		m_matrix[3][2] = 0;
}

matrix_4x3::matrix_4x3(matrix_type matrix[4][3])
{
	for (size_t row = 0; row < 4; row++)
	{
		for (size_t col = 0; col < 3; col++)
		{
			m_matrix[row][col] = matrix[row][col];
		}
	}
}

matrix_3x3 matrix_4x3::operator*(const matrix_3x4& mat_3x4)
{
	const matrix_type src_4x3_00 = m_matrix[0][0];
	const matrix_type src_4x3_01 = m_matrix[0][1];
	const matrix_type src_4x3_02 = m_matrix[0][2];
	const matrix_type src_4x3_10 = m_matrix[1][0];
	const matrix_type src_4x3_11 = m_matrix[1][1];
	const matrix_type src_4x3_12 = m_matrix[1][2];
	const matrix_type src_4x3_20 = m_matrix[2][0];
	const matrix_type src_4x3_21 = m_matrix[2][1];
	const matrix_type src_4x3_22 = m_matrix[2][2];
	const matrix_type src_4x3_30 = m_matrix[3][0];
	const matrix_type src_4x3_31 = m_matrix[3][1];
	const matrix_type src_4x3_32 = m_matrix[3][2];

	const matrix_type src_3x4_00 = mat_3x4.m_matrix[0][0];
	const matrix_type src_3x4_01 = mat_3x4.m_matrix[0][1];
	const matrix_type src_3x4_02 = mat_3x4.m_matrix[0][2];
	const matrix_type src_3x4_03 = mat_3x4.m_matrix[0][3];
	const matrix_type src_3x4_10 = mat_3x4.m_matrix[1][0];
	const matrix_type src_3x4_11 = mat_3x4.m_matrix[1][1];
	const matrix_type src_3x4_12 = mat_3x4.m_matrix[1][2];
	const matrix_type src_3x4_13 = mat_3x4.m_matrix[1][3];
	const matrix_type src_3x4_20 = mat_3x4.m_matrix[2][0];
	const matrix_type src_3x4_21 = mat_3x4.m_matrix[2][1];
	const matrix_type src_3x4_22 = mat_3x4.m_matrix[2][2];
	const matrix_type src_3x4_23 = mat_3x4.m_matrix[2][3];

	matrix_3x3 result;
	result.m_matrix[0][0] = src_4x3_00 * src_3x4_00 + src_4x3_10 * src_3x4_01 + src_4x3_20 * src_3x4_02 + src_4x3_30 * src_3x4_03;
	result.m_matrix[0][1] = src_4x3_01 * src_3x4_00 + src_4x3_11 * src_3x4_01 + src_4x3_21 * src_3x4_02 + src_4x3_31 * src_3x4_03;
	result.m_matrix[0][2] = src_4x3_02 * src_3x4_00 + src_4x3_12 * src_3x4_01 + src_4x3_22 * src_3x4_02 + src_4x3_32 * src_3x4_03;
	result.m_matrix[1][0] = src_4x3_00 * src_3x4_10 + src_4x3_10 * src_3x4_11 + src_4x3_20 * src_3x4_12 + src_4x3_30 * src_3x4_13;
	result.m_matrix[1][1] = src_4x3_01 * src_3x4_10 + src_4x3_11 * src_3x4_11 + src_4x3_21 * src_3x4_12 + src_4x3_31 * src_3x4_13;
	result.m_matrix[1][2] = src_4x3_02 * src_3x4_10 + src_4x3_12 * src_3x4_11 + src_4x3_22 * src_3x4_12 + src_4x3_32 * src_3x4_13;
	result.m_matrix[2][0] = src_4x3_00 * src_3x4_20 + src_4x3_10 * src_3x4_21 + src_4x3_20 * src_3x4_22 + src_4x3_30 * src_3x4_23;
	result.m_matrix[2][1] = src_4x3_01 * src_3x4_20 + src_4x3_11 * src_3x4_21 + src_4x3_21 * src_3x4_22 + src_4x3_31 * src_3x4_23;
	result.m_matrix[2][2] = src_4x3_02 * src_3x4_20 + src_4x3_12 * src_3x4_21 + src_4x3_22 * src_3x4_22 + src_4x3_32 * src_3x4_23;
	return result;
}

vector4 matrix_4x3::operator*(const vector4& vec)
{
	vector4 result(
		vec.x() * m_matrix[0][0] + vec.y() * m_matrix[0][1] + vec.z() * m_matrix[0][2],
		vec.x() * m_matrix[1][0] + vec.y() * m_matrix[1][1] + vec.z() * m_matrix[1][2],
		vec.x() * m_matrix[2][0] + vec.y() * m_matrix[2][1] + vec.z() * m_matrix[2][2],
		vec.x() * m_matrix[3][0] + vec.y() * m_matrix[3][1] + vec.z() * m_matrix[3][2]);
	return result;
}

matrix_3x4 matrix_4x3::transpose(matrix_4x3& matrix)
{
	matrix_3x4 result(1.0f);
	result.m_matrix[0][0] = matrix.m_matrix[0][0];
	result.m_matrix[0][1] = matrix.m_matrix[1][0];
	result.m_matrix[0][2] = matrix.m_matrix[2][0];
	result.m_matrix[0][3] = matrix.m_matrix[3][0];
	result.m_matrix[1][0] = matrix.m_matrix[0][1];
	result.m_matrix[1][1] = matrix.m_matrix[1][1];
	result.m_matrix[1][2] = matrix.m_matrix[2][1];
	result.m_matrix[1][3] = matrix.m_matrix[3][1];
	result.m_matrix[2][0] = matrix.m_matrix[0][2];
	result.m_matrix[2][1] = matrix.m_matrix[1][2];
	result.m_matrix[2][2] = matrix.m_matrix[2][2];
	result.m_matrix[2][3] = matrix.m_matrix[3][2];
	return result;
}