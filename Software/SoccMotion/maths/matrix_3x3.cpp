#include "matrix_3x3.h"

#include "vector3.h"
#include "vector4.h"

matrix_3x3::matrix_3x3(matrix_type value)
{
	m_matrix[0][0] = value; m_matrix[0][1] = 0;		m_matrix[0][2] = 0;
	m_matrix[1][0] = 0;		m_matrix[1][1] = value; m_matrix[1][2] = 0;
	m_matrix[2][0] = 0;		m_matrix[2][1] = 0;		m_matrix[2][2] = value;
}

matrix_3x3 matrix_3x3::inverse(const matrix_3x3& matrix)
{
	vector4 a(matrix[0]);
	vector4 b(matrix[1]);
	vector4 c(matrix[2]);

	auto determinant = vector4::dot(a, vector4::cross(b, c));
	auto one_over_determinant = static_cast<vector_type>(1) / determinant;

	matrix_3x3 result(0);

	result.m_matrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
	result.m_matrix[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]);
	result.m_matrix[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]);

	result.m_matrix[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]);
	result.m_matrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]);
	result.m_matrix[1][2] = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]);

	result.m_matrix[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
	result.m_matrix[2][1] = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]);
	result.m_matrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

	return result * one_over_determinant;
}

matrix_3x3 matrix_3x3::transpose(const matrix_3x3& matrix)
{
	matrix_3x3 result;
	result.m_matrix[0][0] = matrix.m_matrix[0][0];
	result.m_matrix[0][1] = matrix.m_matrix[1][0];
	result.m_matrix[0][2] = matrix.m_matrix[2][0];

	result.m_matrix[1][0] = matrix.m_matrix[0][1];
	result.m_matrix[1][1] = matrix.m_matrix[1][1];
	result.m_matrix[1][2] = matrix.m_matrix[2][1];

	result.m_matrix[2][0] = matrix.m_matrix[0][2];
	result.m_matrix[2][1] = matrix.m_matrix[1][2];
	result.m_matrix[2][2] = matrix.m_matrix[2][2];
	return result;
}

matrix_3x3 matrix_3x3::operator*(const vector_type scalar)
{
	matrix_3x3 result;
	result.m_matrix[0][0] = m_matrix[0][0] * scalar; result.m_matrix[0][1] = m_matrix[0][1] * scalar; result.m_matrix[0][2] = m_matrix[0][2] * scalar;
	result.m_matrix[1][0] = m_matrix[1][0] * scalar; result.m_matrix[1][1] = m_matrix[1][1] * scalar; result.m_matrix[1][2] = m_matrix[1][2] * scalar;
	result.m_matrix[2][0] = m_matrix[2][0] * scalar; result.m_matrix[2][1] = m_matrix[2][1] * scalar; result.m_matrix[2][2] = m_matrix[2][2] * scalar;
	return result;
}

vector3 matrix_3x3::operator[](size_t index) const
{
	return { m_matrix[index][0], m_matrix[index][1], m_matrix[index][2] };
}
