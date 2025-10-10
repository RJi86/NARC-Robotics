#include "vector3.h"

vector3::vector3(vector_type scalar)
{
    m_vec[0] = scalar;
    m_vec[1] = scalar;
    m_vec[2] = scalar;
}

vector3::vector3(vector_type vec[3])
{
    m_vec[0] = vec[0];
    m_vec[1] = vec[1];
    m_vec[2] = vec[2];
}

vector3::vector3(vector_type x, vector_type y, vector_type z)
{
    m_vec[0] = x;
    m_vec[1] = y;
    m_vec[2] = z;
}

vector3 vector3::operator+(vector3 vec) const
{
    vector3 result;
    result.m_vec[0] = m_vec[0] + vec.m_vec[0];
    result.m_vec[1] = m_vec[1] + vec.m_vec[1];
    result.m_vec[2] = m_vec[2] + vec.m_vec[2];
    return result;
}

vector3 vector3::operator*(vector_type scalar) const
{
    vector3 result;
    result.m_vec[0] = m_vec[0] * scalar;
    result.m_vec[1] = m_vec[1] * scalar;
    result.m_vec[2] = m_vec[2] * scalar;
    return result;
}

vector_type vector3::operator[](int index)
{
    return m_vec[index];
}
