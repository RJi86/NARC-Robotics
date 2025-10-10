#include "vector4.h"

#include "vector3.h"

vector4::vector4(vector_type scalar)
{
    m_vec[0] = scalar;
    m_vec[1] = scalar;
    m_vec[2] = scalar;
    m_vec[3] = scalar;
}

vector4::vector4(vector_type vec[4])
{
    m_vec[0] = vec[0];
    m_vec[1] = vec[1];
    m_vec[2] = vec[2];
    m_vec[3] = vec[3];
}

vector4::vector4(vector3 vec3, vector_type w)
{
    m_vec[0] = vec3[0];
    m_vec[1] = vec3[1];
    m_vec[2] = vec3[2];
    m_vec[3] = w;
}

vector4::vector4(vector_type x, vector_type y, vector_type z, vector_type w)
{
    m_vec[0] = x;
    m_vec[1] = y;
    m_vec[2] = z;
    m_vec[3] = w;
}

auto vector4::x() const -> vector_type
{
    return m_vec[0];
}

auto vector4::y() const -> vector_type
{
    return m_vec[1];
}

auto vector4::z() const -> vector_type
{
    return m_vec[2];
}

auto vector4::w() const -> vector_type
{
    return m_vec[3];
}

vector4 vector4::cross(const vector4& x, const vector4& y)
{
    return vector4(
        x.y() * y.z() - y.y() * x.z(),
        x.z() * y.x() - y.z() * x.x(),
        x.x() * y.y() - y.x() * x.y(),
        0.0f);
}

vector_type vector4::dot(const vector4& a, const vector4& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w();
}

vector_type& vector4::operator[](const size_t index)
{
    return m_vec[index];
}
