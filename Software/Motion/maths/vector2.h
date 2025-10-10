#pragma once

#define square(x) ((x)*(x))
#define M_PI      3.14159265358979323846

class Vector2f
{
public:
	Vector2f(float x, float y)
		: m_X(x), m_Y(y)
	{}

	Vector2f(float degrees)
		: m_X(sin((degrees* M_PI) / 180)), m_Y(-cos((degrees * M_PI) / 180))
	{ }

	float GetX() const { return m_X; }
	float GetY() const { return m_Y; }
	float Magnitude() const { return sqrt(square(m_X) + square(m_Y)); }

	static float Multiply(Vector2f& a, Vector2f& b) { return (a.GetX() * b.GetX()) + (a.GetY() * b.GetY()); }

	float operator* (Vector2f& other) { return Multiply(*this, other); }
private:
	float m_X, m_Y;
};