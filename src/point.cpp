#include "inc/point.hpp"

#include <cmath>

static inline bool is_zero(double value) { return std::abs(value) < 10.0 * std::numeric_limits<double>::epsilon(); }

Point::Point() : m_x(0.0), m_y(0.0) {}
Point::Point(double xy) : m_x(xy), m_y(xy) {}
Point::Point(double x, double y) : m_x(x), m_y(y) {}
Point::Point(const Point &other) : m_x(other.x()), m_y(other.y()) {}

double Point::x() const { return m_x; }
double Point::y() const { return m_y; }

Point &Point::set_x(double x) {
	this->m_x = x;
	return *this;
}
Point &Point::set_y(double y) {
	this->m_y = y;
	return *this;
}

double Point::distance(const Point &other) const {
	return std::sqrt(std::pow(m_x - other.x(), 2) + std::pow(m_y - other.y(), 2));
}
Point &Point::normalize() {
	double length = this->distance(Point());
	if (is_zero(length)) {
		return *this;
	}
	m_x /= length;
	m_y /= length;
	return *this;
}

bool Point::operator==(const Point &other) const { return is_zero(m_x - other.x()) && is_zero(m_y - other.y()); }
bool Point::operator!=(const Point &other) const { return !(*this == other); }
Point Point::operator+(const Point &other) const { return Point(m_x + other.x(), m_y + other.y()); }
Point Point::operator-(const Point &other) const { return Point(m_x - other.x(), m_y - other.y()); }
Point Point::operator*(const Point &other) const { return Point(m_x * other.x(), m_y * other.y()); }
Point Point::operator/(const Point &other) const { return Point(m_x / other.x(), m_y / other.y()); }
Point Point::operator*(double scalar) const { return Point(m_x * scalar, m_y * scalar); }
Point Point::operator/(double scalar) const { return Point(m_x / scalar, m_y / scalar); }
Point &Point::operator+=(const Point &other) {
	m_x += other.x();
	m_y += other.y();
	return *this;
}
Point &Point::operator-=(const Point &other) {
	m_x -= other.x();
	m_y -= other.y();
	return *this;
}
Point &Point::operator*=(const Point &other) {
	m_x *= other.x();
	m_y *= other.y();
	return *this;
}
Point &Point::operator/=(const Point &other) {
	m_x /= other.x();
	m_y /= other.y();
	return *this;
}
Point &Point::operator*=(double scalar) {
	m_x *= scalar;
	m_y *= scalar;
	return *this;
}
Point &Point::operator/=(double scalar) {
	m_x /= scalar;
	m_y /= scalar;
	return *this;
}

std::ostream &operator<<(std::ostream &os, const Point &point) {
	os << "(" << point.x() << ", " << point.y() << ")";
	return os;
}