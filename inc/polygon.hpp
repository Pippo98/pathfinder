#ifndef PATHFINDER_POLY_HPP_
#define PATHFINDER_POLY_HPP_

#include <vector>
#include "inc/point.hpp"

/**
 * @brief Polygon class
 */
class Polygon : public std::vector<Point> {
public:
	Polygon();
	Polygon(const Polygon &other);
	Polygon(const std::vector<Point> &points);

	Polygon &operator=(const Polygon &other) = default;
	virtual bool contains(const Point &point) const;
	virtual bool intersects(const Point &point1, const Point &point2) const;
};

class Rectangle : public Polygon {
public:
	Rectangle();
	Rectangle(const Rectangle &other);
	Rectangle(const Point &corner1, const Point &corner2);
	Rectangle(const Point &center, double width, double height);

	bool contains(const Point &point) const;
	bool contains(const Rectangle &other) const;

	Point center() const;
	double width() const;
	double height() const;

	Rectangle &operator=(const Rectangle &other) = default;
	bool operator==(const Rectangle &other) const;
	bool operator!=(const Rectangle &other) const;
};

#endif // PATHFINDER_POLY_HPP_