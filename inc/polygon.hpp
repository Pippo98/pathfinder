#ifndef RRT_STAR_POLY_HPP_
#define RRT_STAR_POLY_HPP_

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

	bool operator==(const Rectangle &other) const;
	bool operator!=(const Rectangle &other) const;
};

#endif // RRT_STAR_POLY_HPP_