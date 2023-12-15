#include "inc/polygon.hpp"

#include <cmath>

Polygon::Polygon() : std::vector<Point>() {}
Polygon::Polygon(const Polygon &other) : std::vector<Point>(other) {}
Polygon::Polygon(const std::vector<Point> &points) : std::vector<Point>(points) {}

bool Polygon::contains(const Point &point) const {
	bool result = false;
	for (int i = 0, j = this->size() - 1; i < this->size(); j = i++) {
		if (((this->at(i).y() > point.y()) != (this->at(j).y() > point.y())) &&
				(point.x() <
				 (this->at(j).x() - this->at(i).x()) * (point.y() - this->at(i).y()) / (this->at(j).y() - this->at(i).y()) +
						 this->at(i).x())) {
			result = !result;
		}
	}
	return result;
}
bool Polygon::intersects(const Point &point1, const Point &point2) const {
	for (int i = 0, j = this->size() - 1; i < this->size(); j = i++) {
		if (this->at(i) == point1 || this->at(i) == point2 || this->at(j) == point1 || this->at(j) == point2) {
			continue;
		}
		double x1 = this->at(i).x(), y1 = this->at(i).y();
		double x2 = this->at(j).x(), y2 = this->at(j).y();
		double x3 = point1.x(), y3 = point1.y();
		double x4 = point2.x(), y4 = point2.y();
		double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
		if (std::abs(denom) < 10.0 * std::numeric_limits<double>::epsilon()) {
			continue;
		}
		double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
		double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
		if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
			return true;
		}
	}
	return false;
}

Rectangle::Rectangle() : Polygon() {
	this->push_back(Point());
	this->push_back(Point());
	this->push_back(Point());
	this->push_back(Point());
}
Rectangle::Rectangle(const Rectangle &other) : Polygon(other) {}
Rectangle::Rectangle(const Point &corner1, const Point &corner2) : Polygon() {
	Point center = (corner1 + corner2) / 2;
	double width = std::abs(corner1.x() - corner2.x());
	double height = std::abs(corner1.y() - corner2.y());
	this->push_back(Point(center.x() - width / 2, center.y() + height / 2));
	this->push_back(Point(center.x() + width / 2, center.y() + height / 2));
	this->push_back(Point(center.x() + width / 2, center.y() - height / 2));
	this->push_back(Point(center.x() - width / 2, center.y() - height / 2));
}
Rectangle::Rectangle(const Point &center, double width, double height) : Polygon() {
	this->push_back(Point(center.x() - width / 2, center.y() + height / 2));
	this->push_back(Point(center.x() + width / 2, center.y() + height / 2));
	this->push_back(Point(center.x() + width / 2, center.y() - height / 2));
	this->push_back(Point(center.x() - width / 2, center.y() - height / 2));
}

bool Rectangle::contains(const Point &point) const {
	return (point.x() >= this->at(0).x() && point.x() <= this->at(1).x() && point.y() >= this->at(3).y() &&
					point.y() <= this->at(0).y());
}
bool Rectangle::contains(const Rectangle &other) const {
	return (this->contains(other.at(0)) && this->contains(other.at(1)) && this->contains(other.at(2)) &&
					this->contains(other.at(3)));
}

Point Rectangle::center() const { return (this->at(0) + this->at(1) + this->at(2) + this->at(3)) / 4; }
double Rectangle::width() const { return std::abs(this->at(0).x() - this->at(1).x()); }
double Rectangle::height() const { return std::abs(this->at(0).y() - this->at(3).y()); }

bool Rectangle::operator==(const Rectangle &other) const {
	return (this->at(0) == other.at(0) && this->at(1) == other.at(1) && this->at(2) == other.at(2) &&
					this->at(3) == other.at(3));
}
bool Rectangle::operator!=(const Rectangle &other) const { return !(*this == other); }