#include "inc/polygon.hpp"

#include <cmath>

Polygon::Polygon() : std::vector<Point>() {}
Polygon::Polygon(const Polygon& other) : std::vector<Point>(other) {}
Polygon::Polygon(const std::vector<Point>& points) : std::vector<Point>(points) {}

Rectangle::Rectangle() : Polygon() {
    this->push_back(Point());
    this->push_back(Point());
    this->push_back(Point());
    this->push_back(Point());
}
Rectangle::Rectangle(const Rectangle& other) : Polygon(other) {}
Rectangle::Rectangle(const Point& top_left, const Point& bottom_right) : Polygon() {
    this->push_back(top_left);
    this->push_back(Point(bottom_right.x(), top_left.y()));
    this->push_back(bottom_right);
    this->push_back(Point(top_left.x(), bottom_right.y()));
}
Rectangle::Rectangle(const Point& center, double width, double height) : Polygon() {
    this->push_back(Point(center.x() - width / 2, center.y() + height / 2));
    this->push_back(Point(center.x() + width / 2, center.y() + height / 2));
    this->push_back(Point(center.x() + width / 2, center.y() - height / 2));
    this->push_back(Point(center.x() - width / 2, center.y() - height / 2));
}

bool Rectangle::contains(const Point& point) const {
    return (point.x() >= this->at(0).x() && point.x() <= this->at(1).x() &&
            point.y() >= this->at(3).y() && point.y() <= this->at(0).y());
}