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
        Polygon(const Polygon& other);
        Polygon(const std::vector<Point>& points);

        virtual bool contains(const Point& point) const = 0;
};

class Rectangle : public Polygon {
    public:
        Rectangle();
        Rectangle(const Rectangle& other);
        Rectangle(const Point& top_left, const Point& bottom_right);
        Rectangle(const Point& center, double width, double height);

        bool contains(const Point& point) const;
};

#endif // RRT_STAR_POLY_HPP_