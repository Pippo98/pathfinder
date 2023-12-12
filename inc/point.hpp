#ifndef RRT_STAR_POINT_HPP_
#define RRT_STAR_POINT_HPP_

#include <iostream>

class Point {
    public:
        Point();
        Point(double xy);
        Point(double x, double y);
        Point(const Point& other);

        double x() const;
        double y() const;

        Point& set_x(double x);
        Point& set_y(double y);

        double distance(const Point& other) const;

        bool operator==(const Point& other) const;
        bool operator!=(const Point& other) const;
        Point operator+(const Point& other) const;
        Point operator-(const Point& other) const;
        Point operator*(const Point& other) const;
        Point operator/(const Point& other) const;
        Point operator*(double scalar) const;
        Point operator/(double scalar) const;
        Point& operator+=(const Point& other);
        Point& operator-=(const Point& other);
        Point& operator*=(const Point& other);
        Point& operator/=(const Point& other);
        Point& operator*=(double scalar);
        Point& operator/=(double scalar);

        friend std::ostream& operator<<(std::ostream& os, const Point& point);

    private:
        double m_x;
        double m_y;
};

#endif // RRT_STAR_POINT_HPP_