#ifndef RRT_STAR_HPP_
#define RRT_STAR_HPP_

#include "inc/point.hpp"
#include "inc/polygon.hpp"

#include <vector>
#include <functional>

class RRTConfig {
public:
    double step_size;
    double goal_radius;
    int max_iterations;

    std::function<bool(const Point&, const Point&)> validate_new_sample;

    RRTConfig() : step_size(0.1), goal_radius(0.1), max_iterations(1000), validate_new_sample([](const Point& a, const Point& b) { return true; }) {}
};

class RRTStar {
    public:
        RRTStar();
        RRTStar(const Rectangle& bounds, const std::vector<Polygon>& obstacles, RRTConfig config);

        void set_bounds(const Rectangle& bounds);
        void set_obstacles(const std::vector<Polygon>& obstacles);
        void set_config(RRTConfig config);

        std::vector<Point> find_path(const Point& start, const Point& goal);

    private:
        Rectangle m_bounds;
        std::vector<Polygon> m_obstacles;
        RRTConfig m_config;

        Point m_sample(const Point &current);
        Point m_nearest(const Point& sample);
};

#endif // RRT_STAR_HPP_