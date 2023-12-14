#include "inc/rrt_star.hpp"

#include <stdio.h>

RRTStar::RRTStar() : m_bounds(), m_obstacles(), m_config() {}
RRTStar::RRTStar(const Rectangle& bounds, const std::vector<Polygon>& obstacles, RRTConfig config) : m_bounds(bounds), m_obstacles(obstacles), m_config(config) {}

void RRTStar::set_bounds(const Rectangle& bounds) {
    m_bounds = bounds;
}
void RRTStar::set_obstacles(const std::vector<Polygon>& obstacles) {
    m_obstacles = obstacles;
}
void RRTStar::set_config(RRTConfig config) {
    m_config = config;
}

std::vector<Point> RRTStar::find_path(const Point& start, const Point& goal) {
    std::vector<Point> path;
    path.push_back(start);


    return path;
}

Point RRTStar::m_sample(const Point &current) {
    return Point();
}
Point RRTStar::m_nearest(const Point& sample) {
    ssize_t nearest_index = -1;
    return Point();
}