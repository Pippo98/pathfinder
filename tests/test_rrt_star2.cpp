#include "inc/rrt_star2.hpp"

#include <cmath>
#include <chrono>
#include <stdio.h>
#include <fstream>
#include <iostream>

void write_node(std::ofstream &f, Node *node) {
    for (Node *child : node->children()) {
        f << node->point().x() << "," << node->point().y() << std::endl;
        write_node(f, child);
    }
}

bool validate_new_sample(const Point &a, const Point &b, const Point &c) {
    double prev_angle = std::atan2(b.y() - a.y(), b.x() - a.x());
    double new_angle = std::atan2(c.y() - b.y(), c.x() - b.x());
    double angle_diff = std::abs(prev_angle - new_angle);

    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
    }

    if (angle_diff > M_PI / 3) {
        return false;
    }
    return true;
}

int main(void) {
  
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::duration<float> fsec;


    RRTStar2 rrt_star;
    RRTConfig config;
    config.step_size = 0.5;
    config.goal_radius = 10;
    config.max_iterations = 15000;
    config.validate_new_sample = validate_new_sample;

    auto obstacles = std::vector<Polygon>({
        Rectangle(Point(20, 20), Point(40, 40)),
        Rectangle(Point(80, 80), Point(90, 90)),
        Rectangle(Point(20, 40), Point(60, 55)),
        Rectangle(Point(80, 100), Point(90, 20)),
        Polygon({
            Point(20, 20),
            Point(30, 20),
            Point(35, 30),
            Point(30, 40),
            Point(20, 40),
            Point(15, 30)
        })
    });

    rrt_star.set_bounds(Rectangle(Point(0, 0), Point(120, 120)));
    rrt_star.set_obstacles(obstacles);
    rrt_star.set_config(config);
    
    auto t0 = clock::now();
    std::vector<Node> path = rrt_star.find_path(Point(0, 0), Point(100, 100));
    auto t1 = clock::now();
    ms delta = std::chrono::duration_cast<ms>(t1 - t0);
    printf("Search duration: %ld\n", delta.count());
    
    std::ofstream f_path;
    f_path.open("path.csv");
    f_path << "x,y" << std::endl;
    for (Node node : path) {
        f_path << node.point().x() << "," << node.point().y() << std::endl;
    }

    std::ofstream f_tree;
    f_tree.open("tree.csv");
    f_tree << "x,y" << std::endl;
    for (Node *node : rrt_star.tree().nodes()) {
        f_tree << node->point().x() << "," << node->point().y() << std::endl;
        write_node(f_tree, node);
    }

    std::ofstream f_obstacles;
    f_obstacles.open("obstacles.csv");
    for (Polygon obstacle : obstacles) {
        for (Point point : obstacle) {
            f_obstacles << point.x() << "," << point.y() << std::endl;
        }
        f_obstacles << obstacle[0].x() << "," << obstacle[0].y() << std::endl;
        f_obstacles << std::endl;
    }

    return 0;
}
