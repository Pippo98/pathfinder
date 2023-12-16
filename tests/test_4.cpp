#include "inc/a_star.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

int main(void) {

    AStar a_star;
    AStarConfig config;
    config.goal_radius = 10;
    config.step_size = 0.6;
    config.max_iterations = 15000;

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

    a_star.set_bounds(Rectangle(Point(0, 0), Point(120, 120)));
    a_star.set_obstacles(obstacles);
    a_star.set_config(config);

    std::vector<AStarNode *> path = a_star.find_path(Point(0, 0), Point(100, 100));

    std::ofstream f_path;
    f_path.open("path.csv");
    f_path << "x,y" << std::endl;
    for (AStarNode *node : path) {
        f_path << node->p.x() << "," << node->p.y() << std::endl;
    }
    f_path.close();

    std::ofstream f_obstacles;
    f_obstacles.open("obstacles.csv");
    for (Polygon obstacle : obstacles) {
        for (Point point : obstacle) {
            f_obstacles << point.x() << "," << point.y() << std::endl;
        }
        f_obstacles << obstacle[0].x() << "," << obstacle[0].y() << std::endl;
        f_obstacles << std::endl;
    }
    f_obstacles.close();


    return EXIT_SUCCESS;
}