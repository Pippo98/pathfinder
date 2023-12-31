#include "inc/a_star.hpp"

#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>

int main(void) {

    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::duration<float> fsec;

    AStar a_star;
    AStarConfig config;
    config.step_size = 0.3;
    config.max_iterations = 100000;
    config.g = [](const AStarNode *come_from, const Point &new_sample, const Point &goal) {
        return come_from->g + come_from->p.distance(new_sample);
    };
    config.h = [](const AStarNode *come_from, const Point &new_sample, const Point &goal) {
        return goal.distance(new_sample);
    };

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

    auto t0 = clock::now();
    std::vector<Point> path = a_star.find_path(Point(0, 0), Point(100, 100));
    auto t1 = clock::now();
    ms delta = std::chrono::duration_cast<ms>(t1 - t0);
    printf("Search duration: %lu\n", delta.count());

    std::ofstream f_path;
    f_path.open("path.csv");
    f_path << "x,y" << std::endl;
    for (const Point &point : path) {
        f_path << point.x() << "," << point.y() << std::endl;
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
