#include "inc/rrt_star.hpp"

#include <stdio.h>
#include <fstream>
#include <iostream>

void write_node(std::ofstream &f, Node *node) {
    for (Node *child : node->children()) {
        f << node->point().x() << "," << node->point().y() << std::endl;
        write_node(f, child);
    }
}

int main(void) {

    RRTStar rrt_star;
    RRTConfig config;
    config.step_size = 1;
    config.goal_radius = 10;
    config.max_iterations = 400000;

    rrt_star.set_bounds(Rectangle(Point(0, 0), Point(120, 120)));
    rrt_star.set_obstacles(std::vector<Polygon>({
        Rectangle(Point(20, 20), Point(40, 40)),
        Rectangle(Point(80, 80), Point(90, 90)),
        Rectangle(Point(20, 40), Point(60, 55)),
        Rectangle(Point(80, 100), Point(90, 20)),
    }));
    rrt_star.set_config(config);

    std::vector<Node> path = rrt_star.find_path(Point(0, 0), Point(100, 100));

    
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

    return 0;
}