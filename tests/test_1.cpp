#include "inc/rrt_star.hpp"

#include <stdio.h>
#include <iostream>
#include <limits>

int main(void) {
    printf("RRT* test 1\n");

    Point p1(1.0, 1.0);
    std::cout << "p1: " << p1 << std::endl;
    Point p2(2.0, 2.0);
    std::cout << "p2: " << p2 << std::endl;
    Point p3 = p1 + p2;
    std::cout << "p3(p1+p2):" << p3 << std::endl;
    p3 = p1 - p2;
    std::cout << "p1 - p2: " << p3 << std::endl;
    p3 += p1;
    std::cout << "p3 += p1: " << p1 << std::endl;
    Point p4 = p2 * 2.1;

    Rectangle r1(Point(), p2);
    std::cout << "p1 in r1: " << r1.contains(p1) << std::endl;
    std::cout << "p4 in r1: " << r1.contains(p4) << std::endl;

    Point p5(1.0, 1.0);
    std::cout << "p5 == p5: " << (p1 == p5) << std::endl;
    std::cout << "p5 != p5: " << (p1 != p5) << std::endl;
    std::cout << "p5 == p5 + epsilon: " << (p1 == p5 + Point(std::numeric_limits<double>::epsilon())) << std::endl;
    std::cout << "p5 == p5 + epsilon: " << (p1 == p5 + Point(std::numeric_limits<double>::epsilon())*10) << std::endl;

    return 0;
}