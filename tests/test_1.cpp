#include "inc/rrt_star.hpp"

#include <stdio.h>

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

    return 0;
}