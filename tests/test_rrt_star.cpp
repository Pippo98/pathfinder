#include "inc/rrt_star.hpp"

#include <chrono>
#include <fstream>
#include <iostream>

int main(void) {
  
  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::duration<float> fsec;

  RRTStar rrt_star;
  RRTStarConfig rrt_star_config;

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
  rrt_star_config.goal_radius = 10.0;
  rrt_star_config.smoothen_iterations = 100;
  rrt_star_config.step_size = 0.5;
  rrt_star_config.max_iterations = 15000;

  rrt_star.setObstacles(obstacles);
  rrt_star.setConfig(rrt_star_config);
  rrt_star.setBounds(Rectangle(Point(0, 0), Point(120, 120)));

  auto t0 = clock::now();
  auto path = rrt_star.findPath(Point(0,0), Point(100, 100));
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

  std::ofstream f_tree;
  f_tree.open("tree.csv");
  f_tree << "x,y" << std::endl;
  RRTStarTree::recursiveIterator(rrt_star.tree().getRoot(), [&f_tree](RRTStarNode *node){
      f_tree << node->p().x() << "," << node->p().y() << std::endl;
  });

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
