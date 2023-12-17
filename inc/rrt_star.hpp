#ifndef PATHFINDER_RRT_STAR_HPP_
#define PATHFINDER_RRT_STAR_HPP_

#include "inc/tree.hpp"
#include "inc/point.hpp"
#include "inc/polygon.hpp"

#include <vector>
#include <functional>

class RRTConfig {
public:
	double step_size;
	double goal_radius;
	int max_iterations;

	std::function<bool(const Point &parent, const Point &child, const Point &new_sample)> validate_new_sample;

	RRTConfig()
			: step_size(0.1), goal_radius(0.1), max_iterations(1000),
				validate_new_sample([](const Point &a, const Point &b, const Point &c) { return true; }) {}
};

class RRTStar {
public:
	RRTStar();
	RRTStar(const Rectangle &bounds, const std::vector<Polygon> &obstacles, RRTConfig config);

	void set_bounds(const Rectangle &bounds);
	void set_obstacles(const std::vector<Polygon> &obstacles);
	void set_config(RRTConfig config);

	std::vector<Node> find_path(const Point &start, const Point &goal);
	Tree tree() const;

private:
	Rectangle m_bounds;
	std::vector<Polygon> m_obstacles;
	RRTConfig m_config;

	Tree m_tree;

	Point m_sample();
	Node &m_nearest_node(const Point &sample);
	Point m_nearest_point(const Point &sample);
	std::vector<Node *> m_near(const Point &sample, double radius);
};

#endif // PATHFINDER_RRT_STAR_HPP_