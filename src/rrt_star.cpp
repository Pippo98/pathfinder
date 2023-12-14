#include "inc/rrt_star.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <random>

RRTStar::RRTStar() : m_bounds(), m_obstacles(), m_config() {}
RRTStar::RRTStar(const Rectangle &bounds, const std::vector<Polygon> &obstacles, RRTConfig config)
		: m_bounds(bounds), m_obstacles(obstacles), m_config(config) {}

void RRTStar::set_bounds(const Rectangle &bounds) { m_bounds = bounds; }
void RRTStar::set_obstacles(const std::vector<Polygon> &obstacles) { m_obstacles = obstacles; }
void RRTStar::set_config(RRTConfig config) { m_config = config; }

bool PointInObstacle(const Point &point, const std::vector<Polygon> &obstacles) {
	for (const Polygon &obstacle : obstacles) {
		if (obstacle.contains(point)) {
			return true;
		}
	}
	return false;
}

std::vector<Node> RRTStar::find_path(const Point &start, const Point &goal) {
	std::vector<Node> path;
	m_tree = Tree(start);

	for (int i = 0; i < m_config.max_iterations; i++) {
		Point sample = m_sample();
		Point nearest = m_nearest(sample);
		if (PointInObstacle(sample, m_obstacles)) {
			continue;
		}

		Point new_point = nearest + (sample - nearest).normalize() * m_config.step_size;
		if (PointInObstacle(new_point, m_obstacles)) {
			continue;
		}

		if (!m_config.validate_new_sample(nearest, new_point)) {
			continue;
		}

		Node *new_node = new Node(new_point, m_tree.get_node(Node(nearest, nullptr)));
		m_tree.add_node(new_node);

		if (new_point.distance(goal) < m_config.goal_radius) {
			path = m_tree.path_to_root(*new_node);
			break;
		}
	}

	return path;
}

Tree RRTStar::tree() const { return m_tree; }

Point RRTStar::m_sample() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-1, 1);

	double x = dis(gen) * m_bounds.width() + m_bounds.center().x();
	double y = dis(gen) * m_bounds.height() + m_bounds.center().y();

	return Point(x, y);
}
Point RRTStar::m_nearest(const Point &sample) {
	Point nearest = m_tree.root()->point();
	double min_distance = sample.distance(nearest);

	for (Node *node : m_tree.nodes()) {
		double distance = sample.distance(node->point());
		if (distance < min_distance) {
			nearest = node->point();
			min_distance = distance;
		}
	}

	return nearest;
}