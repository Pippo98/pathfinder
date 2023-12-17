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

bool PointIntersectsObstacle(const Point &point1, const Point &point2, const std::vector<Polygon> &obstacles) {
	for (const Polygon &obstacle : obstacles) {
		if (obstacle.intersects(point1, point2)) {
			return true;
		}
	}
	return false;
}

std::vector<Node> RRTStar::find_path(const Point &start, const Point &goal) {
	std::vector<Node> path;
	m_tree = Tree(start);

	for (int i = 0; i < m_config.max_iterations; i++) {
		Point new_point = m_sample();
		Node &nearest_node = m_nearest_node(new_point);
		const Point &nearest = nearest_node.point_ref();
		Point nearest_node_parent_point;
		if (nearest_node.parent() != nullptr) {
			nearest_node_parent_point = nearest_node.parent()->point();
		}

		if (nearest.distance(new_point) > m_config.step_size) {
			new_point = nearest + ((new_point - nearest).normalize() * m_config.step_size);
		}
		if (PointInObstacle(new_point, m_obstacles)) {
			continue;
		}
		if (PointIntersectsObstacle(nearest, new_point, m_obstacles)) {
			continue;
		}
		if (!m_config.validate_new_sample(nearest_node_parent_point, nearest, new_point)) {
			continue;
		}

		std::vector<Node *> nears = m_near(new_point, m_config.step_size);
		Node *min_node = nullptr;
		double min_cost = std::numeric_limits<double>::max();
		for (size_t j = 0; j < nears.size(); j++) {
			double cost = nears[j]->path_to_root().size() + nears[j]->point_ref().distance(new_point);
			if (cost < min_cost) {
				min_node = nears[j];
				min_cost = cost;
			}
		}

		Node *new_node;
		if (min_node != nullptr) {
			new_point = min_node->point_ref() + (new_point - min_node->point_ref()).normalize() * m_config.step_size;
			min_node->add_child(new_point);
			new_node = min_node->children().back();
		} else {
			nearest_node.add_child(new_point);
			new_node = nearest_node.children().back();
		}

		m_tree.add_node(new_node);

		if (new_point.distance(goal) < m_config.goal_radius) {
			path = m_tree.path_to_root(*new_node);
			std::cout << "found path" << std::endl;
			break;
		}
	}

	std::reverse(path.begin(), path.end());

	return path;
}

Tree RRTStar::tree() const { return m_tree; }

Point RRTStar::m_sample() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	double x = dis(gen) * m_bounds.width() + m_bounds.center().x() - m_bounds.width() / 2.0;
	double y = dis(gen) * m_bounds.height() + m_bounds.center().y() - m_bounds.height() / 2.0;

	return Point(x, y);
}

Node *recursive_nearest_node(Node *node, const Point &sample, double &min_distance) {
	Node *nearest = nullptr;
	double distance = sample.distance(node->point_ref());
	if (distance <= min_distance) {
		nearest = node;
		min_distance = distance;
	}

	for (Node *child : node->children()) {
		Node *child_nearest = recursive_nearest_node(child, sample, min_distance);
		if (child_nearest != nullptr) {
			nearest = child_nearest;
		}
	}

	return nearest;
}

Node &RRTStar::m_nearest_node(const Point &sample) {
	double min_distance = std::numeric_limits<double>::max();
	Node *nearest = recursive_nearest_node(m_tree.root(), sample, min_distance);

	if (nearest == nullptr) {
		nearest = m_tree.root();
	}

	return *nearest;
}
Point RRTStar::m_nearest_point(const Point &sample) { return m_nearest_node(sample).point_ref(); }

std::vector<Node *> recursive_nearest_nodes(Node *node, const Point &sample, double radius) {
	std::vector<Node *> nodes;
	double distance = sample.distance(node->point_ref());
	if (distance < radius) {
		nodes.push_back(node);
	}

	for (Node *child : node->children()) {
		std::vector<Node *> child_nearest_nodes = recursive_nearest_nodes(child, sample, radius);
		nodes.insert(nodes.end(), child_nearest_nodes.begin(), child_nearest_nodes.end());
	}

	return nodes;
}

std::vector<Node *> RRTStar::m_near(const Point &sample, double radius) {
	return recursive_nearest_nodes(m_tree.root(), sample, radius);
}