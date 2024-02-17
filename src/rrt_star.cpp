#include "inc/rrt_star.hpp"

#include <limits>

RRTStarNode::RRTStarNode() : RRTStarNode(nullptr, Point()) {}
RRTStarNode::RRTStarNode(const Point &p) : RRTStarNode(nullptr, p) {}
RRTStarNode::RRTStarNode(RRTStarNode *_parent, const Point &_p) {
	m_point = _p;
	m_parent = _parent;
	if (m_parent != nullptr) {
		m_steps_to_root = m_parent->steps_to_root() + 1;
	}
}

void RRTStarNode::remove() {
	if (m_parent != nullptr) {
		m_parent->removeChild(this);
	} else {
		RRTStarNode::removeNode(this);
	}
}
void RRTStarNode::removeChild(RRTStarNode *node) {
	auto child = m_children.find(node);
	if (child != m_children.end()) {
		RRTStarNode::removeNode(node);
		m_children.erase(child);
	}
}
void RRTStarNode::removeNode(RRTStarNode *node) {
	while (!node->m_children.empty()) {
		auto to_remove = node->m_children.begin();
		removeNode(*to_remove);
		node->m_children.erase(to_remove);
	}
	delete node;
}

RRTStarNode *RRTStarNode::addChild() { return addChild(Point()); }
RRTStarNode *RRTStarNode::addChild(const Point &p) {
	RRTStarNode *new_node = new RRTStarNode(this, p);
	m_children.insert(new_node);
	return new_node;
}

double RRTStarNode::g() {
	if (m_parent != nullptr) {
		return m_parent->g() + m_point.distance(m_parent->m_point);
	} else {
		return 0.0;
	}
}
Point &RRTStarNode::p() { return m_point; }
uint64_t RRTStarNode::steps_to_root() { return m_steps_to_root; }

void RRTStarNode::setPoint(const Point &p) { m_point = p; }
void RRTStarNode::setParent(RRTStarNode *parent) { m_parent = parent; }

RRTStarTree::RRTStarTree() : RRTStarTree(Point()) {}
RRTStarTree::RRTStarTree(const Point &p) { root = new RRTStarNode(p); }
RRTStarTree::RRTStarTree(RRTStarNode *_root) { root = root; }
void RRTStarTree::recursiveIterator(RRTStarNode *node, std::function<void(RRTStarNode *node)> callback) {
	callback(node);
	for (RRTStarNode *child : node->m_children) {
		recursiveIterator(child, callback);
	}
}
std::vector<Point> RRTStarTree::getPointsToRoot(RRTStarNode *from) {
	std::vector<Point> path;
	auto *curr = from;
	while (curr != nullptr) {
		path.push_back(curr->p());
		curr = curr->m_parent;
	}
	return path;
}
std::vector<RRTStarNode *> RRTStarTree::getNodesToRoot(RRTStarNode *from) {
	std::vector<RRTStarNode *> path;
	auto *curr = from;
	while (curr != nullptr) {
		path.push_back(curr);
		curr = curr->m_parent;
	}
	return path;
}

RRTStar::RRTStar() {
	std::random_device rd;
	gen = std::mt19937(rd());
}

void RRTStar::setConfig(const RRTStarConfig &config) { m_config = config; }
void RRTStar::setBounds(const Rectangle &bounds) { m_bounds = bounds; }
void RRTStar::setObstacles(const std::vector<Polygon> &obstacles) { m_obstacles = obstacles; }

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

RRTStarNode *RRTStar::findNode(const Point &start, const Point &goal) {
	freeProblem();
	m_tree.root = new RRTStarNode(start);
	m_start = start;
	m_goal = goal;
	if (PointInObstacle(goal, m_obstacles)) {
		return nullptr;
	}

	bool updated_iterations = false;
	RRTStarNode *goal_node = nullptr;
	double min_dist_to_goal = start.distance(goal);
	size_t last_iteration = m_config.max_iterations;
	for (int i = 0; i < last_iteration; i++) {
		auto new_p = sample(m_config.step_size * 10.0);
		auto nearest = closestNode(new_p);
		auto &nearest_p = nearest->p();

		if (new_p.distance(nearest_p) > m_config.step_size) {
			new_p = nearest_p + ((new_p - nearest_p).normalize() * m_config.step_size);
		}
		if (PointInObstacle(new_p, m_obstacles) || PointIntersectsObstacle(nearest_p, new_p, m_obstacles)) {
			continue;
		}
		RRTStarNode *new_node = nullptr;
		auto close_nodes = closeNodes(new_p, m_config.step_size);
		if (close_nodes.size() != 0) {
			RRTStarNode *best_node = nullptr;
			double g = std::numeric_limits<double>::max();
			for (const auto &node : close_nodes) {
				if (PointIntersectsObstacle(new_p, node->p(), m_obstacles)) {
					continue;
				}
				double new_g = node->g() + new_p.distance(node->p());
				if (new_g < g) {
					g = new_g;
					best_node = node;
				}
			}
			if (best_node) {
				new_node = best_node->addChild(new_p);
				for (auto node : close_nodes) {
					if (node == best_node) {
						continue;
					}
					if (PointIntersectsObstacle(new_p, node->p(), m_obstacles)) {
						continue;
					}
					double candidate_g = new_node->g() + new_p.distance(node->p());
					if (candidate_g < node->g()) {
						node->setParent(best_node);
					}
				}
			}
		}
		if (new_node == nullptr) {
			new_node = nearest->addChild(new_p);
		}
		double dist_to_goal = new_node->p().distance(goal);
		if (dist_to_goal < min_dist_to_goal) {
			min_dist_to_goal = dist_to_goal;
			goal_node = new_node;
		}
		if (!updated_iterations && dist_to_goal < m_config.goal_radius) {
			updated_iterations = true;
			last_iteration = i + m_config.smoothen_iterations;
		}
	}
	return goal_node;
}

std::vector<Point> RRTStar::findPath(const Point &start, const Point &goal) {
	auto *goal_node = findNode(start, goal);
	if (goal_node == nullptr) {
		return std::vector<Point>();
	}
	auto path = RRTStarTree::getPointsToRoot(goal_node);
	std::reverse(path.begin(), path.end());
	return path;
}
void RRTStar::freeProblem() { delete m_tree.root; }

RRTStarNode *RRTStar::closestNode(const Point &p) {
	RRTStarNode *closest;
	double dist = std::numeric_limits<double>::max();
	RRTStarTree::recursiveIterator(m_tree.root, [&p, &closest, &dist](RRTStarNode *node) {
		if (dist > p.distance(node->p())) {
			dist = p.distance(node->p());
			closest = node;
		}
	});

	return closest;
}
std::vector<RRTStarNode *> RRTStar::closeNodes(const Point &p, double radius) {
	std::vector<RRTStarNode *> nodes;

	RRTStarTree::recursiveIterator(m_tree.root, [&p, &nodes, &radius](RRTStarNode *node) {
		if (p.distance(node->p()) < radius) {
			nodes.push_back(node);
		}
	});

	return nodes;
}

Point RRTStar::sample(double weight_factor) {
	/*
	std::random_device rd;
	std::mt19937 gen(rd());

	double std_dev = std::sqrt(m_bounds.width() * 20);

	double min_x = m_bounds.center().x() - m_bounds.width() / 2.0;
	double min_y = m_bounds.center().y() - m_bounds.height() / 2.0;
	double max_x = min_x + m_bounds.width();
	double max_y = min_y + m_bounds.height();

	std::normal_distribution<double> dist_x(m_goal.x(), std_dev);
	std::normal_distribution<double> dist_y(m_goal.y(), std_dev);

	double x = std::max(min_x, std::min(dist_x(gen), max_x));
	double y = std::max(min_y, std::min(dist_y(gen), max_y));
	*/

	std::uniform_real_distribution<> dis(0, 1);

	double x = dis(gen) * m_bounds.width() + m_bounds.center().x() - m_bounds.width() / 2.0;
	double y = dis(gen) * m_bounds.height() + m_bounds.center().y() - m_bounds.height() / 2.0;

	return Point(x, y);
}
