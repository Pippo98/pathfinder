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
void RRTStarTree::recursiveIteratorSlow(RRTStarNode *node, std::function<void(RRTStarNode *node)> callback) {
	callback(node);
	for (RRTStarNode *child : node->m_children) {
		recursiveIteratorSlow(child, callback);
	}
}
void RRTStarTree::recursiveIteratorFast(RRTStarNode *node, void *data, void(callback)(RRTStarNode *node, void *data)) {
	callback(node, data);
	for (RRTStarNode *child : node->m_children) {
		recursiveIteratorFast(child, data, callback);
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

bool IntersectsOrContained(const Point &contained, const Point &point2, const std::vector<Polygon> &obstacles) {
	for (const Polygon &obstacle : obstacles) {
		if (obstacle.contains(contained) || obstacle.intersects(contained, point2)) {
			return true;
		}
	}
	return false;
}

RRTStarNode *RRTStar::iterateOnce() {
	RRTStarNode *new_node = nullptr;
	auto new_p = sample();
	auto nearest = closestNode(new_p);
	auto &nearest_p = nearest->p();

	if (new_p.distance(nearest_p) > m_config.step_size) {
		new_p = nearest_p + ((new_p - nearest_p).normalize() * m_config.step_size);
	}
	if (IntersectsOrContained(new_p, nearest_p, m_obstacles)) {
		return nullptr;
	}
	auto close_nodes = closeNodes(new_p, m_config.step_size);
	if (close_nodes.size() != 0) {
		RRTStarNode *best_node = nullptr;
		double g = std::numeric_limits<double>::max();
		bool cached_intersections[close_nodes.size()];
		for (int i = 0; i < close_nodes.size(); i++) {
			cached_intersections[i] = PointIntersectsObstacle(new_p, close_nodes[i]->p(), m_obstacles);
			if (cached_intersections[i]) {
				continue;
			}
			double new_g = close_nodes[i]->g() + new_p.distance(close_nodes[i]->p());
			if (new_g < g) {
				g = new_g;
				best_node = close_nodes[i];
			}
		}
		if (best_node) {
			new_node = best_node->addChild(new_p);
			for (int i = 0; i < close_nodes.size(); i++) {
				if (close_nodes[i] == best_node) {
					continue;
				}
				if (cached_intersections[i]) {
					continue;
				}
				double candidate_g = new_node->g() + new_p.distance(close_nodes[i]->p());
				if (candidate_g < close_nodes[i]->g()) {
					close_nodes[i]->setParent(best_node);
				}
			}
		}
	}
	if (new_node == nullptr) {
		new_node = nearest->addChild(new_p);
	}
	// Update goal node
	double dist_to_goal = new_node->p().distance(m_goal);
	if (dist_to_goal < m_config.goal_radius) {
		if (!goal_node || dist_to_goal < goal_node->p().distance(m_goal)) {
			goal_node = new_node;
		}
	}
	return new_node;
}

bool RRTStar::findPath(const Point &start, const Point &goal) {
	freeProblem();
	m_tree.root = new RRTStarNode(start);
	m_start = start;
	m_goal = goal;
	if (PointInObstacle(goal, m_obstacles)) {
		return false;
	}

	bool updated_iterations = false;
	size_t last_iteration = m_config.max_iterations;
	for (int i = 0; i < last_iteration; i++) {
		iterateOnce();
		if (!updated_iterations && goal_node) {
			updated_iterations = true;
			last_iteration = i + m_config.smoothen_iterations;
		}
	}
	return goal_node != nullptr;
}
void RRTStar::freeProblem() {
	delete m_tree.root;
	goal_node = nullptr;
}

std::vector<Point> RRTStar::constructPath() {
	auto path = RRTStarTree::getPointsToRoot(goal_node);
	std::reverse(path.begin(), path.end());
	return path;
}

void RRTStar::smoothPath(size_t iterations) {
	for (size_t i = 0; i < iterations; i++) {
		iterateOnce();
	}
}

RRTStarNode *RRTStar::closestNode(const Point &p) {
	RRTStarNode *closest;
	double dist = std::numeric_limits<double>::max();

	struct tmpData {
		RRTStarNode **closest;
		double *dist;
		Point p;
	} data = {&closest, &dist, p};

	RRTStarTree::recursiveIteratorFast(m_tree.root, &data, [](RRTStarNode *node, void *data_) {
		auto data = (tmpData *)data_;
		if (node->p().distance(data->p) < *data->dist) {
			*data->dist = node->p().distance(data->p);
			*data->closest = node;
		}
	});

	return closest;
}
std::vector<RRTStarNode *> RRTStar::closeNodes(const Point &p, double radius) {
	std::vector<RRTStarNode *> nodes;

	struct tmpData {
		std::vector<RRTStarNode *> *nodes;
		const Point &p;
		double radius;
	} data = {&nodes, p, radius};

	RRTStarTree::recursiveIteratorFast(m_tree.root, &data, [](RRTStarNode *node, void *data_) {
		auto data = (tmpData *)data_;
		if (data->p.distance(node->p()) < data->radius) {
			data->nodes->push_back(node);
		}
	});

	return nodes;
}

Point RRTStar::sample() {
	std::uniform_real_distribution<> dis(0, 1);

	double x = dis(gen) * m_bounds.width() + m_bounds.center().x() - m_bounds.width() / 2.0;
	double y = dis(gen) * m_bounds.height() + m_bounds.center().y() - m_bounds.height() / 2.0;

	return Point(x, y);
}
