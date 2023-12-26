#include "inc/_rrt_star.hpp"

RRTStarNode::RRTStarNode() : RRTStarNode(nullptr, Point()) {}
RRTStarNode::RRTStarNode(const Point &p) : RRTStarNode(nullptr, p) {}
RRTStarNode::RRTStarNode(RRTStarNode *_parent, const Point &_p) {
	point = _p;
	parent = _parent;
	if (parent != nullptr) {
		this->g = parent->steps_to_root() + 1;
	}
}

void RRTStarNode::remove() {
	if (parent != nullptr) {
		parent->removeChild(this);
	} else {
		RRTStarNode::removeNode(this);
	}
}
void RRTStarNode::removeChild(RRTStarNode *node) {
	auto child = children.find(node);
	if (child != children.end()) {
		RRTStarNode::removeNode(node);
		children.erase(child);
	}
}
void RRTStarNode::removeNode(RRTStarNode *node) {
	while (!node->children.empty()) {
		auto to_remove = node->children.begin();
		removeNode(*to_remove);
		node->children.erase(to_remove);
	}
	delete node;
}

RRTStarNode *RRTStarNode::addChild() { return addChild(Point()); }
RRTStarNode *RRTStarNode::addChild(const Point &p) {
	RRTStarNode *new_node = new RRTStarNode(this, p);
	children.insert(new_node);
	return new_node;
}

Point &RRTStarNode::p() { return point; }
uint64_t RRTStarNode::steps_to_root() { return g; }

void RRTStarNode::setPoint(const Point &p) { point = p; }

RRTStarTree::RRTStarTree() : RRTStarTree(Point()) {}
RRTStarTree::RRTStarTree(const Point &p) { root = new RRTStarNode(p); }

RRTStar::RRTStar() {}

void RRTStar::setConfig(const RRTStarConfig &config) { m_config = config; }
void RRTStar::setBounds(const Rectangle &bounds) { m_bounds = bounds; }
void RRTStar::setObstacles(const std::vector<Polygon> &obstacles) { m_obstacles = obstacles; }
