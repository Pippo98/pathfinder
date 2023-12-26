#ifndef _RRT_STAR_HPP
#define _RRT_STAR_HPP

#include "inc/point.hpp"
#include "inc/polygon.hpp"

#include <inttypes.h>
#include <unordered_set>

class RRTStarNode {
public:
	friend class RRTStarTree;

	RRTStarNode();
	RRTStarNode(const Point &p);
	RRTStarNode(RRTStarNode *parent, const Point &p);

	void remove();
	void removeChild(RRTStarNode *node);

	RRTStarNode *addChild();
	RRTStarNode *addChild(const Point &p);

	Point &p();
	uint64_t steps_to_root();

	void setPoint(const Point &p);

private:
	RRTStarNode *parent;
	std::unordered_set<RRTStarNode *> children;

	uint64_t g;
	Point point;

private:
	static void removeNode(RRTStarNode *node);
};

class RRTStarTree {
public:
	friend class RRTStar;
	RRTStarTree();
	RRTStarTree(const Point &p);

private:
	RRTStarNode *root;
};

class RRTStarConfig {
public:
	double step_size;
	size_t max_iterations;

	RRTStarConfig() : step_size(0.1), max_iterations(1000) {}
};

class RRTStar {
public:
	RRTStar();

	void setConfig(const RRTStarConfig &config);
	void setBounds(const Rectangle &bounds);
	void setObstacles(const std::vector<Polygon> &obstacles);

	std::vector<Point> findPath(const Point &start, const Point &goal);

	const Rectangle &bounds() const { return m_bounds; }
	const std::vector<Polygon> &obstacles() const { return m_obstacles; }
	const RRTStarConfig &config() const { return m_config; }

private:
	Rectangle m_bounds;
	std::vector<Polygon> m_obstacles;
	RRTStarConfig m_config;

	Point m_start;
	Point m_goal;

	RRTStarTree m_tree;

private:
	RRTStarNode *closestNode(const Point &p);
	std::vector<RRTStarNode *> closeNodes(const Point &p, double radius);
};

#endif // _RRT_STAR_HPP
