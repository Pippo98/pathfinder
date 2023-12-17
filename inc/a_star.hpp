#ifndef PATHFINDER_A_STAR_HPP_
#define PATHFINDER_A_STAR_HPP_

#include "inc/point.hpp"
#include "inc/polygon.hpp"

#include <map>
#include <queue>
#include <vector>
#include <functional>

template <typename T, typename S, typename C> class AStarPQueue : public std::priority_queue<T, S, C> {
public:
	auto begin() { return this->c.begin(); }
	auto end() { return this->c.end(); }
	auto cbegin() const { return this->c.cbegin(); }
	auto cend() const { return this->c.cend(); }
	void make_heap() { std::make_heap(this->c.begin(), this->c.end(), this->comp); }
};

class AStarNode {
public:
	AStarNode *parent = nullptr;
	Point p;
	double g, h;

	double f() const { return g + h; };

	bool operator==(const AStarNode &other) const { return p == other.p; }
};
static bool node_comparator(AStarNode *left, AStarNode *right) { return left->f() < right->f(); }

class AStarConfig {
public:
	double step_size;
	int max_iterations;

	std::function<double(const AStarNode *come_from, const Point &new_sample, const Point &goal)> g;
	std::function<double(const AStarNode *come_from, const Point &new_sample, const Point &goal)> h;

	AStarConfig()
			: step_size(0.1), max_iterations(1000),
				g([](const AStarNode *come_from, const Point &new_sample, const Point &goal) {
					return come_from->g + come_from->p.distance(new_sample);
				}),
				h([](const AStarNode *come_from, const Point &new_sample, const Point &goal) {
					return goal.distance(new_sample);
				}) {}
};

class AStar {
public:
	AStar();

	void set_config(const AStarConfig &config);
	void set_bounds(const Rectangle &bounds);
	void set_obstacles(const std::vector<Polygon> &obstacles);

	std::vector<Point> find_path(const Point &start, const Point &goal);

private:
	Rectangle m_bounds;
	std::vector<Polygon> m_obstacles;
	AStarConfig m_config;

	Point m_start;
	Point m_goal;

	std::map<int64_t, std::map<int64_t, AStarNode *>> m_known_nodes;

private:
	AStarNode *m_get_or_sample_pos(AStarNode *, const Point &offset);
	AStarNode **m_sample_around(AStarNode *);

	void m_new_known_node(AStarNode *node);
	AStarNode *m_node_is_known(const AStarNode *node);
};

#endif // PATHFINDER_A_STAR_HPP_