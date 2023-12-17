#include "inc/a_star.hpp"

#include <cmath>
#include <fstream>
#include <unistd.h>
#include <unordered_set>

AStar::AStar() {}

void AStar::set_config(const AStarConfig &config) { m_config = config; }
void AStar::set_bounds(const Rectangle &bounds) { m_bounds = bounds; }
void AStar::set_obstacles(const std::vector<Polygon> &obstacles) { m_obstacles = obstacles; }

struct compare_nodes {
	bool operator()(const AStarNode *a, const AStarNode *b) { return a->f() > b->f(); }
};

std::vector<Point> AStar::find_path(const Point &start, const Point &goal) {
	std::unordered_set<AStarNode *> closed_set;
	std::unordered_set<AStarNode *> open_set_lookup;
	m_goal = goal;
	m_start = start;
	AStarNode start_node;
	start_node.p = start;
	start_node.g = 0.0;
	start_node.h = start.distance(goal);

	AStarPQueue<AStarNode *, std::vector<AStarNode *>, compare_nodes> open_set;

	open_set.push(&start_node);
	open_set_lookup.insert(&start_node);

	size_t iteration = 0;
	AStarNode *goal_node = nullptr;

	int new_samples = 0;
	int skipped_nodes = 0;
	while (!open_set.empty() && iteration < m_config.max_iterations) {
		iteration++;
		auto current = open_set.top();
		open_set.pop();

		closed_set.insert(current);
		if (current->p.distance(goal) <= m_config.goal_radius) {
			goal_node = current;
			break;
		}

		// update frontier
		auto new_nodes = m_sample_around(current);
		for (int i = 0; i < 8; i++) {
			new_samples++;
			AStarNode *node = new_nodes[i];

			// if the point was in a obstacle or outside boundaries
			if (node == nullptr) {
				skipped_nodes++;
				continue;
			}

			// Check if point was already discovered
			if (closed_set.find(node) != closed_set.end()) {
				continue;
			}

			double new_g = current->g + current->p.distance(node->p);

			bool is_in_fronteer = open_set_lookup.find(node) != open_set_lookup.end();

			if (new_g < node->g || !is_in_fronteer) {
				node->parent = current;
				node->g = new_g;
				node->h = goal.distance(node->p);

				if (!is_in_fronteer) {
					open_set.push(node);
					open_set_lookup.insert(node);
				} else {
					open_set.make_heap();
				}
			}
		}
	}

	std::vector<Point> path;
	while (goal_node != nullptr) {
		path.push_back(goal_node->p);
		if (goal_node->parent == nullptr)
			break;
		goal_node = goal_node->parent;
	}
	std::reverse(path.begin(), path.end());

	for (auto &rows : m_known_nodes) {
		for (auto &columns : rows.second) {
			delete columns.second;
		}
	}
	m_known_nodes.clear();
	
	return path;
}

void AStar::m_new_known_node(AStarNode *node) {
	int64_t dx = std::round(node->p.x() / m_config.step_size);
	int64_t dy = std::round(node->p.y() / m_config.step_size);
	m_known_nodes[dx][dy] = node;
}

AStarNode *AStar::m_node_is_known(const AStarNode *node) {
	int64_t dx = std::round(node->p.x() / m_config.step_size);
	int64_t dy = std::round(node->p.y() / m_config.step_size);
	if (m_known_nodes.find(dx) != m_known_nodes.end() && m_known_nodes[dx].find(dy) != m_known_nodes[dx].end()) {
		return m_known_nodes[dx][dy];
	} else {
		return nullptr;
	}
}

AStarNode *AStar::m_get_or_sample_pos(AStarNode *node, const Point &offset) {

	AStarNode tmp_node = *node;
	tmp_node.p = tmp_node.p + offset;
	AStarNode *new_node = m_node_is_known(&tmp_node);
	if (new_node != nullptr) {
		return new_node;
	}

	if (!m_bounds.contains(node->p + offset)) {
		return nullptr;
	}
	for (const Polygon &obstacle : m_obstacles) {
		if (obstacle.contains(node->p + offset)) {
			return nullptr;
		}
	}

	new_node = new AStarNode();
	new_node->p = node->p + offset;
	new_node->g = node->g + node->p.distance(new_node->p);
	new_node->h = m_goal.distance(new_node->p);
	new_node->parent = node;
	m_new_known_node(new_node);

	return new_node;
}
AStarNode **AStar::m_sample_around(AStarNode *node) {
	AStarNode **nodes = new AStarNode *[8];
	double ss = m_config.step_size;
	nodes[0] = m_get_or_sample_pos(node, Point(ss, 0));
	nodes[1] = m_get_or_sample_pos(node, Point(-ss, 0));
	nodes[2] = m_get_or_sample_pos(node, Point(0, ss));
	nodes[3] = m_get_or_sample_pos(node, Point(0, -ss));
	nodes[4] = m_get_or_sample_pos(node, Point(ss, ss));
	nodes[5] = m_get_or_sample_pos(node, Point(-ss, ss));
	nodes[6] = m_get_or_sample_pos(node, Point(-ss, -ss));
	nodes[7] = m_get_or_sample_pos(node, Point(ss, -ss));
	return nodes;
}
