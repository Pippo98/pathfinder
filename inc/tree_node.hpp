#ifndef RRT_STAR_TREE_NODE_HPP_
#define RRT_STAR_TREE_NODE_HPP_

#include "inc/point.hpp"

#include <vector>

class Node {
public:
	Node();
	Node(const Point &point, Node *parent);

	Point point() const;
	Node *parent() const;
	std::vector<Node *> children() const;

	Node &set_point(const Point &point);
	Node &set_parent(Node *parent);
	Node &add_child(Node *child);
	Node &add_child(const Point &point);
	Node *get_child(const Node &child) const;

	bool operator==(const Node &other) const;
	bool operator!=(const Node &other) const;

	bool is_root() const;
	bool is_leaf() const;

	double distance(const Node &other) const;
	double distance_to_root() const;

	std::vector<const Node *> path_to_root() const;

	bool has_child(const Node &child) const;
	bool has_parent(const Node &parent) const;

	Node &remove_child(const Node &child);
	Node &remove_parent();
	void remove();

	friend std::ostream &operator<<(std::ostream &os, const Node &node);

private:
	Point m_point;
	Node *m_parent;
	std::vector<Node *> m_children;
};

#endif // RRT_STAR_TREE_NODE_HPP_