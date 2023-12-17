#ifndef PATHFINDER_TREE_HPP_
#define PATHFINDER_TREE_HPP_

#include "inc/tree_node.hpp"

#include <vector>

class Tree {
public:
	Tree();
	Tree(const Point &root);

	Node *root() const;
	std::vector<Node *> nodes() const;

	Tree &set_root(const Point &root);
	Tree &add_node(Node *node);

	bool operator==(const Tree &other) const;
	bool operator!=(const Tree &other) const;

	Node *get_node(const Node &node) const;

	Tree &remove_node(const Node &node);

	bool is_empty() const;

	std::vector<const Node *> get_leaves() const;
	std::vector<Node> path_to_root(const Node &node) const;
	std::vector<const Node *> path_to_root_ptr(const Node &node) const;

	friend std::ostream &operator<<(std::ostream &os, const Tree &tree);

private:
	Node *m_root;
	std::vector<Node *> m_nodes;
};

#endif // PATHFINDER_TREE_HPP_