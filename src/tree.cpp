#include "inc/tree.hpp"

#include <algorithm>

Tree::Tree() { this->m_root = nullptr; }
Tree::Tree(const Point &root) { this->m_root = new Node(root, nullptr); }

Tree &Tree::set_root(const Point &root) {
	if (this->m_root != nullptr) {
		this->m_root->set_point(root);
	} else {
		this->m_root = new Node(root, nullptr);
	}
	return *this;
}
Tree &Tree::add_node(Node *node) {
	this->m_nodes.push_back(node);
	return *this;
}

Node *Tree::root() const { return this->m_root; }
std::vector<Node *> Tree::nodes() const { return this->m_nodes; }

bool Tree::operator==(const Tree &other) const { return this->m_root == other.m_root; }
bool Tree::operator!=(const Tree &other) const { return !(*this == other); }

Node *Tree::get_node(const Node &node) const {
	// recursive search
	for (Node *current : this->m_nodes) {
		if (*current == node) {
			return current;
		}
		Node *n = current->get_child(node);
		if (n != nullptr) {
			return n;
		}
	}
	return nullptr;
}

Tree &Tree::remove_node(const Node &node) {
	Node *n = this->get_node(node);
	if (n != nullptr) {
		n->remove();
		this->m_nodes.erase(std::remove(this->m_nodes.begin(), this->m_nodes.end(), n), this->m_nodes.end());
	}
	return *this;
}

bool Tree::is_empty() const { return this->m_root == nullptr; }

std::vector<Node> Tree::path_to_root(const Node &node) const {
	std::vector<Node> path;
	Node *n = this->get_node(node);
	while (n != nullptr) {
		path.push_back(*n);
		n = n->parent();
	}
	return path;
}
std::vector<const Node *> Tree::path_to_root_ptr(const Node &node) const {
	std::vector<const Node *> path;
	Node *n = this->get_node(node);
	while (n != nullptr) {
		path.push_back(n);
		n = n->parent();
	}
	return path;
}
std::vector<const Node *> Tree::get_leaves() const {
	std::vector<const Node *> leaves;
	for (Node *n : this->m_nodes) {
		if (n->is_leaf()) {
			leaves.push_back(n);
		}
	}
	return leaves;
}

std::ostream &operator<<(std::ostream &os, const Tree &tree) {
	os << "Tree(" << tree.m_root->point() << ", [";
	for (Node *n : tree.m_nodes) {
		os << *n << ", ";
	}
	os << "])";
	return os;
}