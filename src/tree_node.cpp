#include "inc/tree_node.hpp"

#include <vector>
#include <algorithm>

Node::Node() {
    this->m_parent = nullptr;
}
Node::Node(const Point& point, Node* parent) {
    this->m_point = point;
    this->m_parent = parent;
}

void Node::remove() {
    if (this->is_root()) {
        return;
    }
    this->m_parent->remove_child(*this);
    this->m_parent = nullptr;
}
Node& Node::remove_child(const Node& child) {
    for (Node* current : this->m_children) {
        if (*current == child) {
            this->m_children.erase(std::remove(this->m_children.begin(), this->m_children.end(), current), this->m_children.end());
            break;
        }
    }
    return *this;
}
Node& Node::remove_parent() {
    if (this->is_root()) {
        return *this;
    }
    this->m_parent->remove_child(*this);
    this->m_parent = nullptr;
    return *this;
}


Point Node::point() const {
    return this->m_point;
}
Node* Node::parent() const {
    return this->m_parent;
}
std::vector<Node*> Node::children() const {
    return this->m_children;
}

Node& Node::set_point(const Point& point) {
    this->m_point = point;
    return *this;
}
Node& Node::set_parent(Node* parent) {
    this->m_parent = parent;
    return *this;
}
Node& Node::add_child(Node* child) {
    this->m_children.push_back(child);
    return *this;
}
Node& Node::add_child(const Point& point) {
    this->m_children.push_back(new Node(point, this));
    return *this;
}
Node* Node::get_child(const Node& child) const {
    for (Node* current : this->m_children) {
        if (*current == child) {
            return current;
        }
    }
    return nullptr;
}

bool Node::operator==(const Node& other) const {
    return this->m_point == other.m_point;
}
bool Node::operator!=(const Node& other) const {
    return !(*this == other);
}

bool Node::is_root() const {
    return this->m_parent == nullptr;
}
bool Node::is_leaf() const {
    return this->m_children.empty();
}

double Node::distance(const Node& other) const {
    return this->m_point.distance(other.m_point);
}
double Node::distance_to_root() const {
    double distance = 0;
    const Node* current = this;
    while (!current->is_root()) {
        distance += current->distance(*current->m_parent);
        current = current->m_parent;
    }
    return distance;
}

std::vector<const Node*> Node::path_to_root() const {
    std::vector<const Node*> path;
    const Node* current = this;
    while (!current->is_root()) {
        path.push_back(current);
        current = current->m_parent;
    }
    path.push_back(current);
    return path;
}

bool Node::has_child(const Node& child) const {
    bool result = false;
    for (Node* current : this->m_children) {
        if (*current == child) {
            result = true;
            break;
        }
    }
    return result;
}
bool Node::has_parent(const Node& parent) const {
    bool result = false;
    const Node* current = this;
    while (!current->is_root()) {
        if (*current->m_parent == parent) {
            result = true;
            break;
        }
        current = current->m_parent;
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << node.m_point;
    for(Node* child : node.m_children) {
        os << ", " << *child;
    }
    return os;
}