#include "inc/tree.hpp"

int main(void) {
    // generate tree
    Tree tree;
    tree.set_root(Point(0.0, 0.0));
    tree.add_node(new Node(Point(1.0, 1.0), tree.root()));
    tree.add_node(new Node(Point(2.0, 2.0), tree.root()));
    tree.add_node(new Node(Point(3.0, 3.0), tree.root()));
    tree.add_node(new Node(Point(4.0, 4.0), tree.root()));

    // print tree
    std::cout << tree << std::endl;

    // get leaves
    std::vector<const Node*> leaves = tree.get_leaves();
    for (const Node* leaf : leaves) {
        std::cout << "leaf: " << *leaf << std::endl;
    }

    // add child to leaf
    Node* leaf = tree.get_node(*leaves[0]);
    leaf->add_child(Point(5.0, 5.0));
    leaf->add_child(Point(6.0, 6.0));
    leaf->get_child(*leaf->children()[0])->add_child(Point(7.0, 7.0));

    // print tree
    std::cout << tree << std::endl;

    // get leaves
    leaves = tree.get_leaves();
    for (const Node* leaf : leaves) {
        std::cout << "leaf: " << *leaf << std::endl;
    }

    // remove leaf
    tree.remove_node(*leaves[0]);

    // print tree
    std::cout << tree << std::endl;

    Node* n = tree.get_node(Node(Point(5.0, 5.0), nullptr));
    std::cout << "n: " << *n << std::endl;
    n->remove();
    std::cout << tree << std::endl;

    return 0;
}