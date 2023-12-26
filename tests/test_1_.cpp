#include "inc/_rrt_star.hpp"

int main(void) {

  RRTStarNode root;
  auto *cc = root.addChild()->addChild();

  printf("root g: %lu\n", root.steps_to_root());
  printf("cc g: %lu\n", cc->steps_to_root());

  return EXIT_SUCCESS;
}
