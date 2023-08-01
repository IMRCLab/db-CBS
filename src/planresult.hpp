#pragma once
#include <vector>
#include <iostream>

template <typename AStarNodeT, typename StateT, typename StateA>
struct LowLevelPlan {
  std::vector<AStarNodeT> plan;
  std::vector<StateT> trajectory;
  std::vector<StateA> actions;
  float cost;
};
