#pragma once
#include <vector>
#include <iostream>

template <typename AStarNodeT>
struct LowLevelPlan {
  std::vector<AStarNodeT> plan;
  float cost;
};
