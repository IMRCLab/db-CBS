#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <bits/stdc++.h>
#include <tuple>
// BOOST
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// fcl
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
// dynoplan
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include <dynoplan/optimization/multirobot_optimization.hpp>
// dynobench
#include "dynobench/nn.h"
#include <dynobench/multirobot_trajectory.hpp>
#include "dynobench/motions.hpp"

// Conflicts
struct Conflict
{
  double time;
  size_t robot_idx_i;
  Eigen::VectorXd robot_state_i;
  size_t robot_idx_j;
  Eigen::VectorXd robot_state_j;
};
// Constraints
struct HighLevelNode
{
  std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
  std::vector<std::vector<dynoplan::Constraint>> constraints;
  double cost;
  double LB;
  int focalHeuristic;
  int id;

  typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                   boost::heap::mutable_<true>>::handle_type // openset_handle_type
      handle;

  bool operator<(const HighLevelNode &n) const
  {
    return cost > n.cost;
  }
};

struct HighLevelNodeFocal
{
  std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
  std::vector<std::vector<dynoplan::Constraint>> constraints;
  // std::vector<std::vector<std::pair<std::shared_ptr<dynoplan::AStarNode>, size_t>>> result;
  // std::vector<std::map<size_t, dynoplan::Motion*>> result_motions;
  double cost;
  double LB;
  int focalHeuristic;
  int id;

  typename boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>,
                                   boost::heap::mutable_<true>>::handle_type // openset_handle_type
      handle;

  bool operator<(const HighLevelNodeFocal &n) const
  {
    return cost > n.cost;
  }
};

// fefine your binary heap type
typedef boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>, boost::heap::mutable_<true>> openset_t;
// access its handle_type
typedef openset_t::handle_type openset_handle_type;
// create binary heap of handle_type (nested type) of openset_t binary heap
// typedef boost::heap::d_ary_heap<openset_handle_type, boost::heap::arity<2>, boost::heap::mutable_<true>> focalset_t;
struct compareFocalHeuristic
{
  bool operator()(const openset_handle_type &h1,
                  const openset_handle_type &h2) const
  {
    if ((*h1).focalHeuristic != (*h2).focalHeuristic)
    {
      return (*h1).focalHeuristic > (*h2).focalHeuristic;
    }
    return (*h1).cost > (*h2).cost;
  }
};

typedef boost::heap::d_ary_heap<
    openset_handle_type, boost::heap::arity<2>,
    boost::heap::compare<compareFocalHeuristic>, boost::heap::mutable_<true>>
    focalset_t;

// for cbs-style optimization
struct HighLevelNodeOptimization
{
  // std::vector<dynobench::Trajectory> trajectories;
  MultiRobotTrajectory multirobot_trajectory;
  std::unordered_set<size_t> cluster;                               // robot idx for the joint optimization
  std::vector<std::pair<std::unordered_set<size_t>, int>> clusters; // used only with greedy cbs (cluster, its conflict)
  std::vector<std::vector<int>> conflict_matrix;
  // std::vector<std::vector<float>> residual_forces; // for each robot in the final solution
  double cost;
  int conflict;
  int id;

  HighLevelNodeOptimization(int rows, int cols)
      : conflict_matrix(rows, std::vector<int>(cols, 0)),
        // residual_forces(rows, std::vector<float>(rows, 0.0)),
        cost(0.0),
        conflict(0),
        id(0)
  {
  }
  // check if the robot belongs to any cluster, return index of the clusters vector index
  int containsX(size_t X) const
  {
    int index = 0;
    for (const auto &pair : clusters)
    {
      if (pair.first.find(X) != pair.first.end())
      {
        return index;
      }
      ++index;
    }
    return -1;
  }

  typename boost::heap::d_ary_heap<HighLevelNodeOptimization, boost::heap::arity<2>,
                                   boost::heap::mutable_<true>>::handle_type
      handle;

  bool operator<(const HighLevelNodeOptimization &n) const
  {
    return conflict < n.conflict; // max
  }
};

bool getEarliestConflict(
    const std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs,
    Conflict &early_conflict)
{
  size_t max_t = 0;
  for (const auto &sol : solution)
  {
    max_t = std::max(max_t, sol.trajectory.states.size() - 1);
  }
  Eigen::VectorXd node_state;
  std::vector<Eigen::VectorXd> node_states;

  for (size_t t = 0; t <= max_t; ++t)
  {
    node_states.clear();
    size_t robot_idx = 0;
    size_t obj_idx = 0;
    std::vector<fcl::Transform3d> ts_data;
    for (auto &robot : all_robots)
    {
      if (t >= solution[robot_idx].trajectory.states.size())
      {
        node_state = solution[robot_idx].trajectory.states.back();
      }
      else
      {
        node_state = solution[robot_idx].trajectory.states[t];
      }
      node_states.push_back(node_state);
      std::vector<fcl::Transform3d> tmp_ts(1);
      if (robot->name == "car_with_trailers")
      {
        tmp_ts.resize(2);
      }
      robot->transformation_collision_geometries(node_state, tmp_ts);
      ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
      ++robot_idx;
    }
    for (size_t i = 0; i < ts_data.size(); i++)
    {
      fcl::Transform3d &transform = ts_data[i];
      robot_objs[obj_idx]->setTranslation(transform.translation());
      robot_objs[obj_idx]->setRotation(transform.rotation());
      robot_objs[obj_idx]->computeAABB();
      ++obj_idx;
    }
    col_mng_robots->update(robot_objs);
    fcl::DefaultCollisionData<double> collision_data;
    col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision())
    {
      assert(collision_data.result.numContacts() > 0);
      const auto &contact = collision_data.result.getContact(0);

      early_conflict.time = t * all_robots[0]->ref_dt;
      early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
      early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
      assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
      early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
      early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
      std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;

      return true;
    }
  }
  return false;
}
// for heterogeneous case with the residual force
// no prioritization, only create constraints
// doesn't work with car_trailer, assumes robots are in consec.order
bool getEarliestViolations(
    const std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    std::vector<std::string> &robot_types,
    std::map<size_t, std::vector<dynoplan::Constraint>> &constraints)
{
  double max_f = 0.0981; // in Newton
  float rho;
  std::vector<size_t> involved_robots;
  size_t max_t = 0;
  for (const auto &sol : solution)
  {
    max_t = std::max(max_t, sol.trajectory.states.size() - 1);
  }
  Eigen::VectorXd state, state1, state2;
  std::vector<Eigen::VectorXd> states;
  for (size_t t = 0; t <= max_t; ++t)
  {
    states.clear();
    for (size_t robot_idx = 0; robot_idx < solution.size(); ++robot_idx)
    {
      if (t >= solution[robot_idx].trajectory.states.size())
      {
        state = solution[robot_idx].trajectory.states.back();
      }
      else
      {
        state = solution[robot_idx].trajectory.states[t];
      }
      states.push_back(state);
    }
    for (size_t i = 0; i < solution.size(); ++i)
    {
      involved_robots.clear(); // for each robot
      rho = 0;                 // for each robot
      state1 = states.at(i);
      nn_reset(); // reset for each robot
      for (size_t j = 0; j < solution.size(); ++j)
      {
        if (i != j)
        { // fa for each robot coming from neighbors
          state2 = states.at(j);
          auto dist = state1 - state2;
          if (abs(dist(0)) < 0.2 && abs(dist(1)) < 0.2 &&
              abs(dist(2)) < 1.5)
          {
            float input[6] = {
                static_cast<float>(dist(0)), static_cast<float>(dist(1)),
                static_cast<float>(dist(2)), static_cast<float>(dist(3)),
                static_cast<float>(dist(4)), static_cast<float>(dist(5))};
            const auto nnType = (robot_types[j] == "integrator2_3d_large_v0")
                                    ? NN_ROBOT_LARGE
                                    : NN_ROBOT_SMALL;

            nn_add_neighbor(input, nnType);
            // keep track of contributing robots
            involved_robots.push_back(j);
          }
        }
      }
      // after checking all neighbors
      const auto selfType = (robot_types[i] == "integrator2_3d_large_v0")
                                ? NN_ROBOT_LARGE
                                : NN_ROBOT_SMALL;
      const float *rhoOutput = nn_eval(selfType); // in grams
      rho = rhoOutput[0] / 1000 * 9.81;           // in Newtons
      if (rho < -max_f || rho > max_f)
      {
        std::cout << "fa violations" << std::endl;
        for (auto &k : involved_robots)
        { // create constraints for each involved neighbor
          constraints[k].push_back({t * 0.1, states.at(k)});
        }
        // add the self robot
        constraints[i].push_back({t * 0.1, states.at(i)});
        return true; // as soon as the violation happends
      }
    }
  } // time loop
  return false;
}
void createConstraintsFromConflicts(const Conflict &early_conflict, std::map<size_t, std::vector<dynoplan::Constraint>> &constraints)
{
  constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
  constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
}
void export_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
                      std::ofstream *out)
{
  float cost = 0;
  std::string indent = "  ";
  for (auto &n : solution)
    cost += n.trajectory.cost;
  *out << "cost: " << cost << std::endl;
  *out << "result:" << std::endl;
  for (size_t i = 0; i < solution.size(); ++i)
  {
    std::vector<Eigen::VectorXd> tmp_states = solution[i].trajectory.states;
    std::vector<Eigen::VectorXd> tmp_actions = solution[i].trajectory.actions;
    *out << "-" << std::endl;
    *out << indent << "states:" << std::endl;
    for (size_t j = 0; j < tmp_states.size(); ++j)
    {
      *out << indent << "  - " << tmp_states.at(j).format(dynobench::FMT) << std::endl;
    }
    *out << indent << "actions:" << std::endl;
    for (size_t j = 0; j < tmp_actions.size(); ++j)
    {
      *out << indent << "  - " << tmp_actions.at(j).format(dynobench::FMT) << std::endl;
    }
  }
}

void extract_motion_primitives(dynobench::Problem &problem,
                               MultiRobotTrajectory &multi_robot_opt_out,
                               std::map<std::string, std::vector<dynoplan::Motion>> &robot_motions,
                               const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
                               int len)
{

  size_t robot_idx = 0;
  for (auto robot_trajectory : multi_robot_opt_out.trajectories)
  {
    int idx = 0;
    int total_actions = robot_trajectory.actions.size();
    while (idx < total_actions)
    {
      int num_actions = rand() % 11 + len; // without len it's < 10
      num_actions = std::min(num_actions, (total_actions - idx));
      // make sure the left motion primitive has >= 5 length
      if ((total_actions - (idx + num_actions)) < 5 && idx + num_actions < total_actions)
      {
        num_actions = total_actions - idx;
      }
      dynobench::Trajectory new_trajectory;
      std::vector action_vector(robot_trajectory.actions.begin() + idx, robot_trajectory.actions.begin() + idx + num_actions);
      new_trajectory.actions = action_vector;

      std::vector state_vector(robot_trajectory.states.begin() + idx, robot_trajectory.states.begin() + idx + num_actions + 1);
      // rebase
      Eigen::VectorXd first_state = state_vector[0];
      for (auto &state : state_vector)
      {
        state[0] -= first_state[0];
        state[1] -= first_state[1];
        if (!all_robots[robot_idx]->is_2d)
          state[2] -= first_state[2];
      }
      new_trajectory.states = state_vector;
      dynoplan::Motion new_motion;
      traj_to_motion(new_trajectory, *all_robots[robot_idx], new_motion, /*check collision*/ true);
      new_motion.traj = new_trajectory;
      new_motion.idx = robot_motions[problem.robotTypes[robot_idx]].size();
      robot_motions[problem.robotTypes[robot_idx]].push_back(std::move(new_motion));
      idx += num_actions;
    }
    robot_motions[problem.robotTypes[robot_idx]].shrink_to_fit();
    robot_idx++;
  }
}

bool sanity_check(
    const dynobench::Trajectory &sol1,
    const dynobench::Trajectory &sol2,
    double robot_size = 0.4)
{
  using CollisionGeometry = fcl::CollisionGeometryd;
  using CollisionObject = fcl::CollisionObjectd;

  // --- Create robot shapes depending on type ---
  std::shared_ptr<CollisionGeometry> geom1;
  std::shared_ptr<CollisionGeometry> geom2;

  geom1 = std::make_shared<fcl::Sphered>(robot_size);
  geom2 = std::make_shared<fcl::Sphered>(robot_size);

  CollisionObject obj1(geom1);
  CollisionObject obj2(geom2);

  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  size_t max_t = std::max(sol1.states.size() - 1, sol2.states.size() - 1);

  for (size_t t = 0; t <= max_t; ++t)
  {
    Eigen::VectorXd state1 = (t < sol1.states.size()) ? sol1.states[t] : sol1.states.back();
    Eigen::VectorXd state2 = (t < sol2.states.size()) ? sol2.states[t] : sol2.states.back();

    fcl::Vector3d p1, p2;
    if (state1.size() >= 3 && state2.size() >= 3)
    {
      p1 = fcl::Vector3d(state1[0], state1[1], state1[2]);
      p2 = fcl::Vector3d(state2[0], state2[1], state2[2]);
    }
    else
    {
      p1 = fcl::Vector3d(state1[0], state1[1], 0.0);
      p2 = fcl::Vector3d(state2[0], state2[1], 0.0);
    }

    obj1.setTranslation(p1);
    obj2.setTranslation(p2);

    result.clear();
    fcl::collide(&obj1, &obj2, request, result);

    if (result.isCollision())
    {
      std::cout << "collision at timestep " << t << std::endl;
      return false; // sanity check failed, they collide
    }
  }

  return true; // no collisions detected
}