#pragma once

#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <bits/stdc++.h>
#include <string>
// BOOST
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// OTHERS
#include "robots.h"
#include "robotStatePropagator.hpp"
// FCL
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
// DYNOPLAN
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include <dynoplan/optimization/multirobot_optimization.hpp>
// DYNOBENCH
#include <dynobench/multirobot_trajectory.hpp>
#include "dynobench/motions.hpp"
#include "dynobench/nn.h"

// ===================== structs =====================

struct Conflict {
  double time;
  size_t robot_idx_i;
  Eigen::VectorXd robot_state_i;
  size_t robot_idx_j;
  Eigen::VectorXd robot_state_j;
};

struct HighLevelNode {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    std::vector<std::vector<dynoplan::Constraint>> constraints;
    double cost;
    double LB;
    int focalHeuristic;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
        boost::heap::mutable_<true>>::handle_type handle;

    bool operator<(const HighLevelNode& n) const;
};

struct HighLevelNodeFocal {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    std::vector<std::vector<dynoplan::Constraint>> constraints;
    double cost;
    double LB;
    int focalHeuristic;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>,
        boost::heap::mutable_<true>>::handle_type handle;

    bool operator<(const HighLevelNodeFocal& n) const;
};

// heaps
typedef boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>,
    boost::heap::mutable_<true>> openset_t;

typedef openset_t::handle_type openset_handle_type;

struct compareFocalHeuristic {
  bool operator()(const openset_handle_type& h1,
                  const openset_handle_type& h2) const;
};

typedef boost::heap::d_ary_heap<
    openset_handle_type,
    boost::heap::arity<2>,
    boost::heap::compare<compareFocalHeuristic>,
    boost::heap::mutable_<true>> focalset_t;

struct HighLevelNodeOptimization {
    MultiRobotTrajectory multirobot_trajectory;
    std::unordered_set<size_t> cluster;
    std::vector<std::pair<std::unordered_set<size_t>, int>> clusters;
    std::vector<std::vector<int>> conflict_matrix;
    double cost;
    int conflict;
    int id;

    HighLevelNodeOptimization(int rows, int cols);

    int containsX(size_t X) const;
    int getIndexOfSet(std::unordered_set<size_t>& target_set);
    std::tuple<int, int, int> getMaxElement();

    typename boost::heap::d_ary_heap<HighLevelNodeOptimization,
        boost::heap::arity<2>,
        boost::heap::mutable_<true>>::handle_type handle;

    bool operator<(const HighLevelNodeOptimization& n) const;
};

struct Obstacle {
    std::vector<double> center;
    std::vector<double> size;
    std::string type;
    std::string octomap_file = "";
};

// ===================== utility functions =====================
bool getEarliestViolations(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    std::vector<std::string>& robot_types,
    std::map<size_t, std::vector<dynoplan::Constraint>>& constraints);

bool getEarliestConflict(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd*>& robot_objs,
    Conflict& early_conflict);

void createConstraintsFromConflicts(
    const Conflict& early_conflict,
    std::map<size_t, std::vector<dynoplan::Constraint>>& constraints);

void export_solutions(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    std::ofstream* out);

void extract_motion_primitives(
    dynobench::Problem& problem,
    MultiRobotTrajectory& multi_robot_opt_out,
    std::map<std::string, std::vector<dynoplan::Motion>>& robot_motions,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    int len);

// ===================== timing / stats =====================

using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point& start_time);

bool fail_and_write_stats(
    std::ofstream& stats,
    const std::string& planner_name,
    const std::string& instanceName,
    const std::string& reason);