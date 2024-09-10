#include <algorithm>
#include <bits/stdc++.h>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <string>
#include "dbcbs_utils.hpp"
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/tdbastar_epsilon.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
#include <dynobench/multirobot_trajectory.hpp>

// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

using namespace dynoplan;
#define DYNOBENCH_BASE "../dynoplan/dynobench/"

int main(int argc, char *argv[]){
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile;
  std::string discreteSearchFile;
  std::string outFile;
  bool sum_robots_cost = true;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      "discrete,i", po::value<std::string>(&discreteSearchFile)->required())(
      "out,o", po::value<std::string>(&outFile)->required()); 

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  MultiRobotTrajectory discrete_search_sol;
  discrete_search_sol.read_from_yaml(discreteSearchFile.c_str());
  size_t num_robots = discrete_search_sol.trajectories.size();
  MultiRobotTrajectory multirobot_sol; // for the priority optimization
  multirobot_sol.trajectories.resize(num_robots);
  auto pr_opt_start = std::chrono::high_resolution_clock::now();
  Options_trajopt pr_opt_traj;
  pr_opt_traj.solver_id = 0;
  pr_opt_traj.control_bounds = 1;
  pr_opt_traj.use_warmstart = 1;
  pr_opt_traj.weight_goal = 400;
  pr_opt_traj.max_iter = 50;
  pr_opt_traj.soft_control_bounds = true; 
  std::unordered_set<size_t> other_robots; 
  
  for(size_t i = 0; i < num_robots; i++){
    if(i > 0)
      other_robots.insert(i-1); // skip the first robot, no moving obstacles for it
    // 1. get the environment (use the tmpNode.solution=already optimized to create moving obstacles)
    std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(5) + ".yaml";
    create_dir_if_necessary(tmp_envFile);
    get_desired_moving_obstacles(envFile, multirobot_sol,/*robot_idx_sol*/discrete_search_sol.trajectories.at(i),
                                /*outFile*/tmp_envFile, i, other_robots);
    // 2. create the problem
    dynobench::Problem tmp_problem(tmp_envFile);
    tmp_problem.models_base_path = DYNOBENCH_BASE "models/";
    // 3. run the trajectory optimization
    Result_opti tmp_opti_out;
    trajectory_optimization(tmp_problem, discrete_search_sol.trajectories.at(i), pr_opt_traj, 
                            multirobot_sol.trajectories.at(i), tmp_opti_out);
    if(!tmp_opti_out.success)
      std::cout << "Priority-based optimization failed for robot: " << i << std::endl;
  }
  std::cout << "Priority-based optimization is done." << std::endl;
  multirobot_sol.to_yaml_format(outFile.c_str());
  auto pr_opt_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> pr_opt_duration = pr_opt_end - pr_opt_start;
  std::cout << "Time taken for the priority-based optimization: " << pr_opt_duration.count() << " seconds" << std::endl;

  return 0;
}