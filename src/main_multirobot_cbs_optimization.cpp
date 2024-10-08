#include "dynobench/multirobot_trajectory.hpp"
#include <algorithm>
#include <bits/stdc++.h>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <dynoplan/optimization/multirobot_optimization.hpp>
// #include <dynoplan/optimization/ocp.hpp>
#include "dynobench/motions.hpp"
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/ocp.hpp>
#include <string>
#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "dbcbs_utils.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/tdbastar_epsilon.hpp"
#include "dynoplan/tdbastar/planresult.hpp"

using namespace dynoplan;
#define DYNOBENCH_BASE "../dynoplan/dynobench/"
// for debug the cbs-opt and develop. No need to run the discrete search each time
int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile; // problem instance .yaml file
  // std::string initFile; // parallel/independent optimization output .yaml file
  std::string discreteSearchFile; // discrete search output .yaml file
  std::string outFile; // output of the optimization .yaml file
  bool sum_robot_cost = true;
  bool feasible = false;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      // "init,i", po::value<std::string>(&initFile)->required())(
      "discrete,d", po::value<std::string>(&discreteSearchFile)->required())(
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

  // Create the problem, robots, robot_objs, col_mng
  dynobench::Problem problem(envFile);
  std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
  problem.models_base_path = models_base_path;
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  for (const auto &robotType : problem.robotTypes){
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
                (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
        robots.push_back(robot);
  }
  std::vector<std::shared_ptr<fcl::CollisionGeometryd>> collision_geometries;
  std::vector<fcl::CollisionObjectd*> robot_objs;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
  col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  size_t col_geom_id = 0;
  col_mng_robots->setup();
  size_t i = 0;
  Eigen::Vector3d radii = Eigen::Vector3d(.12, .12, .3);
  // doesn't consider car with trailer
  for (const auto &robot : robots){
    collision_geometries.insert(collision_geometries.end(), 
                            robot->collision_geometries.begin(), robot->collision_geometries.end());
    // collision_geometries.push_back(std::make_shared<fcl::Ellipsoidd>(radii));
    auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
    collision_geometries[col_geom_id]->setUserData((void*)i);
    robot_objs.push_back(robot_obj);
    col_geom_id++;
    i++;
  }
  col_mng_robots->registerObjects(robot_objs);
  size_t num_robots = robots.size();
  bool only_max = true;
  // read given files
  MultiRobotTrajectory discrete_search_sol;
  discrete_search_sol.read_from_yaml(discreteSearchFile.c_str()); 
  // Handle trajectories, prepare for the optimization
  // I. Parallel/Independent optimization
  auto optimization_start = std::chrono::high_resolution_clock::now();
  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0; // 1 - time optimal, no moving obstacles 
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 400;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = true; 
  MultiRobotTrajectory parallel_multirobot_sol;
  parallel_multirobot_sol.trajectories.resize(num_robots);
  // since homogen. robots
  Result_opti opti_out;
  dynobench::Problem tmp_problem;
  tmp_problem.models_base_path = problem.models_base_path;
  tmp_problem.robotType = problem.robotTypes.at(0);
  tmp_problem.p_lb = problem.p_lb;
  tmp_problem.p_ub = problem.p_ub;
  tmp_problem.robotTypes.push_back(problem.robotTypes.at(0)); 
  tmp_problem.obstacles = problem.obstacles;
  
  for (size_t i = 0; i < num_robots; i++){
    tmp_problem.goal = problem.goals[i];
    tmp_problem.start = problem.starts[i];
    opti_out.success = false;
    trajectory_optimization(tmp_problem, discrete_search_sol.trajectories.at(i), options_trajopt, parallel_multirobot_sol.trajectories.at(i),
                opti_out);
    if(!opti_out.success) 
      std::cout << "failure of parallel/independent optimization for robot " << i << std::endl;
  }
  parallel_multirobot_sol.to_yaml_format("/tmp/dynoplan/parallel_multirobot_sol.yaml");
  // for the optimization, augment the discrete search with f
  MultiRobotTrajectory discrete_search_sol_fa;
  discrete_search_sol_fa.read_from_yaml("/tmp/dynoplan/result_dbecbs_fa.yaml");
  // start the greedy optimization part
  typename boost::heap::d_ary_heap<HighLevelNodeOptimization, boost::heap::arity<2>,
                                        boost::heap::mutable_<true> > open_opt;
  HighLevelNodeOptimization tmp (num_robots, num_robots);
  tmp.multirobot_trajectory = parallel_multirobot_sol;
  tmp.cost = parallel_multirobot_sol.get_cost();
  if (!getConflicts(tmp.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, tmp.conflict_matrix)){
    std::cout << "No inter-robot conflict in the root node" << std::endl;
    tmp.multirobot_trajectory.to_yaml_format(outFile.c_str());
    return 0;
  }
  bool greedy_cbs = true;
  if(greedy_cbs){
    std::vector<int> cluster_tracking(num_robots + 1, 0);
    HighLevelNodeOptimization tmpNode = tmp;
    int max_conflict_cluster_index;
    int index_i, index_j;
    // i. initialize clusters using conflict mtrx. ONLY in-conflict robots belong to clusters. Merge already
    std::cout << "initializing the tmpNode clusters" << std::endl;
    for (size_t i = 0; i < num_robots; i++){
      for (size_t j = 0; j <= i; j++){
        if(tmp.conflict_matrix[i][j] > 0){
          tmpNode.clusters.push_back({{i, j}, tmp.conflict_matrix[i][j]}); 
          std::cout << "(" << i << " " << j << " " << "conflict value: " << tmp.conflict_matrix[i][j] << ")" <<  std::endl;
        }
      }
    }
    while(true){
      // ii. extract the MAX conflict cluster
      auto max_conflict_cluster_it = std::max_element(tmpNode.clusters.begin(), tmpNode.clusters.end(),
                                    [](std::pair<std::unordered_set<size_t>, int>& a, std::pair<std::unordered_set<size_t>, int>& b) {
                          return a.second < b.second; }); // compared based on conflicts
      // DEBUG
      if(max_conflict_cluster_it->first.size())
        cluster_tracking.at(max_conflict_cluster_it->first.size()) += 1;
      std::cout << "max cluster elements: ";
      for (const auto& elem : max_conflict_cluster_it->first) {
        std::cout << elem << " ";
      }
      std::cout << "\nconflict value: " << max_conflict_cluster_it->second << std::endl;
      // iii. jointly optimiza the one with MAX conflicts
      std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(6) + ".yaml";
      create_dir_if_necessary(tmp_envFile);
      std::cout << "tmp envFile: " << tmp_envFile << std::endl;
      // robots become integrator2_3d_res_v0
      get_moving_obstacle(envFile, /*initGuess*/tmpNode.multirobot_trajectory, /*outputFile*/tmp_envFile, max_conflict_cluster_it->first, /*moving_obs*/true, /*res force*/true);
      feasible = execute_optimizationMetaRobot(tmp_envFile,
                              /*initialGuess*/discrete_search_sol_fa, // always from the discrete search with fa
                              /*solution*/tmpNode.multirobot_trajectory, // update the solution
                              DYNOBENCH_BASE,
                              max_conflict_cluster_it->first,
                              sum_robot_cost,
                              /*residual force*/true);
      if(feasible){
        // iv. zero the optimized cluster's conflict, don't remove it
        max_conflict_cluster_index = tmpNode.getIndexOfSet(max_conflict_cluster_it->first);
        tmpNode.clusters.at(max_conflict_cluster_index).second = 0; 
        // v. check for collision, update the collision_matrix
        std::for_each(tmpNode.conflict_matrix.begin(), tmpNode.conflict_matrix.end(), [](std::vector<int>& row) {
            std::fill(row.begin(), row.end(), 0); });
        if (!getConflicts(tmpNode.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, tmpNode.conflict_matrix)){
          auto optimization_end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> opt_duration = optimization_end - optimization_start;
          std::cout << "Time taken for optimization: " << opt_duration.count() << " seconds" << std::endl;
          std::cout << "No inter-robot conflict" << std::endl;
          tmpNode.multirobot_trajectory.to_yaml_format(outFile.c_str());
          if(!cluster_tracking.empty()){
            std::ofstream fout(outFile, std::ios::app); 
            fout << "cluster_tracking:" << std::endl;
            for (auto &c: cluster_tracking){
              fout << "  - " << c << std::endl;
            }
          }
          bool joint_opt = true;
          if(joint_opt){
            MultiRobotTrajectory joint_opt_sol;
            joint_opt_sol.trajectories.resize(num_robots);
            joint_opt_sol.trajectories = tmpNode.multirobot_trajectory.trajectories;
            MultiRobotTrajectory multirobot_trajectory;
            multirobot_trajectory.trajectories.resize(num_robots);
            multirobot_trajectory.trajectories = tmpNode.multirobot_trajectory.trajectories;
            std::unordered_set<size_t> joint_cluster;
            for (size_t i = 0; i < num_robots; ++i) {
              joint_cluster.insert(i);
              for(auto &state : multirobot_trajectory.trajectories.at(i).states){
                state.conservativeResize(state.size() + 1); // Resize to fit new element
                state(state.size() - 1) = 0; // add the residual
              }
            }
            std::cout << "nxs: " << multirobot_trajectory.get_nxs().size() << std::endl;
             std::string joint_opt_envFile = "/tmp/dynoplan/joint_opt_envFile.yaml";
            std::cout << "joint_opt_envFile: " << joint_opt_envFile << std::endl;
            get_moving_obstacle(envFile, /*initGuess*/multirobot_trajectory, /*outputFile*/joint_opt_envFile, 
                        /*cluster*/joint_cluster, /*moving_obs*/false, /*residual force*/true);
            auto joint_optimization_start = std::chrono::high_resolution_clock::now();
            bool joint_feasible = execute_optimizationMetaRobot(joint_opt_envFile, //all robots with augmented state
                                  /*initialGuess*/multirobot_trajectory, //conflict free optimized solution
                                  /*solution*/joint_opt_sol, // expected not to be empty, proper size/no augmentation
                                  DYNOBENCH_BASE,
                                  joint_cluster,
                                  sum_robot_cost,
                                  /*residual force*/true);
            if(joint_feasible){
              auto joint_optimization_end = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> joint_opt_duration = joint_optimization_end - joint_optimization_start;
              std::cout << "Time taken for joint optimization: " << joint_opt_duration.count() << " seconds" << std::endl;
              joint_opt_sol.to_yaml_format("/tmp/dynoplan/joint_opt_sol.yaml");
            }
          }
          return 0;
        }
        // vi. the max conflict happening in the output, extract this pair
        auto [i, j, max_conflict] = tmpNode.getMaxElement(); // row, column, max conflict
        std::cout << "max conflict between: " << i << ", " << j << ", conflict: " << max_conflict << std::endl;
        // none of them belong to any cluster
        index_i = tmpNode.containsX(i); // which element in clusters
        index_j = tmpNode.containsX(j);
        if(index_i < 0 && index_j < 0){
          std::cout << "creating new cluster" << std::endl;
          tmpNode.clusters.push_back({{i, j}, tmp.conflict_matrix[i][j]}); 
        }
        // both belong to some cluster
        else if(index_i >= 0 && index_j >= 0){
          std::cout << "merging two existing clusters" << std::endl;
          tmpNode.clusters.at(index_i).first.insert(tmpNode.clusters.at(index_j).first.begin(), tmpNode.clusters.at(index_j).first.end());
          tmpNode.clusters.at(index_i).second = std::max(tmpNode.clusters.at(index_i).second, max_conflict);
          if(index_i != index_j)
            tmpNode.clusters.erase(tmpNode.clusters.begin() + index_j); // delete the old one
        }
        // only one belong to some cluster
        else {
          if(index_i >= 0){
            tmpNode.clusters.at(index_i).first.insert(j);
            tmpNode.clusters.at(index_i).second = max_conflict;
            
          }
          else{
            tmpNode.clusters.at(index_j).first.insert(i);
            tmpNode.clusters.at(index_j).second = max_conflict;
          }
          std::cout << "robot " << (index_i >= 0 ? i : j) << " already belongs to some cluster" << std::endl;
        }
      }
    }
  }

}

// when I optimize each cluster inside the loop, trajectory
  