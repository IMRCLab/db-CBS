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

#define DYNOBENCH_BASE "../dynoplan/dynobench/"
// for debug the cbs-opt and develop. No need to run the discrete search each time
int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile; // problem instance .yaml file
  std::string initFile; // parallel/independent optimization output .yaml file
  std::string discreteSearchFile; // discrete search output .yaml file
  std::string outFile; // output of the optimization .yaml file
  bool sum_robot_cost = true;
  bool feasible = false;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      "init,i", po::value<std::string>(&initFile)->required())(
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
  // doesn't consider car with trailer
  for (const auto &robot : robots){
    collision_geometries.insert(collision_geometries.end(), 
                            robot->collision_geometries.begin(), robot->collision_geometries.end());
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
  MultiRobotTrajectory parallel_multirobot_sol;
  parallel_multirobot_sol.read_from_yaml(initFile.c_str());
  MultiRobotTrajectory discrete_search_sol;
  discrete_search_sol.read_from_yaml(discreteSearchFile.c_str()); 
  // Handle trajectories, prepare for the optimization
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
    HighLevelNodeOptimization tmpNode = tmp;
    int max_conflict_cluster_index;
    int index_i, index_j;
    // i. initialize clusters using conflict mtrx. ONLY in-conflict robots belong to clusters
    for (size_t i = 0; i < num_robots; i++){
      for (size_t j = 0; j <= i; j++){
        if(tmp.conflict_matrix[i][j] > 0){
          tmpNode.clusters.push_back({{i, j}, tmp.conflict_matrix[i][j]}); 
        }
      }
    }
    while(true){
      // ii. extract the MAX conflict cluster
      auto max_conflict_cluster_it = std::max_element(tmpNode.clusters.begin(), tmpNode.clusters.end(),
                                    [](std::pair<std::unordered_set<size_t>, int>& a, std::pair<std::unordered_set<size_t>, int>& b) {
                          return a.second < b.second; }); // compared based on conflicts
      // iii. jointly optimiza the one with MAX conflicts
      std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(6) + ".yaml";
      std::cout << "tmp envFile: " << tmp_envFile << std::endl;
      get_moving_obstacle(envFile, /*initGuess*/tmpNode.multirobot_trajectory, /*outputFile*/tmp_envFile, max_conflict_cluster_it->first, /*moving_obs*/false);
      feasible = execute_optimizationMetaRobot(tmp_envFile,
                              /*initialGuess*/discrete_search_sol, // always from the discrete search
                              /*solution*/tmpNode.multirobot_trajectory, // update the solution
                              DYNOBENCH_BASE,
                              max_conflict_cluster_it->first,
                              sum_robot_cost);
      if(feasible){
        // iv. zero the optimized cluster's conflict, don't remove it
        max_conflict_cluster_index = tmpNode.getIndexOfSet(max_conflict_cluster_it->first);
        tmpNode.clusters.at(max_conflict_cluster_index).second = 0; 
        // v. check for collision, update the collision_matrix
        std::for_each(tmpNode.conflict_matrix.begin(), tmpNode.conflict_matrix.end(), [](std::vector<int>& row) {
            std::fill(row.begin(), row.end(), 0); });
        if (!getConflicts(tmpNode.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, tmpNode.conflict_matrix)){
          std::cout << "No inter-robot conflict" << std::endl;
          tmpNode.multirobot_trajectory.to_yaml_format(outFile.c_str());
          return 0;
        }
        // vi. the max conflict happening in the output, extract this pair
        auto [i, j, max_conflict] = tmpNode.getMaxElement(); // row, column, max conflict
        // none of them belong to any cluster
        index_i = tmpNode.containsX(i); // which element in clusters
        index_j = tmpNode.containsX(j);
        if(index_i < 0 && index_j < 0){
          tmpNode.clusters.push_back({{i, j}, tmp.conflict_matrix[i][j]}); 
        }
        // one of them belong to some cluster
        else if(index_i >= 0 || index_j >= 0){
          if(index_i >= 0){
            tmpNode.clusters.at(index_i).first.insert(j);
            tmpNode.clusters.at(index_i).second = max_conflict;
          }
          else{
            tmpNode.clusters.at(index_j).first.insert(i);
            tmpNode.clusters.at(index_j).second = max_conflict;
          }
        }
        // both belong to some cluster
        else {
          tmpNode.clusters.at(index_i).first.insert(tmpNode.clusters.at(index_j).first.begin(), tmpNode.clusters.at(index_j).first.end());
          tmpNode.clusters.at(index_i).second = std::max(tmpNode.clusters.at(index_i).second, max_conflict);
        }
      }
    }
  }
  else {
    int opt_id = 1;
    // check for collision and create clusters from this Node's solution outside the loop for the root node to initialize the Open set
    for (size_t i = 0; i < num_robots; i++){
      for (size_t j = 0; j <= i; j++){
        // create cluster for each on-collision pairs
        if(tmp.conflict_matrix[i][j] > 0){
          HighLevelNodeOptimization startNode = tmp;
          startNode.id = opt_id;
          // Zeroing out all elements
          std::for_each(startNode.conflict_matrix.begin(), startNode.conflict_matrix.end(), [](std::vector<int>& row) {
              std::fill(row.begin(), row.end(), 0);
          });
          startNode.cluster = {i, j};
          startNode.conflict = tmp.conflict_matrix[i][j]; // only the pair we are interested to optimize jointly
          ++opt_id;
          auto handle = open_opt.push(startNode);
          (*handle).handle = handle;
        }
      }
    }
    while(!open_opt.empty()){
      HighLevelNodeOptimization N = open_opt.top();
      std::cout << "open set size: " << open_opt.size() << std::endl;
      std::cout << "best node N.id: " << N.id << " N.conflicts: " << N.conflict << std::endl;
      open_opt.pop();
      std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(6) + ".yaml";
      std::cout << "tmp envFile: " << tmp_envFile << std::endl;
      get_moving_obstacle(envFile, /*initGuess*/N.multirobot_trajectory, /*outputFile*/tmp_envFile, N.cluster);
      // run the optimization for the cluster
      feasible = execute_optimizationMetaRobot(tmp_envFile,
                              /*initialGuess*/discrete_search_sol, // always from the discrete search
                              /*solution*/N.multirobot_trajectory, // update the solution
                              DYNOBENCH_BASE,
                              N.cluster,
                              sum_robot_cost);

      if(feasible){
        // update the cost, max conflict in these trajectories
        N.cost = N.multirobot_trajectory.get_cost();
        if(!getConflicts(N.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, N.conflict_matrix)){
          std::cout << "No inter-robot conflict" << std::endl;
          N.multirobot_trajectory.to_yaml_format(outFile.c_str());
          return 0;
        }
        // create clusters from this Node's updated solution
        for (size_t i = 0; i < num_robots; i++){
          for (size_t j = 0; j <= i; j++){
            // create cluster for each in-collision pairs
            if(N.conflict_matrix[i][j] > 0){
              HighLevelNodeOptimization newNode = N;
              newNode.id = opt_id;
              // Zeroing out all elements
              std::for_each(newNode.conflict_matrix.begin(), newNode.conflict_matrix.end(), [](std::vector<int>& row) {
                  std::fill(row.begin(), row.end(), 0);
              });
              // check if collision between new pair of robots
              if(newNode.cluster.find(i) == newNode.cluster.end() && newNode.cluster.find(j) == newNode.cluster.end()){
                newNode.cluster = {i, j};
              }
              // check if the collision with/between any of robots of the existing cluster, then merge it
              else if(newNode.cluster.find(i) != newNode.cluster.end() || newNode.cluster.find(j) != newNode.cluster.end()){
                size_t c = newNode.cluster.find(i) == newNode.cluster.end() ? i : j;
                newNode.cluster.insert(c); // add new element
              }
              std::cout << "new Node cluster: " << std::endl;
              for(auto &a : newNode.cluster){
                std::cout << a << std::endl;
              }
              newNode.conflict = N.conflict_matrix[i][j]; // only the pair we are interested to optimize jointly later while picking from Open
              auto handle = open_opt.push(newNode);
              (*handle).handle = handle;
              ++opt_id;

            }
          }
        }
      } 
    } 
  }
  

}

// when I optimize each cluster inside the loop, trajectory
  