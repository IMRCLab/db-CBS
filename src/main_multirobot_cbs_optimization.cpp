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
  // read given files
  MultiRobotTrajectory parallel_multirobot_sol;
  parallel_multirobot_sol.read_from_yaml(initFile.c_str());
  MultiRobotTrajectory discrete_search_sol;
  discrete_search_sol.read_from_yaml(discreteSearchFile.c_str()); 
  MultiRobotTrajectory multi_out = parallel_multirobot_sol;
  // Handle trajectories, prepare for the optimization
  typename boost::heap::d_ary_heap<HighLevelNodeOptimization, boost::heap::arity<2>,
                                        boost::heap::mutable_<true> > open_opt;
  HighLevelNodeOptimization tmp (num_robots, num_robots);
  tmp.multirobot_trajectory = parallel_multirobot_sol;
  tmp.cost = parallel_multirobot_sol.get_cost();
  tmp.conflict = getConflicts(tmp.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, tmp.conflict_matrix);
  auto handle = open_opt.push(tmp);
  (*handle).handle = handle;
  int opt_id = 1;
  while(!open_opt.empty()){
    std::cout << "Open_opt set" << std::endl;
    for (auto &f : open_opt) {
      std::cout << "id: " << f.id << std::endl;
      std::cout << "conflict number: " << f.conflict << std::endl;
    }
    HighLevelNodeOptimization N = open_opt.top();
    std::cout << "best node N.id: " << N.id << " N.conflicts: " << N.conflict << std::endl;
    open_opt.pop();
    if(N.conflict == 0){
      std::cout << "No inter-robot conflict" << std::endl;
      N.multirobot_trajectory.to_yaml_format(outFile.c_str());
      return 0;
    }
    for (size_t i = 0; i < num_robots; i++){
      for (size_t j = 0; j <= i; j++){
        if(N.conflict_matrix[i][j] > 0){
          std::cout << "collision, clustering " << i << ", " << j << ", " << N.conflict_matrix[i][j] << std::endl;
          feasible = false;
          HighLevelNodeOptimization newNode = N;
          newNode.id = opt_id;
          std::cout << "Node ID is " << opt_id << std::endl;
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
          // get the environment, moving obstacles = non-cluster robots with soft-constrained optimized
          std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(6) + ".yaml";
          get_moving_obstacle(envFile, /*initGuess*/newNode.multirobot_trajectory, /*outputFile*/tmp_envFile, newNode.cluster);
          // run the optimization for the cluster
          feasible = execute_optimizationMetaRobot(/*envFile*/tmp_envFile,
                                  /*initialGuess*/discrete_search_sol, // always from the discrete search
                                  /*solution*/newNode.multirobot_trajectory, // update the solution
                                  DYNOBENCH_BASE,
                                  newNode.cluster,
                                  sum_robot_cost);

          if(feasible){
            // update the cost, max conflict in these trajectories
            newNode.cost = newNode.multirobot_trajectory.get_cost();
            newNode.conflict = getConflicts(newNode.multirobot_trajectory.trajectories, robots, col_mng_robots, robot_objs, newNode.conflict_matrix);
            std::cout << "new Node conflict: " << newNode.conflict << std::endl;
            auto handle = open_opt.push(newNode);
            (*handle).handle = handle;
          }
          ++opt_id;
        } 
      }
    } 
  } // while loop for the optimization

}
