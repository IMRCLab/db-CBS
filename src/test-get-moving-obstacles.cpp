#include <iostream>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include "dynobench/motions.hpp"
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/multirobot_optimization.hpp>
#include <dynoplan/optimization/ocp.hpp>
#include <string>
#include <vector>
#include <bits/stdc++.h>

#define DYNOBENCH_BASE "../dynoplan/dynobench/"

struct Obstacle {
    std::vector<double> center;
    std::vector<double> size;
    std::string type;
};

// Convert Obstacle struct to YAML node
YAML::Node to_yaml(const Obstacle& obs) {
    YAML::Node node;
    node["center"] = obs.center;
    node["size"] = obs.size;
    node["type"] = obs.type;
    return node;
}

void output_to_moving_obs(const std::string &env_file,
                        const std::string &initial_guess_file){
  // custom params
  double size = 0.5;
  std::string type = "sphere";
  size_t current_robot_idx = 0; // which robot is considered for optimization
  YAML::Node env = YAML::LoadFile(env_file);
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  // read the result
  YAML::Node initial_guess = YAML::LoadFile(initial_guess_file);
  size_t num_robots = initial_guess["result"].size();
  const auto &start = env["robots"][current_robot_idx]["start"];
  const auto &goal = env["robots"][current_robot_idx]["goal"];
  const auto &robot_type = env["robots"][current_robot_idx]["type"];
  // data
  YAML::Node data;
  data["environment"]["max"] = env_max;
  data["environment"]["min"] = env_min;
  // start, goal of the robot 
  YAML::Node robot_node;
  robot_node["start"] = start;
  robot_node["goal"] = goal;
  robot_node["type"] = robot_type;
  data["robots"].push_back(robot_node);

  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());
  size_t max_t = 0;
  size_t robot_idx = 0;
  for (const auto& traj : init_guess_multi_robot.trajectories){
      if(robot_idx != current_robot_idx){
          max_t = std::max(max_t, traj.states.size() - 1);
      }
      ++robot_idx;
  }
  YAML::Node moving_obstacles_node; // for all robots
  Eigen::VectorXd state;
  std::vector<Obstacle> moving_obs_per_time;
  std::vector<std::vector<Obstacle>> moving_obs;
  for (size_t t = 0; t <= max_t; ++t){ 
    moving_obs_per_time.clear();
    for (size_t i = 0; i < num_robots; i++){
      if (i != current_robot_idx){
          if (t >= init_guess_multi_robot.trajectories.at(i).states.size()){
              state = init_guess_multi_robot.trajectories.at(i).states.back();    
          }
          else {
              state = init_guess_multi_robot.trajectories.at(i).states[t];
          }
        // into vector
        Obstacle obs;
        obs.center = {state(0), state(1), state(2)};
        obs.size = {size};
        obs.type = "sphere";
        moving_obs_per_time.push_back({obs});
      }
    }
    moving_obs.push_back(moving_obs_per_time);
  }
  for (const auto &obs_list : moving_obs) {
      YAML::Node yaml_obs_list;
      for (const auto &obs : obs_list) {
          yaml_obs_list.push_back(to_yaml(obs));
      }
      data["environment"]["moving_obstacles"].push_back(yaml_obs_list);
  }
  // Write YAML node to file
  std::ofstream fout("moving_obs_swap4_drone.yaml");
  fout << data;
  fout.close();
}

// mrs case
void get_artificial_env(const std::string &env_file,
                        const std::string &initial_guess_file,
                        const std::string &out_file,
                        const std::unordered_set<size_t> &cluster){
  // custom params for the obstacle
  double size = 0.5;
  std::string type = "sphere";
  YAML::Node env = YAML::LoadFile(env_file);
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  // data
  YAML::Node data;
  data["environment"]["max"] = env_max;
  data["environment"]["min"] = env_min;
  // read the result
  YAML::Node initial_guess = YAML::LoadFile(initial_guess_file);
  size_t num_robots = initial_guess["result"].size();

  for (size_t i = 0; i < num_robots; i++){
    if (cluster.find(i) != cluster.end()){ //  robots that are within cluster
      YAML::Node robot_node;
      robot_node["start"] = env["robots"][i]["start"];
      robot_node["goal"] = env["robots"][i]["goal"];
      robot_node["type"] = env["robots"][i]["type"];
      data["robots"].push_back(robot_node);
    }
    
  }

  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());
  size_t max_t = 0;
  size_t robot_idx = 0;
  for (const auto& traj : init_guess_multi_robot.trajectories){
      // if(cluster.find(robot_idx) == cluster.end()){
        max_t = std::max(max_t, traj.states.size() - 1);
      // }
      ++robot_idx;
  }
  YAML::Node moving_obstacles_node; // for all robots
  Eigen::VectorXd state;
  std::vector<Obstacle> moving_obs_per_time;
  std::vector<std::vector<Obstacle>> moving_obs;
  std::cout << "MAXT: " << max_t << std::endl;
  for (size_t t = 0; t <= max_t; ++t){ 
    moving_obs_per_time.clear();
    for (size_t i = 0; i < num_robots; i++){
      if (cluster.find(i) == cluster.end()){
          if (t >= init_guess_multi_robot.trajectories.at(i).states.size()){
              state = init_guess_multi_robot.trajectories.at(i).states.back();    
          }
          else {
              state = init_guess_multi_robot.trajectories.at(i).states[t];
          }
        // into vector
        Obstacle obs;
        obs.center = {state(0), state(1), state(2)};
        obs.size = {size};
        obs.type = "sphere";
        moving_obs_per_time.push_back({obs});
      }
    }
    moving_obs.push_back(moving_obs_per_time);
  }
  for (const auto &obs_list : moving_obs) {
      YAML::Node yaml_obs_list;
      for (const auto &obs : obs_list) {
          yaml_obs_list.push_back(to_yaml(obs));
      }
      data["environment"]["moving_obstacles"].push_back(yaml_obs_list);
  }
  // Write YAML node to file
  std::ofstream fout(out_file);
  fout << data;
  fout.close();
}

int main(int argc, char* argv[]){
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    std::string envFile; 
    std::string inputFile; 
    std::string outputFile;

    desc.add_options()
      ("help", "produce help message")
      ("env,e", po::value<std::string>(&envFile)->required(), "problem instance file (yaml)")
      ("input,i", po::value<std::string>(&inputFile)->required(), "initial guess file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)");
    try {
      po::variables_map vm;
      po::store(po::parse_command_line(argc, argv, desc), vm);
      po::notify(vm);

      if (vm.count("help") != 0u) {
        std::cout << desc << "\n";
        return 0;
      }
    } catch (po::error& e) {
      std::cerr << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return 1;
    }
    std::cout << envFile << std::endl;
    // output_to_moving_obs(envFile, inputFile);
    std::unordered_set<size_t> cluster_test = {2,3};
    get_artificial_env(envFile, inputFile, outputFile, cluster_test);

    return 0;
}