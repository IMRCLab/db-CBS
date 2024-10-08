#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <bits/stdc++.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
// #include "planresult.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/multirobot_optimization.hpp>
#include "dynobench/motions.hpp"
#include <tuple>

// Conflicts 
struct Conflict {
  double time;
  size_t robot_idx_i;
  Eigen::VectorXd robot_state_i;
  size_t robot_idx_j;
  Eigen::VectorXd robot_state_j;
};
// Constraints
struct HighLevelNode {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    std::vector<std::vector<dynoplan::Constraint>> constraints;
    double cost; 
    double LB;
    int focalHeuristic;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type // openset_handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
};

struct HighLevelNodeFocal {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    std::vector<std::vector<dynoplan::Constraint>> constraints;
    std::vector<std::vector<std::pair<std::shared_ptr<dynoplan::AStarNode>, size_t>>> result;
    // std::vector<std::map<size_t, dynoplan::Motion*>> result_motions;
    double cost; 
    double LB;
    int focalHeuristic;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type // openset_handle_type
        handle;

    bool operator<(const HighLevelNodeFocal& n) const {
      return cost > n.cost;
    }
};

 // fefine your binary heap type
typedef boost::heap::d_ary_heap<HighLevelNodeFocal, boost::heap::arity<2>, boost::heap::mutable_<true>> openset_t;
// access its handle_type
typedef openset_t::handle_type openset_handle_type;
// create binary heap of handle_type (nested type) of openset_t binary heap
// typedef boost::heap::d_ary_heap<openset_handle_type, boost::heap::arity<2>, boost::heap::mutable_<true>> focalset_t;
struct compareFocalHeuristic { 
  bool operator()(const openset_handle_type& h1,
                  const openset_handle_type& h2) const{
  if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
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
struct HighLevelNodeOptimization {
    // std::vector<dynobench::Trajectory> trajectories;
    MultiRobotTrajectory multirobot_trajectory;
    std::unordered_set<size_t> cluster; // robot idx for the joint optimization
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
          id(0) {
    }
    // check if the robot belongs to any cluster, return index of the clusters vector index
    int containsX(size_t X) const {
      int index = 0;
      for (const auto& pair : clusters) {
        if (pair.first.find(X) != pair.first.end()) {
            return index;
        }
        ++index;
      }
      return -1;
    }

    int getIndexOfSet(std::unordered_set<size_t>& target_set) {
        auto it = std::find_if(clusters.begin(), clusters.end(),
            [&](const std::pair<std::unordered_set<size_t>, int>& pair) {
                return pair.first == target_set; // Compare unordered_sets
            });

        if (it != clusters.end()) {
            return std::distance(clusters.begin(), it); // Get the index
        }
        return -1; 
    }
    // return the indices (row, column) of 2D matrix the max element
    std::tuple<int, int, int> getMaxElement() {
      int max_value = std::numeric_limits<int>::min();
      int max_vector_index = -1;
      int max_element_index = -1;

      for (size_t i = 0; i < conflict_matrix.size(); ++i) {
        auto max_it = std::max_element(conflict_matrix[i].begin(), conflict_matrix[i].end());
        if (max_it != conflict_matrix[i].end()) {
          int current_max_value = *max_it;
          if (current_max_value > max_value) {
              max_value = current_max_value;
              max_vector_index = i;
              max_element_index = std::distance(conflict_matrix[i].begin(), max_it);
          }
        }
      }
      return {max_vector_index, max_element_index, max_value};
    }
    typename boost::heap::d_ary_heap<HighLevelNodeOptimization, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNodeOptimization& n) const {
      return conflict < n.conflict; // max
    }
};

bool getEarliestConflict(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd*>& robot_objs,
    Conflict& early_conflict){
    size_t max_t = 0;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.states.size() - 1);
    }
    Eigen::VectorXd node_state;
    std::vector<Eigen::VectorXd> node_states;
    
    for (size_t t = 0; t <= max_t; ++t){
        node_states.clear();
        size_t robot_idx = 0;
        size_t obj_idx = 0;
        std::vector<fcl::Transform3d> ts_data;
        for (auto &robot : all_robots){
          if (t >= solution[robot_idx].trajectory.states.size()){
              node_state = solution[robot_idx].trajectory.states.back();    
          }
          else {
              node_state = solution[robot_idx].trajectory.states[t];
          }
          node_states.push_back(node_state);
          std::vector<fcl::Transform3d> tmp_ts(1);
          if (robot->name == "car_with_trailers") {
            tmp_ts.resize(2);
          }
          robot->transformation_collision_geometries(node_state, tmp_ts);
          ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
          // ts_data.insert(ts_data.end(), tmp_ts.back()); // just trailer
          ++robot_idx;
        }
        for (size_t i = 0; i < ts_data.size(); i++) {
          fcl::Transform3d &transform = ts_data[i];
          robot_objs[obj_idx]->setTranslation(transform.translation());
          robot_objs[obj_idx]->setRotation(transform.rotation());
          robot_objs[obj_idx]->computeAABB();
          ++obj_idx;
        }
        col_mng_robots->update(robot_objs);
        fcl::DefaultCollisionData<double> collision_data;
        col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<double>);
        if (collision_data.result.isCollision()) {
            assert(collision_data.result.numContacts() > 0);
            const auto& contact = collision_data.result.getContact(0);

            early_conflict.time = t * all_robots[0]->ref_dt;
            early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
            early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
            assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
            early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
            early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
            std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;

// #ifdef DBG_PRINTS
//             std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;
//             auto si_i = all_robots[early_conflict.robot_idx_i]->getSpaceInformation();
//             si_i->printState(early_conflict.robot_state_i);
//             auto si_j = all_robots[early_conflict.robot_idx_j]->getSpaceInformation();
//             si_j->printState(early_conflict.robot_state_j);
// #endif
            return true;
        } 
    }
    return false;
}

void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<dynoplan::Constraint>>& constraints){
    constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
    constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
}
// assumes if residual force, then add 0 to all robots, only homogeneous robots for now
void export_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, 
                      std::ofstream *out, bool residual_forse = false){
    float cost = 0;
    std::string indent = "  ";
    for (auto& n : solution)
      cost += n.trajectory.cost;
    *out << "cost: " << cost << std::endl; 
    *out << "result:" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
      std::vector<Eigen::VectorXd> tmp_states = solution[i].trajectory.states;
      std::vector<Eigen::VectorXd> tmp_actions = solution[i].trajectory.actions;
      *out << "-" << std::endl;
      *out << indent << "states:" << std::endl;
      for (size_t j = 0; j < tmp_states.size(); ++j){
        if(residual_forse){
          tmp_states.at(j).conservativeResize(tmp_states.at(j).size() + 1);
          tmp_states.at(j)(tmp_states.at(j).size() - 1) = 0;
        }
        *out << indent << "  - " << tmp_states.at(j).format(dynobench::FMT) << std::endl;
      }
      *out << indent << "actions:" << std::endl;
      for (size_t j = 0; j < tmp_actions.size(); ++j){
          *out << indent << "  - " << tmp_actions.at(j).format(dynobench::FMT) << std::endl;
      }
    }
}

void export_intermediate_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, 
                      std::vector<std::vector<dynoplan::Constraint>> constraints, const Conflict& early_conflict, std::ofstream *out){
    float cost = 0;
    for (auto& n : solution)
      cost += n.trajectory.cost;

    size_t all_constraints = 0;
    for (auto& c : constraints){
      all_constraints += c.size(); // for each robot vector of constraints
    }
    *out << "cost: " << cost << std::endl; 
    *out << "result:" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
        std::vector<Eigen::VectorXd> tmp_states = solution[i].trajectory.states;
        std::vector<Eigen::VectorXd> tmp_actions = solution[i].trajectory.actions;
        *out << "  - states:" << std::endl;
        for (size_t j = 0; j < tmp_states.size(); ++j){
            *out << "      - ";
            *out << tmp_states.at(j).format(dynobench::FMT)<< std::endl;
        }
        *out << "    actions:" << std::endl;
        for (size_t j = 0; j < tmp_actions.size(); ++j){
            *out << "      - ";
            *out << tmp_actions.at(j).format(dynobench::FMT)<< std::endl;
            
        }
    }
    // constraints of the node
    *out << "constraints: " << all_constraints << std::endl;
    // write conflicts
    *out << "conflict:" << std::endl;
    *out << "    time:" << std::endl;
    *out << "      - ";
    *out << early_conflict.time << std::endl;
    // only one state can be used for visualization
    *out << "    states:" << std::endl;
    *out << "      - ";
    *out << early_conflict.robot_state_i.format(dynobench::FMT) << std::endl;
    *out << "      - ";
    *out << early_conflict.robot_state_j.format(dynobench::FMT) << std::endl;

}

void export_constraints(const std::vector<std::vector<dynoplan::Constraint>> &final_constraints,
                        std::ofstream *out) {
  *out << "constraints:" << std::endl;
  // for (const auto& c : final_constraints){
  for (size_t j = 0; j < final_constraints.size(); j++){ 
    if (final_constraints[j].size() > 0){
      const auto& c = final_constraints[j];
      *out << "  - robot_id: " << j << std::endl;
      *out << "    states:" << std::endl;
      for (size_t i = 0; i < c.size(); i ++){
          if (c[i].constrained_state.size() > 0)
            *out << "      - ";
            *out << c[i].constrained_state.format(dynobench::FMT) << std::endl;
      }
      *out << "    time:" << std::endl;
      for (size_t i = 0; i < c.size(); i ++){
          if (c[i].constrained_state.size() > 0)
            *out << "      - ";
            *out << c[i].time << std::endl;
      }
    }
  }
}

// meta-robot related additions
struct Obstacle {
    std::vector<double> center;
    std::vector<double> size;
    std::string type;
    std::string octomap_file = "";
};

// Convert Obstacle struct to YAML node
YAML::Node obstacle_to_yaml(const Obstacle& obs) {
    YAML::Node node;
    node["center"] = obs.center;
    node["size"] = obs.size;
    node["type"] = obs.type;
    node["octomap_file"] = obs.octomap_file;
    return node;
}
// for moving obstacles META-robot. Joint robots become "integrator2_3d_res_v0", and start/goal augments
// moving obstacles have Ellipsoid shape
void get_moving_obstacle(const std::string &env_file,
                        // const std::string &initial_guess_file,
                        MultiRobotTrajectory init_guess_multi_robot,
                        const std::string &out_file,
                        std::unordered_set<size_t> &cluster,
                        bool moving_obstacles = false,
                        bool residual_force = false){
  // custom params for the obstacle
  double radius = 0.1; // radius
  Eigen::Vector3d radii = Eigen::Vector3d(.12, .12, .3); // from tro paper
  std::string type = "sphere";
  YAML::Node env = YAML::LoadFile(env_file);
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  // data
  YAML::Node data;
  data["environment"]["max"] = env_max;
  data["environment"]["min"] = env_min;
  // read the result
  // YAML::Node initial_guess = YAML::LoadFile(initial_guess_file);
  // size_t num_robots = initial_guess["result"].size();
  size_t num_robots = init_guess_multi_robot.trajectories.size();
  for (size_t i = 0; i < num_robots; i++){
    if (cluster.find(i) != cluster.end()){ // robots that are within cluster
      YAML::Node robot_node;
      if(residual_force){
        // augment the start/goal
        env["robots"][i]["start"].push_back(0);
        env["robots"][i]["goal"].push_back(0);
        robot_node["type"] = "integrator2_3d_res_v0"; // env["robots"][i]["type"];
      }
      else
        robot_node["type"] = env["robots"][i]["type"];

      robot_node["start"] = env["robots"][i]["start"];
      robot_node["goal"] = env["robots"][i]["goal"];
      data["robots"].push_back(robot_node);
    }
    
  }
  // static obstacles
  if(env["environment"]["obstacles"]){
    for (const auto &obs : env["environment"]["obstacles"]) {
      YAML::Node obs_node;
      std::string octomap_filename;
      if (obs["type"].as<std::string>() == "octomap") {
        obs_node["center"] = YAML::Node(YAML::NodeType::Sequence);  // Empty list
        obs_node["size"] = YAML::Node(YAML::NodeType::Sequence);  // Empty list
        obs_node["octomap_file"] = obs["octomap_file"];
        obs_node["type"] = "octomap";
      }
      else {
        obs_node["center"] = obs["center"];
        obs_node["size"] = obs["size"];
        obs_node["type"] = obs["type"];
      } 
      data["environment"]["obstacles"].push_back(obs_node); // if no moving obs, but clusters
    }
  }
  
  if(moving_obstacles){
    size_t max_t = 0;
    size_t index = 0;
    for (const auto& traj : init_guess_multi_robot.trajectories){
      max_t = std::max(max_t, traj.states.size() - 1); // among all paths, optimization needs the longest traj
      ++index;
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
          obs.size = {radius}; // {radii(0), radii(1), radii(2)};
          obs.type = "sphere"; // "ellipsoid";
          moving_obs_per_time.push_back({obs});
        }
      }
      // static obstacles per timestamp
      for (const auto &obs : env["environment"]["obstacles"]) {
        Obstacle octomap_obs;
        std::string octomap_filename;
        if (obs["type"].as<std::string>() == "octomap") {
          octomap_obs.center = {};
          octomap_obs.size = {};
          octomap_obs.octomap_file = obs["octomap_file"].as<std::string>();
          octomap_obs.type = "octomap";
          moving_obs_per_time.push_back({octomap_obs});
        }
      }
      moving_obs.push_back(moving_obs_per_time);
    }
    for (const auto &obs_list : moving_obs) {
      YAML::Node yaml_obs_list;
      for (const auto &obs : obs_list) {
        yaml_obs_list.push_back(obstacle_to_yaml(obs));
      }
      data["environment"]["moving_obstacles"].push_back(yaml_obs_list);
    }
  }

  // Write YAML node to file
  std::ofstream fout(out_file);
  fout << data;
  fout.close();
}

// for meta-robot clustering, it counts how many times each robot collide with other members
bool getConflicts(
    // const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    const std::vector<dynobench::Trajectory>& multi_robot_trajectories,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd*>& robot_objs,
    std::vector<std::vector<int>>& conflict_matrix){
    bool collision = false;
    size_t max_t = 0;
    for (const auto& traj : multi_robot_trajectories){
      max_t = std::max(max_t, traj.states.size() - 1);
    }
    Eigen::VectorXd node_state;
    std::vector<Eigen::VectorXd> node_states;
    for (size_t t = 0; t <= max_t; ++t){
        node_states.clear();
        size_t robot_idx = 0;
        size_t obj_idx = 0;
        std::vector<fcl::Transform3d> ts_data;
        for (auto &robot : all_robots){
          if (t >= multi_robot_trajectories.at(robot_idx).states.size()){
              node_state = multi_robot_trajectories.at(robot_idx).states.back();    
          }
          else {
              node_state = multi_robot_trajectories.at(robot_idx).states[t];
          }
          node_states.push_back(node_state);
          std::vector<fcl::Transform3d> tmp_ts(1);
          if (robot->name == "car_with_trailers") {
            tmp_ts.resize(2);
          }
          robot->transformation_collision_geometries(node_state, tmp_ts);
          ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
          ++robot_idx;
        }
        for (size_t i = 0; i < ts_data.size(); i++) {
          fcl::Transform3d &transform = ts_data[i];
          robot_objs[obj_idx]->setTranslation(transform.translation());
          robot_objs[obj_idx]->setRotation(transform.rotation());
          robot_objs[obj_idx]->computeAABB();
          ++obj_idx;
        }
        col_mng_robots->update(robot_objs);
        fcl::DefaultCollisionData<double> collision_data;
        col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<double>);
        if (collision_data.result.isCollision()) {
            assert(collision_data.result.numContacts() > 0);
            collision = true;
            for(size_t k = 0; k < collision_data.result.numContacts(); k++){ // not 2 only ?
              const auto& contact = collision_data.result.getContact(k);
              auto idx_i = (size_t)contact.o1->getUserData();
              auto idx_j = (size_t)contact.o2->getUserData();
              assert(idx_i != idx_j);
              // get lower-triangle matrix
              if(idx_i >= idx_j)
                conflict_matrix[idx_i][idx_j] += 1;
              else
                conflict_matrix[idx_j][idx_i] += 1;
              // std::cout << "(Opt) CONFLICT at time " << t << " " << idx_i << " " << idx_j << std::endl;
            }
        } 
    }
    // get the max element
    // int max_collision = 0;
    // for (const auto& cft : conflict_matrix) {
    //     int localMax = *std::max_element(cft.begin(), cft.end());
    //     if (localMax > max_collision) {
    //         max_collision = localMax;
    //     }
    // }
    // return max_collision;
    return collision;
}

struct MaxCollidingRobots {
  size_t idx_i;
  size_t idx_j;
  int collisions;
};

// hard-coded, check j, k
void export_solutions_joint(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, std::ofstream *out){
  float cost = 0;
  size_t max_t = 0;
  size_t max_a = 0;
  int k = 6+1; // for the state, residual force + 1
  int j = 3; // for the actions
  Eigen::VectorXd tmp_state(k*2);
  Eigen::VectorXd tmp_action(j*2);
  std::vector<Eigen::VectorXd> joint_states;
  std::vector<Eigen::VectorXd> joint_actions;

  for (auto& n : solution){
    cost += n.trajectory.cost;
    max_t = std::max(max_t, n.trajectory.states.size() - 1);
    max_a = std::max(max_a, n.trajectory.actions.size() - 1);
  }

  *out << "cost: " << cost << std::endl; 
  *out << "result:" << std::endl;
  *out << "  - states:" << std::endl;
  for (size_t t = 0; t <= max_t; ++t){
    for (size_t i = 0; i < solution.size(); ++i){ // for each robot
      if (t >= solution[i].trajectory.states.size()){
          tmp_state.segment(i*k,k-1) = solution[i].trajectory.states.back();    // res
          // tmp_state.segment(i*k,k) = solution[i].trajectory.states.back();    
      }
      else {
          tmp_state.segment(i*k,k-1) = solution[i].trajectory.states[t]; // res
          // tmp_state.segment(i*k,k) = solution[i].trajectory.states[t];
      }
      tmp_state((i+1)*k-1) = 0; // for the residual force
    }
    joint_states.push_back(tmp_state);
  }
  // write to file 
  for (size_t j = 0; j < joint_states.size(); ++j){
      *out << "      - ";
      *out << joint_states.at(j).format(dynobench::FMT)<< std::endl;
  }
  *out << "    actions:" << std::endl;
  for (size_t t = 0; t <= max_a; ++t){
    for (size_t i = 0; i < solution.size(); ++i){ 
      if (t >= solution[i].trajectory.actions.size()){
          tmp_action.segment(i*j,j) = solution[i].trajectory.actions.back();    
      }
      else {
          tmp_action.segment(i*j,j) = solution[i].trajectory.actions[t];
      }
    }
    joint_actions.push_back(tmp_action); 
  }
  // write to file 
  for (size_t j = 0; j < joint_actions.size(); ++j){
      *out << "      - ";
      *out << joint_actions.at(j).format(dynobench::FMT)<< std::endl;
  }
}
// #include <boost/heap/d_ary_heap.hpp>

// // Define your element type (e.g., HighLevelNode)
// struct HighLevelNode {
//     // Define your element properties and methods
// };

// // Define your binary heap type
// typedef boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>> BinaryHeap;

// // Access the handle_type typedef
// typedef BinaryHeap::handle_type HandleType;

// Now you can use HandleType to declare variables that represent handles to elements in the binary heap
