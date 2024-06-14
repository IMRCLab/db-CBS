#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
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

void export_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, 
                      std::ofstream *out){
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
        // *out << "  - states:" << std::endl;
        *out << indent << "states:" << std::endl;
        for (size_t j = 0; j < tmp_states.size(); ++j){
            // *out << "      - ";
            *out << indent << "  - " << tmp_states.at(j).format(dynobench::FMT)<< std::endl;
        }
        // *out << "    actions:" << std::endl;
        *out << indent << "actions:" << std::endl;
        for (size_t j = 0; j < tmp_actions.size(); ++j){
            // *out << "      - ";
            *out << indent << "  - " << tmp_actions.at(j).format(dynobench::FMT)<< std::endl;
            
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
