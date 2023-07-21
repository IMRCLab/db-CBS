#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

#include "db_astar.hpp"
#include "planresult.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
// For each robot
struct Agent {
    std::vector<double> start_;
    std::vector<double> goal_;
    std::vector<fcl::CollisionObjectf *> obstacles_;
    std::shared_ptr<Robot> robot_;    
};

// Conflicts 
struct Conflict {
  float time;
  std::vector<ob::State*> conflict_states;
  size_t length; // corresponds to number of robots
};

// Constraints
struct Constraint {
//   Constraint(float time, ob::State* state) : time(time), constrained_state(state) {}
  float time;
  ob::State* constrained_state;
};

struct HighLevelNode {
    std::vector<LowLevelPlan<AStarNode*>> solution;
    std::vector<std::vector<Constraint>> constraints;
    // std::map<size_t, std::vector<Constraint>> constraints;

    float cost; 
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
  };


bool getEarliestConflict(const std::vector<LowLevelPlan<AStarNode*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots,
                    Conflict& early_conflict){
    int max_t = 0;
    std::vector<fcl::CollisionObjectf*> robot_objs_;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots_;
    col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_robots_->setup();
    ob::State *node_state;
    std::vector<ob::State*> node_states;
    for (const auto& sol : solution){
      max_t = std::max<int>(max_t, sol.plan.size() - 1);
    }
    
    for (int t = 0; t <= max_t; ++t){
        // std::cout << "TIMESTAMP: " << t << std::endl;
        node_states.clear();
        robot_objs_.clear();
        col_mng_robots_->clear();
        for (size_t i = 0; i < all_robots.size(); ++i){
            // std::cout << "ROBOT " << i << std::endl;
            if (t >= solution[i].plan.size()){
                node_state = solution[i].plan.back()->state;    
            }
            else {
                node_state = solution[i].plan[t]->state;
            }
            node_states.push_back(node_state);
            const auto transform = all_robots[i]->getTransform(node_state,0);
            auto robot = new fcl::CollisionObjectf(all_robots[i]->getCollisionGeometry(0)); 
            // std::cout << transform.translation() << std::endl;
            robot->setTranslation(transform.translation());
            robot->setRotation(transform.rotation());
            robot->computeAABB();
            robot_objs_.push_back(robot);
        }
        col_mng_robots_->registerObjects(robot_objs_);
        fcl::DefaultCollisionData<float> collision_data;
        col_mng_robots_->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
        if (collision_data.result.isCollision()) {
            early_conflict.time = t;
            early_conflict.conflict_states = node_states;
            early_conflict.length = node_states.size();
            std::cout << "CONFLICT at time " << t << std::endl;
            return true;
        } 
    }
    return false;
}

// Constraints from Conflicts
void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<Constraint>>& constraints){
        for (size_t i = 0; i < early_conflict.length; ++i){ // for each Robot in conflict
            for (size_t j = 0; j < early_conflict.length; ++j){
                if (j==i){
                    continue;
                }
                Constraint temp_const = {early_conflict.time, early_conflict.conflict_states[j]};
                constraints[i].push_back(temp_const);
            }
    }

}

int main() {
    
    std::string inputFile = "/home/akmarak-laptop/IMRC/db-CBS/example/parallelpark.yaml";
    std::string filename_motions = "/home/akmarak-laptop/IMRC/db-CBS/results/dbg/motions.msgpack";

    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    std::vector<fcl::CollisionObjectf *> obstacles;
    std::vector<std::vector<fcl::Vector3f>> positions;
    for (const auto &obs : env["environment"]["obstacles"])
    {
        if (obs["type"].as<std::string>() == "box"){
            const auto &size = obs["size"];
            std::shared_ptr<fcl::CollisionGeometryf> geom;
            geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
            const auto &center = obs["center"];
            auto co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
            co->computeAABB();
            obstacles.push_back(co);
        }
        else {
        throw std::runtime_error("Unknown obstacle type!");
        }
    }
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
        position_bounds.setLow(i, env_min[i].as<double>());
        position_bounds.setHigh(i, env_max[i].as<double>());
    }
    // std::vector<Agent> agents;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<std::string> robot_types;

    std::vector<double> start_reals;
    std::vector<double> goal_reals;
    HighLevelNode start;
    
    start.solution.resize(env["robots"].size());
    start.constraints.resize(env["robots"].size());
    start.cost = 0;
    start.id = 0;
    int i = 0;
    std::vector<std::shared_ptr<Robot>> robots;
    for (const auto &robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);
        for (const auto& v : robot_node["start"]) {
            start_reals.push_back(v.as<double>());
        }
        for (const auto& v : robot_node["goal"]) {
            goal_reals.push_back(v.as<double>());
        }
        starts.push_back(start_reals);
        goals.push_back(goal_reals);
        robot_types.push_back(robotType);

        DBAstar<Constraint> llplanner;
        // bool success = llplanner.search(filename_motions, start_reals, goal_reals, 
        //     obstacles, robot, robotType, env_min, start.constraints[i], start.solution[i]); 
        bool success = llplanner.search(filename_motions, starts[i], goals[i], 
            obstacles, robots[i], robot_types[i], env_min, start.constraints[i], start.solution[i]); 
        std::cout<< "Start node run done:" << success << std::endl;
        start.cost += start.solution[i].cost;
        std::cout << "High Level Node Cost: " << start.cost << std::endl;
        start_reals.clear();
        goal_reals.clear();
        i++;  
    } 

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> > open;
    auto handle = open.push(start);
    (*handle).handle = handle;
    int id = 1;
    while (!open.empty()) {
      HighLevelNode P = open.top();
      open.pop();
      Conflict inter_robot_conflict;
      if (!getEarliestConflict(P.solution, robots, inter_robot_conflict)) {
        std::cout << "done; cost: " << P.cost << std::endl;
      }
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);
      int it=0;
      for (const auto& c : constraints){
        HighLevelNode newNode = P;
        size_t i = c.first;
        newNode.id = id;
        newNode.constraints[i].insert(newNode.constraints[i].begin(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[i].cost;
        // run the low level planner
        DBAstar<Constraint> llplanner;
        bool success = llplanner.search(filename_motions, starts[it], goals[it], 
            obstacles, robots[it], robot_types[it], env_min, newNode.constraints[i], newNode.solution[i]); 
        newNode.cost += newNode.solution[i].cost;
        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          std::cout<< "Run with constraints is done:" << success << std::endl;
        }
        it++;
        id++;
      }
      // test constraints on R1 = (time, R2(time))
    //   for (size_t i = 0; i < constraints[0].size(); ++i){
    //         std::cout << constraints[0][i].time << std::endl;
    //         const auto node_state = constraints[0][i].constrained_state;
    //         const auto transform = robots[0]->getTransform(node_state,0);
    //         std::cout << transform.translation() << std::endl;
    //   }
    }

    return 0;
}














// To do:
//     * Better data structure for db-astar.search 
//     * 


// bool getEarliestConflict(const std::vector<LowLevelPlan<AStarNode*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots,
//                     std::vector<Conflict>& all_conflicts){
//     int max_t = 0;
//     std::vector<fcl::CollisionObjectf*> robot_objs_;
//     std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots_;
//     col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
//     col_mng_robots_->setup();
//     ob::State *node_state;
//     std::vector<ob::State*> node_states;
//     for (const auto& sol : solution){
//       max_t = std::max<int>(max_t, sol.plan.size() - 1);
//     }
    
//     for (int t = 0; t <= max_t; ++t){
//         std::cout << "TIMESTAMP: " << t << std::endl;
//         node_states.clear();
//         robot_objs_.clear();
//         col_mng_robots_->clear();
//         for (size_t i = 0; i < all_robots.size(); ++i){
//             std::cout << "ROBOT " << i << std::endl;
//             if (t >= solution[i].plan.size()){
//                 node_state = solution[i].plan.back()->state;    
//             }
//             else {
//                 node_state = solution[i].plan[t]->state;
//             }
//             node_states.push_back(node_state);
//             const auto transform = all_robots[i]->getTransform(node_state,0);
//             auto robot = new fcl::CollisionObjectf(all_robots[i]->getCollisionGeometry(0)); 
//             std::cout << transform.translation() << std::endl;
//             robot->setTranslation(transform.translation());
//             robot->setRotation(transform.rotation());
//             robot->computeAABB();
//             robot_objs_.push_back(robot);
//         }
//         col_mng_robots_->registerObjects(robot_objs_);
//         // col_mng_robots_->update();
//         fcl::DefaultCollisionData<float> collision_data;
//         col_mng_robots_->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
//         if (collision_data.result.isCollision()) {
//             Conflict inter_robot_conflict(t,node_states);
//             std::cout << "CONFLICT at time " << t << std::endl;
//             all_conflicts.push_back(inter_robot_conflict);
//         } 
//     }
//     if (all_conflicts.size() != 0){
//         return true;
//     }
//     return false;
// }


// void createConstraintsFromConflicts(const std::vector<Conflict>& conflicts, std::map<size_t, std::vector<Constraint>>& constraints){
//     for (size_t i = 0; i < conflicts.size(); ++i){
//         for (size_t j = 0; j < conflicts[i].length; ++j){ // for each Rs in conflict
//             std::vector<ob::State*> conflict_states;
//             for (size_t k = 0; k < conflicts[i].length; ++k){
//                 if (k==j){
//                     continue;
//                 }
//                 conflict_states.push_back(conflicts[i].states[k]);
//             }
            
//             Constraint temp_const(conflicts[i].time, conflict_states);
//             constraints[j].push_back(temp_const);
//         }
//     }

// }