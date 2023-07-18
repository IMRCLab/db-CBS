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
  Conflict(float time, const std::vector<ob::State*> states) : time(time), states(states), length(states.size()) {}
  float time;
  const std::vector<ob::State*> states;
  size_t length; // corresponds to number of robots
};

// Constraints
struct Constraint {
  Constraint(float time, const std::vector<ob::State*> states) : time(time), states(states) {}
  float time;
  const std::vector<ob::State*> states;
};

struct HighLevelNode {
    std::vector<LowLevelPlan<AStarNode*>> solution;
    // std::vector<Constraint> constraints;
    float cost; 
    // int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
  };

// Constraint

bool getConflicts(const std::vector<LowLevelPlan<AStarNode*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots,
                    std::vector<Conflict>& all_conflicts){
    int max_t = 0;
    std::vector<fcl::CollisionObjectf*> robot_objs_;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots_;
    col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    ob::State *node_state;
    std::vector<ob::State*> node_states;
    for (const auto& sol : solution){
      max_t = std::max<int>(max_t, sol.plan.size() - 1);
    }
    
    for (int t = 0; t <= max_t; ++t){
        node_states.clear();
        for (size_t i = 0; i < all_robots.size(); ++i){
            if (t >= solution[i].plan.size()){
                node_state = solution[i].plan.back()->state;    
            }
            else {
                node_state = solution[i].plan[t]->state;
            }
            node_states.push_back(node_state);
            const auto transform = all_robots[i]->getTransform(node_state,0);
            auto robot = new fcl::CollisionObjectf(all_robots[i]->getCollisionGeometry(0)); 
            
            robot->setTranslation(transform.translation());
            robot->setRotation(transform.rotation());
            robot->computeAABB();
            robot_objs_.push_back(robot);
        }
        col_mng_robots_->registerObjects(robot_objs_);
        col_mng_robots_->setup();
        fcl::DefaultCollisionData<float> collision_data;
        col_mng_robots_->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
        if (collision_data.result.isCollision()) {
            Conflict inter_robot_conflict(t,node_states);
            all_conflicts.push_back(inter_robot_conflict);
        } 
    }
    if (all_conflicts.size() != 0){
        return true;
    }
    return false;
}
// Constraints from Conflicts
void createConstraintsFromConflicts(const std::vector<Conflict>& conflicts, std::map<size_t, std::vector<Constraint>>& constraints){
    for (size_t i = 0; i < conflicts.size(); ++i){
        for (size_t j = 0; j < conflicts[i].length; ++j){ // for each Rs in conflict
            std::vector<ob::State*> conflict_states;
            for (size_t k = 0; k < conflicts[i].length; ++k){
                if (k==j){
                    continue;
                }
                conflict_states.push_back(conflicts[i].states[k]);
            }
            
            Constraint temp_const(conflicts[i].time, conflict_states);
            constraints[j].push_back(temp_const);
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
    std::vector<double> start_reals;
    std::vector<double> goal_reals;
    HighLevelNode start;
    
    start.solution.resize(env["robots"].size());
    // start.constraints.resize(env["robots"].size());
    start.cost = 0;
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
        DBAstar llplanner;
        bool success = llplanner.search(filename_motions, start_reals, goal_reals, 
            obstacles, robot, robotType, env_min, start.solution[i]); // better data structure needed
        // bool success = llplanner.search(filename_motions, robot_node, robot,
            // obstacles, env_min, env_max, start.solution[i]);
        std::cout<< "Run done:" << success << std::endl;
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
    while (!open.empty()) {
      HighLevelNode P = open.top();
      open.pop();
      std::vector<Conflict> conflicts;
      if (!getConflicts(P.solution, robots, conflicts)) {
        std::cout << "done; cost: " << P.cost << std::endl;
      }
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(conflicts, constraints);
      for (size_t i = 0; i < constraints[0].size(); ++i){
            std::cout << constraints[0][i].time << std::endl;
            const auto node_state = constraints[0][i].states[0];
            const auto transform = robots[0]->getTransform(node_state,0);
            std::cout << transform.translation() << std::endl;
      }
   
    } // while

    return 0;
}

// To do:
//     * Better data structure for db-astar.search 
//     * 
