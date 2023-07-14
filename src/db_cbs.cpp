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

struct Agent {
    std::vector<double> start_;
    std::vector<double> goal_;
    std::vector<fcl::CollisionObjectf *> obstacles_;
    std::shared_ptr<Robot> robot_;    
};


struct HighLevelNode {
    std::vector<LowLevelPlan<AStarNode*>> solution;
    // std::vector<Constraints> constraints;

    float cost; // current->gscore

    // int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
  };

bool getFirstConflict(const std::vector<LowLevelPlan<AStarNode*>>& solution, const std::shared_ptr<Robot>& all_robots){
    int max_t = 0;
    std::vector<fcl::CollisionObjectf*> robot_objs_;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots_;

    for (const auto& sol : solution){
      max_t = std::max<int>(max_t, sol.plan.size() - 1);
    }
    auto si = all_robots->getSpaceInformation();
    
    for (int t = 0; t <= max_t; ++t){
        for (size_t i = 0; i < solution.size(); ++i){
            auto node_state = solution[i].plan[t]->state;
            // if (si->isValid(node_state)) {
            //     std::cout << "Valid state" << std::endl;
            //     return false;
            // }
            const auto transform = all_robots->getTransform(node_state, i);
            auto robot = new fcl::CollisionObjectf(all_robots->getCollisionGeometry(i)); 
            
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
            return true;
        } 
    }
    return false;
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
    // start.constraints.resize(initialStates.size());
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
    std::shared_ptr<Robot> joint_robot = create_joint_robot(robots);

    // typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    //                                  boost::heap::mutable_<true> > open;

    // auto handle = open.push(start);
    // (*handle).handle = handle;
    bool conf = getFirstConflict(start.solution, joint_robot);
    // if (conf){
    //     std::cout<<"Conflict"<<std::endl;
    // }
    // while (!open.empty()) {
    //   HighLevelNode P = open.top();

    // }

    return 0;
}

// To do:
//     * db-A* running in an iterative manner
//     * Better data structure for db-astar.search 
//     * 
