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

#include <idbastar/optimization/ocp.hpp>

// #include "dynobench/motions.hpp"
// #include <Eigen/Dense>

// #define dynobench_base "../../dynobench/"

// using namespace dynoplan;
// using namespace dynobench;

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
    std::vector<LowLevelPlan<AStarNode*, ob::State*, oc::Control*>> solution;
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

void print_solution(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots){
    int max_t = 0;
    ob::State *node_state;
    for (const auto& sol : solution){
      max_t = std::max<int>(max_t, sol.trajectory.size() - 1);
    }
    for (int t = 0; t <= max_t; ++t){
        std::cout << "/////////// "<< "time: " << t << "///////////" << std::endl;
        for (size_t i = 0; i < all_robots.size(); ++i){
            std::cout << "robot " << i << std::endl;
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
            }
            const auto transform = all_robots[i]->getTransform(node_state,0);
            auto robot = new fcl::CollisionObjectf(all_robots[i]->getCollisionGeometry(0)); 
            std::cout << transform.translation() << std::endl;
        }
    }
}
bool getEarliestConflict(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots,
                    Conflict& early_conflict){
    int max_t = 0;
    std::vector<fcl::CollisionObjectf*> robot_objs_;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots_;
    col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_robots_->setup();
    ob::State *node_state;
    std::vector<ob::State*> node_states;
    for (const auto& sol : solution){
      max_t = std::max<int>(max_t, sol.trajectory.size() - 1);
    }
    
    for (int t = 0; t <= max_t; ++t){
        // std::cout << "TIMESTAMP: " << t << std::endl;
        node_states.clear();
        robot_objs_.clear();
        col_mng_robots_->clear();
        for (size_t i = 0; i < all_robots.size(); ++i){
            // std::cout << "ROBOT " << i << std::endl;
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
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
                Constraint temp_const = {early_conflict.time*0.1, early_conflict.conflict_states[j]};
                constraints[i].push_back(temp_const);
            }
    }

}


// export path to .yaml file
void export_solutions(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, 
                        const std::vector<std::shared_ptr<Robot>>& robots, std::string outputFile){
    // std::string outputFile = "db_solution_test.yaml";
    std::ofstream out(outputFile);
    std::vector<double> reals;
    float cost = 0;
    for (auto& n : solution)
      cost += n.cost;
    out << "delta: " << 0.5 << std::endl;
    out << "epsilon: " << 1 << std::endl;
    out << "cost: " << cost << std::endl; 
    out << "result:" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
        auto si = robots[i]->getSpaceInformation(); 
        out << "  - states:" << std::endl;
        for (size_t j = 0; j < solution[i].trajectory.size(); ++j){
            const auto node_state = solution[i].trajectory[j];
            out << "      - ";
            printState(out, si, node_state);
            out << std::endl;
        }
    }
    auto si = robots.back()->getSpaceInformation();
    out << "      - ";
    // printState(out, si, solution.back().trajectory.back());
    printState(out, si, solution.back().plan.back()->state);
    out << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
        auto si = robots[i]->getSpaceInformation(); 
        out << "    actions:" << std::endl;
        for (size_t j = 0; j < solution[i].actions.size(); ++j){
            const auto& node_action = solution[i].actions[j];
            // out << "      # ";
            out << "      - ";
            printAction(out, si, node_action);
            out << std::endl;
        }

    }
}


#define dynobench_base "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/"

void execute_optimization(std::string file)
{

    using namespace dynoplan;
    using namespace dynobench;

    Options_trajopt options_trajopt;
    Problem problem("/home/akmarak-laptop/IMRC/db-CBS/example/parallelpark.yaml");
    // (dynobench_base +
                //   std::string("envs/db_test/parallelpark.yaml"));
                  // std::string("envs/unicycle2_v0/parallelpark_0.yaml"));

    Trajectory init_guess("/home/akmarak-laptop/IMRC/db-CBS/buildDebug/" + file);

        // std::string("data/unicycle2_0_parallelark_guess_0.yaml"));

    options_trajopt.solver_id = static_cast<int>(SOLVER::traj_opt);
    options_trajopt.control_bounds = 1;
    options_trajopt.use_warmstart = 1;
    options_trajopt.weight_goal = 100;
    options_trajopt.max_iter = 20; //50;
    problem.models_base_path = dynobench_base + std::string("models/");

    Result_opti result;
    Trajectory sol;
    trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
    std::string outputFile = "test_dbcbs_opt.yaml";
    std::ofstream out(outputFile);
    result.write_yaml(out);
    // BOOST_TEST_CHECK(result.feasible);
    std::cout << "cost is " << result.cost << std::endl;
    // BOOST_TEST_CHECK(result.cost <= 10.);
}

int main() {
    
    std::string inputFile = "/home/akmarak-laptop/IMRC/db-CBS/example/parallelpark.yaml";
    std::string filename_motions = "/home/akmarak-laptop/IMRC/db-CBS/results/dbg/motions.msgpack";
    std::string outputFile = "db_solution_test.yaml";
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
        bool success = llplanner.search(filename_motions, starts[i], goals[i], 
            obstacles, robots[i], robot_types[i], env_min, start.constraints[i], start.solution[i]); 
        start.cost += start.solution[i].cost;
        std::cout << "High Level Node Cost: " << start.cost << std::endl;
        start_reals.clear();
        goal_reals.clear();
        i++;  
    } 
    // For Debugging
    // print_solution(start.solution, robots);
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
        std::cout << "Final solution! cost: " << P.cost << std::endl;
        export_solutions(P.solution, robots, outputFile);
        execute_optimization(outputFile);
        return 0;
      }
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);
      for (const auto& c : constraints){
        HighLevelNode newNode = P;
        size_t i = c.first;
        newNode.id = id;
        std::cout << "Node ID is " << id << std::endl;
        newNode.constraints[i].insert(newNode.constraints[i].begin(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[i].cost;
        std::cout << "New node cost: " << newNode.cost << std::endl;

        // run the low level planner
        DBAstar<Constraint> llplanner;
        bool success = llplanner.search(filename_motions, starts[i], goals[i], 
            obstacles, robots[i], robot_types[i], env_min, newNode.constraints[i], newNode.solution[i]); 
        newNode.cost += newNode.solution[i].cost;
        std::cout << "Updated New node cost: " << newNode.cost << std::endl;

        if (success) {
            // print_solution(newNode.solution, robots);

          auto handle = open.push(newNode);
          (*handle).handle = handle;
          
        }
        else {

        }
        id++;
      }

    }

    return 0;
}













