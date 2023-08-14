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
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>


namespace ob = ompl::base;
namespace oc = ompl::control;

// Conflicts 
struct Conflict {
  float time;
  size_t robot_idx_i;
  ob::State* robot_state_i;
  size_t robot_idx_j;
  ob::State* robot_state_j;
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
    size_t max_t = 0;
    ob::State *node_state;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.size() - 1);
    }
    for (size_t t = 0; t <= max_t; ++t){
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
            // auto robot = new fcl::CollisionObjectf(all_robots[i]->getCollisionGeometry(0));
            std::cout << transform.translation() << std::endl;
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

bool getEarliestConflict(
    const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution,
    const std::vector<std::shared_ptr<Robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots,
    Conflict& early_conflict)
{
    size_t max_t = 0;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.size() - 1);
    }

    std::vector<fcl::CollisionObjectf*> col_mng_objs;
    col_mng_robots->getObjects(col_mng_objs);

    ob::State* node_state;
    std::vector<ob::State*> node_states;
    
    for (size_t t = 0; t <= max_t; ++t){
        // std::cout << "TIMESTAMP: " << t << std::endl;
        node_states.clear();
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
            col_mng_objs[i]->setTranslation(transform.translation());
            col_mng_objs[i]->setRotation(transform.rotation());
            col_mng_objs[i]->computeAABB();
        }
        col_mng_robots->update(col_mng_objs);
        fcl::DefaultCollisionData<float> collision_data;
        col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
        if (collision_data.result.isCollision()) {
            //debug
            // fcl::DefaultDistanceData<float> inter_robot_distance_data;
            // inter_robot_distance_data.request.enable_signed_distance = true;
            // col_mng_robots_->distance(&inter_robot_distance_data, fcl::DefaultDistanceFunction<float>);

            assert(collision_data.result.numContacts() > 0);
            const auto& contact = collision_data.result.getContact(0);

            early_conflict.time = t * all_robots[0]->dt();
            early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
            early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
            early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
            early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];

            std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;
            auto si_i = all_robots[early_conflict.robot_idx_i]->getSpaceInformation();
            si_i->printState(early_conflict.robot_state_i);
            auto si_j = all_robots[early_conflict.robot_idx_j]->getSpaceInformation();
            si_j->printState(early_conflict.robot_state_j);
            return true;
        } 
    }
    return false;
}

// Constraints from Conflicts
void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<Constraint>>& constraints){
    constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
    constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
}

void export_joint_solutions(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, 
                        const std::vector<std::shared_ptr<Robot>>& robots, std::string outputFile){
    std::ofstream out(outputFile);
    std::vector<double> reals;
    ob::State *node_state;
    oc::Control *node_action;
    float cost = 0;
    size_t max_t = 0;
    size_t max_a = 0;
    for (auto& sol : solution){
      cost += sol.cost;
      max_t = std::max(max_t, sol.trajectory.size() - 1);
      max_a = std::max(max_a, sol.actions.size() - 1);
    }

    out << "delta: " << 0.5 << std::endl;
    out << "epsilon: " << 1 << std::endl;
    out << "cost: " << cost << std::endl; 
    out << "result:" << std::endl;
    out << "  - states:" << std::endl;
    std::vector<double> joint_state;
    std::vector<double> joint_action;
    std::vector<double> last_state;
    for (size_t t = 0; t <= max_t; ++t){
        out << "      - [";
        for (size_t i = 0; i < robots.size(); ++i){
            std::vector<double> reals;
            auto si = robots[i]->getSpaceInformation(); 
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
            }
            si->getStateSpace()->copyToReals(reals, node_state);
            joint_state.insert(joint_state.end(), reals.begin(), reals.end());
        }
        for (size_t k = 0; k < joint_state.size(); ++k) {
                out << joint_state[k];
                if (k < joint_state.size() - 1) {
                    out << ",";
                }
        }
        out << "]" << std::endl;
        joint_state.clear();

    }

    // for the action
    out << "    actions:" << std::endl;
    for (size_t t = 0; t <= max_a; ++t){
        out << "      - ";
        out << "[";
        for (size_t i = 0; i < robots.size(); ++i){
            std::vector<double> reals;
            auto si = robots[i]->getSpaceInformation(); 
            const size_t dim = si->getControlSpace()->getDimension();
            if (t >= solution[i].actions.size()){
                node_action = solution[i].actions.back();    
            }
            else {
                node_action = solution[i].actions[t];
            }
            for (size_t d = 0; d < dim; ++d)
            {
                double *address = si->getControlSpace()->getValueAddressAtIndex(node_action, d);
                reals.push_back(*address);
            }
            joint_action.insert(joint_action.end(), reals.begin(), reals.end());
        }
        for (size_t k = 0; k < joint_action.size(); ++k) {
                out << joint_action[k];
                if (k < joint_action.size() - 1) {
                    out << ",";
                }
        }
        out << "]" << std::endl;
        joint_action.clear();
    }

    
}

#define dynobench_base "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/"

void execute_optimization(std::string env_file, std::string initial_guess_file, std::string output_file)
{

    using namespace dynoplan;
    using namespace dynobench;

    Options_trajopt options_trajopt;
    Problem problem(env_file);
    Trajectory init_guess(initial_guess_file);


    options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
    options_trajopt.control_bounds = 1;
    options_trajopt.use_warmstart = 1;
    options_trajopt.weight_goal = 100;
    options_trajopt.max_iter = 50;
    problem.models_base_path = dynobench_base + std::string("models/");

    Result_opti result;
    Trajectory sol;
    trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
    std::ofstream out(output_file);
    std::cout << "cost is " << result.cost << std::endl;
    result.write_yaml_joint(out);
    // result.write_yaml_joint(out);
    // BOOST_TEST_CHECK(result.feasible);
    // BOOST_TEST_CHECK(result.cost <= 10.);
}

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string motionsFile;
    std::string outputFile;
    std::string jointFile;
    std::string optimizationFile;
    // std::string outputFileSimple;
    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("motions,m", po::value<std::string>(&motionsFile)->required(), "motions file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("joint,jnt", po::value<std::string>(&jointFile)->required(), "joint output file (yaml)")
      ("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)");
      
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
    // load the msgpck
    std::ifstream is( motionsFile.c_str(), std::ios::in | std::ios::binary );
    // get length of file
    is.seekg (0, is.end);
    int length = is.tellg();
    is.seekg (0, is.beg);
    //
    msgpack::unpacker unpacker;
    unpacker.reserve_buffer(length);
    is.read(unpacker.buffer(), length);
    unpacker.buffer_consumed(length);
    msgpack::object_handle oh;
    unpacker.next(oh);
    msgpack::object msg_objs = oh.get(); 

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
        /*bool success =*/ llplanner.search(msg_objs, starts[i], goals[i], 
            obstacles, robots[i], robot_types[i], env_min.size(), start.constraints[i], start.solution[i]);
        // TODO: need to handle the failure case, too!

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

    // allocate data for conflict checking
    std::vector<fcl::CollisionObjectf*> col_mng_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_robots->setup();

    for (size_t i = 0; i < robots.size(); ++i) {
        auto coll_obj = new fcl::CollisionObjectf(robots[i]->getCollisionGeometry(0));
        robots[i]->getCollisionGeometry(0)->setUserData((void*)i);
        col_mng_objs.push_back(coll_obj);
    }
    col_mng_robots->registerObjects(col_mng_objs);

    while (!open.empty()) {
      HighLevelNode P = open.top();
      open.pop();
      Conflict inter_robot_conflict;
      if (!getEarliestConflict(P.solution, robots, col_mng_robots, inter_robot_conflict)) {
        std::cout << "Final solution! cost: " << P.cost << std::endl;
        export_solutions(P.solution, robots, outputFile);
        export_joint_solutions(P.solution, robots, jointFile);
        execute_optimization(inputFile, jointFile, optimizationFile);
        return 0;
        break;
      }
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);
      for (const auto& c : constraints){
        HighLevelNode newNode = P;
        size_t i = c.first;
        newNode.id = id;
        std::cout << "Node ID is " << id << std::endl;
        newNode.constraints[i].insert(newNode.constraints[i].end(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[i].cost;
        std::cout << "New node cost: " << newNode.cost << std::endl;

        // run the low level planner
        DBAstar<Constraint> llplanner;
        bool success = llplanner.search(msg_objs, starts[i], goals[i], 
            obstacles, robots[i], robot_types[i], env_min.size(), newNode.constraints[i], newNode.solution[i]); 
        newNode.cost += newNode.solution[i].cost;
        std::cout << "Updated New node cost: " << newNode.cost << std::endl;

        if (success) {
        //   print_solution(newNode.solution, robots);

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











