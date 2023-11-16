#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "planresult.hpp"


namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace dynoplan;

// Conflicts 
struct Conflict {
  double time;
  size_t robot_idx_i;
  Eigen::VectorXd robot_state_i;
  size_t robot_idx_j;
  Eigen::VectorXd robot_state_j;
};

// Constraints
struct Constraint {
  double time;
  Eigen::VectorXd constrained_state;
};
struct HighLevelNode {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    // std::vector<std::vector<Constraint>> constraints;
    float cost; 
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;
    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
};
bool getEarliestConflict(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd*>& robot_objs,
    Conflict& early_conflict)
{
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

#define DYNOBENCH_BASE "../dynoplan/dynobench/"

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)");

    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.outFile = outputFile;
    options_tdbastar.max_motions = 50;
    options_tdbastar.motionsFile = "../dynoplan/data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin";
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
    // tdbastar problem
    dynobench::Problem problem(inputFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
    Out_info_tdb out_tdb;
    std::cout << "*** options_tdbastar ***" << std::endl;
    options_tdbastar.print(std::cout);
    std::cout << "***" << std::endl;

    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    std::vector<fcl::CollisionObjectf *> obstacles;
    std::vector<std::vector<fcl::Vector3f>> positions;
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> collision_geometries;
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

    fcl::AABBf workspace_aabb(
        fcl::Vector3f(env_min[0].as<double>(),
        env_min[1].as<double>(),-1),
        fcl::Vector3f(env_max[0].as<double>(), env_max[1].as<double>(), 1));

    std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
    std::vector<dynobench::Trajectory> ll_trajs;
    for (const auto &robotType : problem.robotTypes){
        std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
                (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
        robots.push_back(robot);
    }
    // Initialize the root node
    HighLevelNode start;
    start.solution.resize(env["robots"].size());
    start.cost = 0;
    start.id = 0;
    // read motions, run tbastar
    create_dir_if_necessary(outputFile);
    std::ofstream out(outputFile);
    out << "result:" << std::endl;
    std::vector<Motion> motions;
    int robot_id = 0;
    for (const auto &robot : robots){
        collision_geometries.insert(collision_geometries.end(),
                                robot->collision_geometries.begin(),
                                robot->collision_geometries.end());
        load_motion_primitives_new(
            options_tdbastar.motionsFile, *robot, motions, options_tdbastar.max_motions,
            options_tdbastar.cut_actions, false, options_tdbastar.check_cols);
        options_tdbastar.motions_ptr = &motions;
        tdbastar(problem, options_tdbastar, start.solution[robot_id].trajectory, out_tdb, robot_id, out);
        robot_id++;
    }
    // allocate data for conflict checking, check for conflicts
    std::vector<fcl::CollisionObjectd*> robot_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    col_mng_robots->setup();
    for (size_t i = 0; i < collision_geometries.size(); i++){
          size_t userData = i;
          auto robot_obj = new fcl::CollisionObject(collision_geometries[i]);
          collision_geometries[i]->setUserData((void*)userData);
          robot_objs.push_back(robot_obj);
    }
    col_mng_robots->registerObjects(robot_objs);
    Conflict inter_robot_conflict;
    getEarliestConflict(start.solution, robots, col_mng_robots, robot_objs, inter_robot_conflict);

   
   
    return 0;
}
