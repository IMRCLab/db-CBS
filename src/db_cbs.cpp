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

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "planresult.hpp"
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"


namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace dynoplan;

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
  float time;
  ob::State* constrained_state;
};

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
    // read motions, run tbastar
    // std::vector<std::vector<Motion>> motions;
    std::vector<Motion> motions;
    int robot_id = 0;
    for (const auto &robot : robots){
        load_motion_primitives_new(
            options_tdbastar.motionsFile, *robot, motions, options_tdbastar.max_motions,
            options_tdbastar.cut_actions, false, options_tdbastar.check_cols);
        options_tdbastar.motions_ptr = &motions;
        dynobench::Trajectory traj_out;
        tdbastar(problem, options_tdbastar, traj_out, out_tdb, robot_id);
        traj_out.to_yaml_format(outputFile);
        std::cout << "*** input_tdb *** " << std::endl;
        out_tdb.write_yaml(std::cout);
        std::cout << "***" << std::endl;
        robot_id++;
    }
    // allocate data for conflict checking
    std::vector<fcl::CollisionObjectf*> col_mng_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_robots->setup();

    // for (size_t i = 0; i < robots.size(); ++i) {
    //     for (size_t p = 0; p < robots[i]->numParts(); ++p) {
    //         auto coll_obj = new fcl::CollisionObjectf(robots[i]->getCollisionGeometry(p));
    //         size_t userData = i;
    //         robots[i]->getCollisionGeometry(p)->setUserData((void*)userData);
    //         col_mng_objs.push_back(coll_obj);
    //     }
    // }
    // col_mng_robots->registerObjects(col_mng_objs);
   
    return 0;
}
