#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <filesystem>
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/tdbastar_epsilon.hpp"

// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "dbcbs_utils.hpp"

namespace fs = std::filesystem;
using namespace dynoplan;

#define DYNOBENCH_BASE "../dynoplan/dynobench/"
// no external constraints assumed
// extracting expanded nodes
// no high-level search is saved for visualization
// using merged_aabb and status-based collision checking
// assumes the problem instance has a single robot, using robotType
int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string cfgFile;
    double timeLimit;

    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")
      ("time_limit,t", po::value<double>(&timeLimit)->required(), "time limit for search");

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
    // read files
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    cfg = cfg["db-ecbs"]["default"];
    float alpha = cfg["alpha"].as<float>();
    bool filter_duplicates = cfg["filter_duplicates"].as<bool>();
    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.outFile = outputFile;
    options_tdbastar.search_timelimit = timeLimit;
    options_tdbastar.cost_delta_factor = 0;
    options_tdbastar.fix_seed = 1;
    options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
    options_tdbastar.rewire = true;
    options_tdbastar.w = 1.3;
    // other custom parameters
    std::string focal_heuristic = "volume_wise"; 
    bool save_expanded_trajs = true;
    bool merged = true; // for aabb merged
    fs::path output_path(outputFile);
    std::string output_folder = output_path.parent_path().string();
    std::vector<dynobench::Trajectory> expanded_trajs;
    
    // create the problem
    dynobench::Problem problem(inputFile);
    dynobench::Problem problem_original(inputFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
    // tdb_out info
    Out_info_tdb out_tdb;
    std::cout << "*** options_tdbastar_epsilon ***" << std::endl;
    options_tdbastar.print(std::cout);
    std::cout << "***" << std::endl;
    // trajectory for the final solution
    dynobench::Trajectory final_trajectory;
    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    // read constraints
    std::vector<Constraint> constraints;
    // read motions
    const auto robot_type = problem.robotType;
    std::string motionsFile;
    std::vector<Motion> motions;
    if (robot_type == "unicycle1_v0" || robot_type == "unicycle1_sphere_v0"){
        motionsFile = "../new_format_motions/unicycle1_v0/unicycle1_v0.msgpack";
    } else if (robot_type == "unicycle2_v0"){
        motionsFile = "../new_format_motions/unicycle2_v0/unicycle2_v0.msgpack";
    } else if (robot_type == "car1_v0"){
        motionsFile = "../new_format_motions/car1_v0/car1_v0.msgpack";
    } else if (robot_type == "integrator2_2d_v0"){
        motionsFile = "../new_format_motions/integrator2_2d_v0/integrator2_2d_v0.msgpack";
    } else if (robot_type == "integrator2_3d_v0"){
        motionsFile = "../new_format_motions/integrator2_3d_v0/integrator2_3d_v0_long.bin.im.bin.sp.bin";
    } else { 
        throw std::runtime_error("Unknown motion filename for this robottype!");
    }
    options_tdbastar.motionsFile = motionsFile;
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
                (problem.models_base_path + robot_type + ".yaml").c_str(), problem.p_lb, problem.p_ub);
    // load motion primitives
    std::map<std::string, std::vector<Motion>> robot_motions;
    load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions[problem.robotType], 
                              options_tdbastar.max_motions,
                              options_tdbastar.cut_actions, false, options_tdbastar.check_cols, merged);
    options_tdbastar.motions_ptr = &robot_motions[problem.robotType]; 
    // create tmp variables
    size_t fake_robot_id = 0;
    std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
    robots.push_back(robot);
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution; // single robot
    std::vector<std::vector<std::pair<std::shared_ptr<AStarNode>, size_t>>> result(1);
    std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>>*> heuristics(robots.size(), nullptr);
    // handle collision objects
    std::vector<fcl::CollisionObjectd*> robot_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robot;
    col_mng_robot = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    col_mng_robot->setup();
    for (auto col : robot->collision_geometries){
        auto robot_obj = new fcl::CollisionObject(col);
        robot_objs.push_back(robot_obj);
    }
    col_mng_robot->registerObjects(robot_objs);
    // do the reverse search first
    if (cfg["heuristic1"].as<std::string>() == "reverse-search"){
        options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
        dynobench::Trajectory tmp_trajectory;
        problem.start.head(robot->translation_invariance).setConstant(std::sqrt(std::numeric_limits<double>::max()));
        Eigen::VectorXd tmp_state = problem.start;
        problem.start = problem.goal;
        problem.goal = tmp_state;

        tdbastar_epsilon(problem, options_tdbastar, 
                tmp_trajectory, /*constraints*/{},
                out_tdb, fake_robot_id, /*reverse_search*/true, 
                expanded_trajs, solution, result, robot_motions,
                robots, col_mng_robot, robot_objs,
                nullptr, &heuristics[0], options_tdbastar.w, focal_heuristic);

        std::cout << "computed heuristic with " << heuristics[0]->size() << " entries." << std::endl;

    }
    // set back everything for forward search
    problem.start = problem_original.start;
    problem.goal = problem_original.goal;
    options_tdbastar.delta = cfg["delta_0"].as<float>();
    expanded_trajs.clear();
    // disable/enable motins
    disable_motions(robot, problem.robotType, options_tdbastar.delta, filter_duplicates, alpha, 
            options_tdbastar.max_motions, robot_motions[problem.robotType]);

    tdbastar_epsilon(problem, options_tdbastar, 
        final_trajectory, /*constraints*/{},
        out_tdb, fake_robot_id, /*reverse_search*/false, 
        expanded_trajs, solution, result, robot_motions,
        robots, col_mng_robot, robot_objs,
        heuristics[0], nullptr, options_tdbastar.w, focal_heuristic);
    if(!out_tdb.solved){
        std::cout << "Couldn't find initial solution for robot." << std::endl;
        return 0;
    }
    else{
        if (save_expanded_trajs){
            std::ofstream out2(output_folder + "/expanded_trajs.yaml");
            out2 << "trajs:" << std::endl;
            for (auto traj : expanded_trajs){
            out2 << "  - " << std::endl;
            traj.to_yaml_format(out2, "    ");
            }
        }
    }
    return 0;
}