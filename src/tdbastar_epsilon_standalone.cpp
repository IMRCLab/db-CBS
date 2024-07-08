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
#include "dynoplan/tdbastar/planresult.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "dbcbs_utils.hpp"

#define DYNOBENCH_BASE "../dynoplan/dynobench/"
using namespace dynoplan;
namespace fs = std::filesystem;

// How to run: 
// from build folder, ./discrete_search_test --env {problem instance, yaml}
// --input {solution of the other robot, yaml} --output {path to folder} --cfg {configuration file, yaml} --t 1000000
void read_input_yaml(const std::string file, dynobench::Trajectory &traj, size_t robot_idx){
    YAML::Node node = YAML::LoadFile(file);
    CHECK(node["result"], "");
    traj.read_from_yaml(node["result"][robot_idx]);
}

int main(int argc, char* argv[]){
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string envFile; // problem instance
    std::string inputFile; // file with solution
    std::string outputPath; // output file
    std::string cfgFile;
    double timeLimit;

    desc.add_options()
      ("help", "produce help message")
      ("env,e", po::value<std::string>(&envFile)->required(), "problem instance file (yaml)")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputPath)->required(), "output directory")
      ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")
      ("timelimit,t", po::value<double>(&timeLimit)->required(), "time limit for the discrete search (ms)");

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
    fs::path output_path(outputPath);
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    cfg = cfg["db-ecbs"]["default"];
    float alpha = cfg["alpha"].as<float>();
    bool filter_duplicates = cfg["filter_duplicates"].as<bool>();
    bool save_expanded_trajs = cfg["save_expanded_trajs"].as<bool>();
    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.search_timelimit = timeLimit;
    options_tdbastar.cost_delta_factor = 0;
    options_tdbastar.fix_seed = 1;
    options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
    options_tdbastar.w = cfg["suboptimality_factor"].as<float>(); 
    options_tdbastar.rewire = cfg["rewire"].as<bool>();
    options_tdbastar.always_add_node = cfg["always_add_node"].as<bool>();
    
    size_t robot_id_to_check = 8; 
    size_t step = 1;

    // problem
    dynobench::Problem problem(envFile);
    dynobench::Problem problem_original(envFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
    Out_info_tdb out_tdb;
    std::cout << "*** options tdbastar-epsilon ***" << std::endl;
    options_tdbastar.print(std::cout);
    std::cout << "***" << std::endl;
    // load problem description
    YAML::Node env = YAML::LoadFile(envFile);
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> collision_geometries;
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
        position_bounds.setLow(i, env_min[i].as<double>());
        position_bounds.setHigh(i, env_max[i].as<double>());
    }
    fcl::AABBf workspace_aabb(
    fcl::Vector3f(env_min[0].as<double>(), env_min[1].as<double>(),-1),
    fcl::Vector3f(env_max[0].as<double>(), env_max[1].as<double>(), 1));

    std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
    std::string motionsFile;
    std::vector<std::string> all_motionsFile;
    for (const auto &robotType : problem.robotTypes){
        std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
                (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
        robots.push_back(robot);
        if (robotType == "unicycle1_v0" || robotType == "unicycle1_sphere_v0"){
            motionsFile = "../new_format_motions/unicycle1_v0/unicycle1_v0.msgpack";
        } else if (robotType == "unicycle2_v0"){
            motionsFile = "../new_format_motions/unicycle2_v0/unicycle2_v0.msgpack";
        } else if (robotType == "car1_v0"){
            motionsFile = "../new_format_motions/car1_v0/car1_v0.msgpack";
        } else if (robotType == "integrator2_2d_v0"){
            motionsFile = "../new_format_motions/integrator2_2d_v0/integrator2_2d_v0.msgpack";
        } else if (robotType == "integrator2_3d_v0"){
            motionsFile = "../new_format_motions/integrator2_3d_v0/long_50/integrator2_3d_v0.bin.im.bin.sp.bin";
        } else{
            throw std::runtime_error("Unknown motion filename for this robottype!");
        }
        all_motionsFile.push_back(motionsFile);
    }
    std::map<std::string, std::vector<Motion>> robot_motions;
    // allocate data for conflict checking, check for conflicts
    std::vector<fcl::CollisionObjectd*> robot_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    size_t col_geom_id = 0;
    col_mng_robots->setup();
    size_t i = 0;
    for (const auto &robot : robots){
        collision_geometries.insert(collision_geometries.end(), 
                                robot->collision_geometries.begin(), robot->collision_geometries.end());
        auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
        collision_geometries[col_geom_id]->setUserData((void*)i);
        robot_objs.push_back(robot_obj);
        if (robot_motions.find(problem.robotTypes[i]) == robot_motions.end()){
            options_tdbastar.motionsFile = all_motionsFile[i];
            load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions[problem.robotTypes[i]], 
                                        options_tdbastar.max_motions,
                                        options_tdbastar.cut_actions, false, options_tdbastar.check_cols);
        }
        if (robot->name == "car_with_trailers") {
            col_geom_id++;
            auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
            collision_geometries[col_geom_id]->setUserData((void*)i); // for the trailer
            robot_objs.push_back(robot_obj);
        }
        col_geom_id++;
        i++;
    }
    col_mng_robots->registerObjects(robot_objs);
    std::vector<dynobench::Trajectory> expanded_trajs_tmp;
    std::vector<LowLevelPlan<dynobench::Trajectory>> tmp_solutions(robots.size());
    std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>>*> heuristics(robots.size(), nullptr);
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>>* heuristic = nullptr;

    if (cfg["heuristic1"].as<std::string>() == "reverse-search"){
      options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
      auto robot = robots[robot_id_to_check];
      // start to inf for the reverse search
      problem.starts[robot_id_to_check].head(robot->translation_invariance).setConstant(std::sqrt(std::numeric_limits<double>::max()));
      Eigen::VectorXd tmp_state = problem.starts[robot_id_to_check];
      problem.starts[robot_id_to_check] = problem.goals[robot_id_to_check];
      problem.goals[robot_id_to_check] = tmp_state;
      options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id_to_check]]; 
      // tdbA*epsilon
      LowLevelPlan<dynobench::Trajectory> tmp_solution;
      tdbastar_epsilon(problem, options_tdbastar, 
            tmp_solution.trajectory,/*constraints*/{},
            out_tdb, robot_id_to_check,/*reverse_search*/true, 
            expanded_trajs_tmp, tmp_solutions, robot_motions,
            robots, col_mng_robots, robot_objs,
            nullptr, &heuristics[robot_id_to_check], options_tdbastar.w);
      std::cout << "computed heuristic_epsilon with " << heuristics[robot_id_to_check]->size() << " entries." << std::endl;
      if (save_expanded_trajs){
        std::ofstream out3(output_path.string() + "reverse.yaml");
        out3 << "trajs:" << std::endl;
        for (auto i = 0; i < expanded_trajs_tmp.size(); i += 100){
            auto traj = expanded_trajs_tmp.at(i);
            out3 << "  - " << std::endl;
            traj.to_yaml_format(out3, "    ");
        }
      }
    }
   
    // custom sizes, not read from problems
    HighLevelNodeFocal start;
    start.solution.resize(env["robots"].size());
    start.constraints.resize(env["robots"].size());
    start.result.resize(env["robots"].size());
    start.cost = 0;
    start.id = 0;
    start.LB = 0;
    // read the provided trajectory of the neighbor
    size_t j = 0; // keep track of robot_idx
    // for(size_t i = 0; i < 1; i++){ // robots.size()
    //     if (i == robot_id_to_check){
    //         continue;
    //     }
    //     read_input_yaml(inputFile, start.solution.at(i).trajectory, j); 
    //     start.solution[i].trajectory.cost = start.solution[i].trajectory.actions.size() * robots.at(i)->ref_dt; 
    //     j++;
    // }
    read_input_yaml(inputFile, start.solution.at(0).trajectory, 0); 
    // read_input_yaml(inputFile, start.solution.at(1).trajectory, 1); 
    // common parameters
    options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id_to_check]]; 
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
    options_tdbastar.delta = cfg["delta_0"].as<float>();
    // run tdbA* epsilon, solution of the other robots are already added
    expanded_trajs_tmp.clear();
    
    tdbastar_epsilon(problem, options_tdbastar, 
        start.solution[robot_id_to_check].trajectory, start.constraints[robot_id_to_check],
        out_tdb, robot_id_to_check,/*reverse_search*/false, 
        expanded_trajs_tmp, start.solution, robot_motions,
        robots, col_mng_robots, robot_objs,
        heuristics[robot_id_to_check], nullptr, options_tdbastar.w);
        
    if(!out_tdb.solved){
        std::cout << "tdbA*-epsilon couldn't find initial solution."<< std::endl;
        if (save_expanded_trajs){
            std::ofstream out3(output_path.string() + "failed.yaml");
            out3 << "trajs:" << std::endl;
            for (auto i = 0; i < expanded_trajs_tmp.size(); i += 100){
                auto traj = expanded_trajs_tmp.at(i);
                out3 << "  - " << std::endl;
                traj.to_yaml_format(out3, "    ");
            }
        }
        return 0;
    }
    else {
        std::cout << "tdbA*-epsilon final solution with cost: " << start.solution[robot_id_to_check].trajectory.cost << std::endl; 
        std::cout << output_path.string() << std::endl;
        std::string out_file_tdb = output_path.string() + "tdb_epsilon_solution.yaml";
        create_dir_if_necessary(out_file_tdb);
        std::ofstream out(out_file_tdb);
        export_solutions(start.solution, &out);
        if (save_expanded_trajs){
            size_t step = 1;
            if (expanded_trajs_tmp.size() > 1000)
                step = 100;
            std::ofstream out3(output_path.string() + "forward.yaml");
            out3 << "trajs:" << std::endl;
            for (auto i = 0; i < expanded_trajs_tmp.size(); i += step){
                auto traj = expanded_trajs_tmp.at(i);
                out3 << "  - " << std::endl;
                traj.to_yaml_format(out3, "    ");
            }
        }
    }
    return 0;
}