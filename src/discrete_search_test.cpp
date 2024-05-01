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
void read_input_yaml(const std::string file, dynobench::Trajectory &traj){
    YAML::Node node = YAML::LoadFile(file);
    CHECK(node["result"], "");
    traj.read_from_yaml(node["result"][0]); // assumes the file has solution for only a single robot
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
    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.search_timelimit = timeLimit;
    options_tdbastar.cost_delta_factor = 0;
    options_tdbastar.fix_seed = 1;
    options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
    options_tdbastar.w = 1.3;
    options_tdbastar.rewire = true;
    bool save_expanded_trajs = false;
    // tdbastar problem
    dynobench::Problem problem(envFile);
    dynobench::Problem problem_original(envFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
    Out_info_tdb out_tdb, out_tdb_e;
    std::cout << "*** options tdbastar ***" << std::endl;
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
            motionsFile = "../new_format_motions/integrator2_3d_v0/integrator2_3d_v0.bin.im.bin.sp.bin";
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
    std::vector<LowLevelPlan<dynobench::Trajectory>> tmp_solutions;
    size_t robot_id_to_check = 1;  // robot we run planner for
    // Heuristic computation for tdbA*
    std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>>*> heuristics(robots.size(), nullptr);
    // for tdbA* epsilon
    std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>>*> heuristics_e(robots.size(), nullptr);
    if (cfg["heuristic1"].as<std::string>() == "reverse-search"){
      options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
      auto robot = robots[robot_id_to_check];
      // start to inf for the reverse search
      problem.starts[robot_id_to_check].head(robot->translation_invariance).setConstant(std::sqrt(std::numeric_limits<double>::max()));
      Eigen::VectorXd tmp_state = problem.starts[robot_id_to_check];
      problem.starts[robot_id_to_check] = problem.goals[robot_id_to_check];
      problem.goals[robot_id_to_check] = tmp_state;
      expanded_trajs_tmp.clear();
      options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id_to_check]]; 
      LowLevelPlan<dynobench::Trajectory> tmp_solution;
      // tdbA* 
      tdbastar(problem, options_tdbastar, tmp_solution.trajectory,/*constraints*/{},
        out_tdb, robot_id_to_check,/*reverse_search*/true, expanded_trajs_tmp, nullptr, &heuristics[robot_id_to_check]);
      std::cout << "computed heuristic with " << heuristics[robot_id_to_check]->size() << " entries." << std::endl;

      // tdbA*epsilon
      expanded_trajs_tmp.clear();
      LowLevelPlan<dynobench::Trajectory> tmp_solution_e;
      tdbastar_epsilon(problem, options_tdbastar, 
            tmp_solution_e.trajectory,/*constraints*/{},
            out_tdb, robot_id_to_check,/*reverse_search*/true, 
            expanded_trajs_tmp, tmp_solutions, robot_motions,
            robots, col_mng_robots, robot_objs,
            nullptr, &heuristics_e[robot_id_to_check], options_tdbastar.w);
      std::cout << "computed heuristic_epsilon with " << heuristics_e[robot_id_to_check]->size() << " entries." << std::endl;
       
    }
    // custom sizes, not read from problems
    HighLevelNodeFocal start;
    start.solution.resize(2);
    start.constraints.resize(2);
    start.result.resize(2);
    start.cost = 0;
    start.id = 0;
    start.LB = 0;
    // read the provided trajectory of the neighbor
    read_input_yaml(inputFile, start.solution.at(0).trajectory); // for the provided solution, robot_id=0
    start.solution[0].trajectory.cost = start.solution[0].trajectory.actions.size() * robots.at(0)->ref_dt; // set the cost of the provided solution
    HighLevelNodeFocal start_e = start; 
    // common parameters
    options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id_to_check]]; 
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
    options_tdbastar.delta = cfg["delta_0"].as<float>();
    // run tdbastar
    expanded_trajs_tmp.clear();
    tdbastar(problem, options_tdbastar, start.solution[robot_id_to_check].trajectory, start.constraints[robot_id_to_check],
                out_tdb, robot_id_to_check,/*reverse_search*/false, expanded_trajs_tmp, heuristics[robot_id_to_check], nullptr);
    if(!out_tdb.solved){
        std::cout << "tdbA* couldn't find initial solution."<< std::endl;
    }
    // run tdbA* epsilon, solution of the other robot is already added
    expanded_trajs_tmp.clear();
    tdbastar_epsilon(problem, options_tdbastar, 
        start_e.solution[robot_id_to_check].trajectory,start.constraints[robot_id_to_check],
        out_tdb_e, robot_id_to_check,/*reverse_search*/false, 
        expanded_trajs_tmp, start_e.solution, robot_motions,
        robots, col_mng_robots, robot_objs,
        heuristics_e[robot_id_to_check], nullptr, options_tdbastar.w);
    if(!out_tdb_e.solved){
        std::cout << "tdbA*-epsilon couldn't find initial solution."<< std::endl;
    }
    if (!out_tdb.solved && !out_tdb_e.solved){
        std::cout << "both planners failed, quitting the run!" << std::endl;
        return 0;
    }
    // tdbA*, run and get the solution
    while (true){
        Conflict conflict_tdb;
        out_tdb.solved = false;
        if(!getEarliestConflict(start.solution, robots, col_mng_robots, robot_objs, conflict_tdb)){
            for (size_t i=0; i < start.solution.size(); i++){
                start.cost += start.solution[i].trajectory.cost;
            }
            std::string out_file_tdb = output_path.string() + "/tdb_solution.yaml";
            create_dir_if_necessary(out_file_tdb);
            std::ofstream out(out_file_tdb);
            export_solutions(start.solution, &out);
            out_tdb.solved=true;
            break;
        }

        std::map<size_t, std::vector<Constraint>> constraints_tdb;
        createConstraintsFromConflicts(conflict_tdb, constraints_tdb);
        // grow constraints of the same robot, use the same high-level node
        auto c = constraints_tdb[robot_id_to_check];
        start.constraints[robot_id_to_check].insert(start.constraints[robot_id_to_check].end(), 
                                    c.begin(), c.end());
        expanded_trajs_tmp.clear();
        options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id_to_check]]; 
        tdbastar(problem, options_tdbastar, start.solution[robot_id_to_check].trajectory, 
                 start.constraints[robot_id_to_check], out_tdb, robot_id_to_check,/*reverse_search*/false, 
                  expanded_trajs_tmp, heuristics[robot_id_to_check]);
    }
    // run tdbA*-epsilon, get the solution
    while(true){
        Conflict conflict_tdb_e;
        out_tdb_e.solved = false;
        if(!getEarliestConflict(start_e.solution, robots, col_mng_robots, robot_objs, conflict_tdb_e)){
            for (size_t i=0; i < start_e.solution.size(); i++){
                start_e.cost += start_e.solution[i].trajectory.cost;
            }
            std::string out_file_tdb_e = output_path.string() + "/tdb_epsilon_solution.yaml";
            create_dir_if_necessary(out_file_tdb_e);
            std::ofstream out(out_file_tdb_e);
            export_solutions(start_e.solution, &out);
            out_tdb_e.solved=true;
            break;
        } 
    }
    if (out_tdb.solved){
        std::cout << "tdbA* final solution with cost: " << start.cost << std::endl; 
    }
    if (out_tdb_e.solved){
        std::cout << "tdbA*-epsilon final solution with cost: " << start_e.cost << std::endl; 
    }
    return 0;
}