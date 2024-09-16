#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <bits/stdc++.h>
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
#include <dynobench/multirobot_trajectory.hpp>

// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "dbcbs_utils.hpp"

using namespace dynoplan;
namespace fs = std::filesystem;

#define DYNOBENCH_BASE "../dynoplan/dynobench/"
#define REBUILT_FOCAL_LIST
#define CHECK_FOCAL_LIST

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string optimizationFile;
    std::string cfgFile;
    double timeLimit;

    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)")
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
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    // cfg = cfg["db-ecbs"]["default"];
    float alpha = cfg["alpha"].as<float>();
    bool filter_duplicates = cfg["filter_duplicates"].as<bool>();
    fs::path output_path(outputFile);
    std::string output_folder = output_path.parent_path().string();
    bool save_search_video = false;
    bool save_expanded_trajs = cfg["save_expanded_trajs"].as<bool>();
    std::string conflicts_folder = output_folder + "/conflicts";
    bool ellipsoid_collision_shape = cfg["ellipsoid_collision_shape"].as<bool>();
    Eigen::Vector3d radii = Eigen::Vector3d(.12, .12, .3); // from tro paper
    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.outFile = outputFile;
    options_tdbastar.search_timelimit = timeLimit;
    options_tdbastar.cost_delta_factor = 0;
    options_tdbastar.fix_seed = 1;
    options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
    options_tdbastar.w = cfg["suboptimality_factor"].as<float>(); 
    options_tdbastar.rewire = cfg["rewire"].as<bool>();
    options_tdbastar.always_add_node = cfg["always_add_node"].as<bool>();
    // options_tdbastar.max_expands = 200;
    // tdbastar problem
    dynobench::Problem problem(inputFile);
    dynobench::Problem problem_original(inputFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
    Out_info_tdb out_tdb;
    std::cout << "*** options_tdbastar ***" << std::endl;
    options_tdbastar.print(std::cout);
    std::cout << "***" << std::endl;

    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> collision_geometries;
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
    // std::vector<dynobench::Trajectory> ll_trajs;
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
        if(ellipsoid_collision_shape && robot->name == "Integrator2_3d"){
          collision_geometries.push_back(std::make_shared<fcl::Ellipsoidd>(radii)); // for inter-robot collision checking
        }
        else{
          collision_geometries.insert(collision_geometries.end(), 
                              robot->collision_geometries.begin(), robot->collision_geometries.end());
        }
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
    // Heuristic computation
    size_t robot_id = 0;
    std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>>*> heuristics(robots.size(), nullptr);
    std::vector<dynobench::Trajectory> expanded_trajs_tmp;
    std::vector<LowLevelPlan<dynobench::Trajectory>> tmp_solutions(robots.size());
    if (cfg["heuristic1"].as<std::string>() == "reverse-search"){
      std::map<std::string, std::vector<Motion>> robot_motions_reverse;
      options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
      options_tdbastar.max_motions = cfg["heuristic1_num_primitives_0"].as<size_t>();
      
      std::cout << "Running the reverse search" << std::endl;
      auto reverse_start = std::chrono::high_resolution_clock::now();
      for (const auto &robot : robots){
        // load motions
        if (robot_motions_reverse.find(problem.robotTypes[robot_id]) == robot_motions_reverse.end()){
            options_tdbastar.motionsFile = all_motionsFile[robot_id];
            load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions_reverse[problem.robotTypes[robot_id]], 
                                        options_tdbastar.max_motions,
                                        options_tdbastar.cut_actions, false, options_tdbastar.check_cols);
        }
        // start to inf for the reverse search
        LowLevelPlan<dynobench::Trajectory> tmp_solution;
        problem.starts[robot_id].head(robot->translation_invariance).setConstant(std::sqrt(std::numeric_limits<double>::max()));
        Eigen::VectorXd tmp_state = problem.starts[robot_id];
        problem.starts[robot_id] = problem.goals[robot_id];
        problem.goals[robot_id] = tmp_state;
        expanded_trajs_tmp.clear();
        options_tdbastar.motions_ptr = &robot_motions_reverse[problem.robotTypes[robot_id]]; 
        tdbastar_epsilon(problem, options_tdbastar, 
                tmp_solution.trajectory,/*constraints*/{},
                out_tdb, robot_id,/*reverse_search*/true, 
                expanded_trajs_tmp, tmp_solutions, robot_motions,
                robots, col_mng_robots, robot_objs,
                nullptr, &heuristics[robot_id], options_tdbastar.w);
        std::cout << "computed heuristic with " << heuristics[robot_id]->size() << " entries." << std::endl;
        robot_id++;
      }
      auto reverse_end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = reverse_end - reverse_start;
      std::cout << "Time taken for the reverse search: " << duration.count() << " seconds" << std::endl;
    }
    if (save_search_video){
      std::cout << "***Going to save all intermediate solutions with conflicts!***" << std::endl;
      if (!fs::exists(conflicts_folder)) {
        fs::create_directory(conflicts_folder);
      }
    }
    bool solved_db = false;
    std::cout << "Running the main loop" << std::endl;
    auto discrete_start = std::chrono::high_resolution_clock::now();
    // main loop
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
    options_tdbastar.delta = cfg["delta_0"].as<float>();
    options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
    for (size_t iteration = 0; ; ++iteration) {
      std::cout << "iteration: " << iteration << std::endl;
      if (iteration > 0) {
        if (solved_db) {
            options_tdbastar.delta *= cfg["delta_0"].as<float>();
        } else {
            options_tdbastar.delta *= 0.99;
        }
        options_tdbastar.max_motions *= cfg["num_primitives_rate"].as<float>();
        options_tdbastar.max_motions = std::min<size_t>(options_tdbastar.max_motions, 1e6);
      }
      // disable/enable motions 
      for (auto& iter : robot_motions) {
          for (size_t i = 0; i < problem.robotTypes.size(); ++i) {
              if (iter.first == problem.robotTypes[i]) {
                  disable_motions(robots[i], problem.robotTypes[i], options_tdbastar.delta, filter_duplicates, alpha, 
                                  options_tdbastar.max_motions, iter.second);
                  break;
              }
          }
      }
      solved_db = false;
      HighLevelNodeFocal start;
      start.solution.resize(env["robots"].size());
      start.constraints.resize(env["robots"].size());
      start.result.resize(env["robots"].size());
      start.cost = 0;
      start.id = 0;
      start.LB = 0;
      bool start_node_valid = true;
      robot_id = 0;
      int id = 1;
      std::cout << "Node ID is " << id << ", root" << std::endl;
      for (const auto &robot : robots){
        expanded_trajs_tmp.clear();
        options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id]]; 
        tdbastar_epsilon(problem, options_tdbastar, 
                start.solution[robot_id].trajectory, start.constraints[robot_id],
                out_tdb, robot_id,/*reverse_search*/false, 
                expanded_trajs_tmp, start.solution, robot_motions,
                robots, col_mng_robots, robot_objs,
                heuristics[robot_id], nullptr, options_tdbastar.w);
        if(!out_tdb.solved){
          std::cout << "Couldn't find initial solution for robot " << robot_id << "." << std::endl;
          start_node_valid = false;
          break;
        }
        start.cost += start.solution[robot_id].trajectory.cost;
        start.LB += start.solution[robot_id].trajectory.fmin;
        robot_id++;
      }
      start.focalHeuristic = highLevelfocalHeuristicState(start.solution, robots, col_mng_robots, robot_objs); 
      if (!start_node_valid) {
            continue;
      }
      
      openset_t open;
      focalset_t focal;

      auto handle = open.push(start);
      (*handle).handle = handle;
      focal.push(handle);
      
      
      size_t expands = 0;
      double best_cost = (*handle).cost;
      
      while (!open.empty()){
#ifdef REBUILT_FOCAL_LIST 
          focal.clear();
          double LB = open.top().LB;
          auto iter = open.ordered_begin();
          auto iterEnd = open.ordered_end();
          for (; iter != iterEnd; ++iter) {
            auto cost = (*iter).cost;
            if (cost <= LB * options_tdbastar.w) {
              const HighLevelNodeFocal& n = *iter;
              focal.push(n.handle);
            }
            else {
              break;
            }
          }
#else 
        {
          double oldbest_best_cost = best_cost;
          best_cost = open.top().cost;
          if (best_cost > oldbest_best_cost) {
            auto iter = open.ordered_begin();
            auto iterEnd = open.ordered_end();
            for (; iter != iterEnd; ++iter) {
              auto cost = (*iter).cost;
              if (cost > oldbest_best_cost * options_tdbastar.w && cost <= best_cost * options_tdbastar.w) { // check, LB ?
                const HighLevelNodeFocal& n = *iter;
                focal.push(n.handle);
              }
              if (cost > best_cost * options_tdbastar.w) {
                break;
              }
            }
          }
        }
#endif
#ifdef CHECK_FOCAL_LIST 
          bool mismatch = false;
          auto LB_test = open.top().LB; // ? cost in Wolfgang's code
          auto iter_test = open.ordered_begin();
          auto iterEnd_test = open.ordered_end();
          for (; iter_test != iterEnd_test; ++iter_test) {
            const auto& node_test = *iter_test;
            auto cost_test = node_test.cost;
            if (cost_test <= LB_test * options_tdbastar.w) {
              if (std::find(focal.begin(), focal.end(), node_test.handle) ==
                  focal.end()) {
                std::cout << "focal misses some nodes " << std::endl;
                mismatch = true;
              }

            } else {
              if (std::find(focal.begin(), focal.end(), node_test.handle) !=
                  focal.end()) {
                std::cout << "focalSet shouldn't have some nodes "  << std::endl;
                mismatch = true;
              }
            }
          }
          assert(!mismatch);
#endif
        std::cout << "high-level Open set size: " << open.size() << std::endl;
        std::cout << "high-level Focal set size: " << focal.size() << std::endl;
        std::cout << "cost bound: " << LB * options_tdbastar.w << std::endl;
        auto current_handle = focal.top();
        HighLevelNodeFocal P = *current_handle;
        std::cout << "high-level best node focalHeuristic: " << P.focalHeuristic << std::endl;
        focal.pop();
        open.erase(current_handle);

        Conflict inter_robot_conflict;
        if (!getEarliestConflict(P.solution, robots, col_mng_robots, robot_objs, inter_robot_conflict)){
          solved_db = true;
          auto discrete_end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> duration = discrete_end - discrete_start;
          std::cout << "Final solution from db-ecbs!" << std::endl; 
          create_dir_if_necessary(outputFile);
          std::ofstream out_db(outputFile);
          export_solutions(P.solution, &out_db);
          std::cout << "Number of HL nodes: " << id << std::endl;
          std::cout << "Time taken for discrete search: " << duration.count() << " seconds" << std::endl;
          return 0;
          // priority-based optimization, no smart prioritization - sequentially only
          // r1, r2 with r1 as moving obst., r3 with r1, r2 as moving obs., etc.
          if (cfg["priority_based_optimization"].as<bool>()){
            auto pr_opt_start = std::chrono::high_resolution_clock::now();
            Options_trajopt pr_opt_traj;
            pr_opt_traj.solver_id = 0;
            pr_opt_traj.control_bounds = 1;
            pr_opt_traj.use_warmstart = 1;
            pr_opt_traj.weight_goal = 400;
            pr_opt_traj.max_iter = 50;
            pr_opt_traj.soft_control_bounds = true; 
            std::unordered_set<size_t> other_robots; 
            MultiRobotTrajectory multirobot_sol; // for the priority optimization
            multirobot_sol.trajectories.resize(robots.size());
            for(size_t i = 0; i < robots.size(); i++){
              if(i > 0)
                other_robots.insert(i-1); // skip the first robot, no moving obstacles for it
              // 1. get the environment (use the tmpNode.solution=already optimized to create moving obstacles)
              std::string tmp_envFile = "/tmp/dynoplan/tmp_envFile_" + gen_random(5) + ".yaml";
              create_dir_if_necessary(tmp_envFile);
              get_desired_moving_obstacles(/*envFile*/inputFile, multirobot_sol,/*robot_idx_sol*/P.solution.at(i).trajectory,
                                          /*outFile*/tmp_envFile, i, other_robots);
              // 2. create the problem
              dynobench::Problem tmp_problem(tmp_envFile);
              tmp_problem.models_base_path = DYNOBENCH_BASE "models/";
              // 3. run the trajectory optimization
              Result_opti tmp_opti_out;
              trajectory_optimization(tmp_problem, P.solution.at(i).trajectory, pr_opt_traj, 
                                      multirobot_sol.trajectories.at(i), tmp_opti_out);
              if(!tmp_opti_out.success){
                std::cout << "Priority-based optimization failed for robot: " << i << std::endl;
                return 0;
              }
            }
            std::cout << "Priority-based optimization is done." << std::endl;
            multirobot_sol.to_yaml_format(optimizationFile.c_str());
            auto pr_opt_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> pr_opt_duration = pr_opt_end - pr_opt_start;
            std::cout << "Time taken for the priority-based optimization: " << pr_opt_duration.count() << " seconds" << std::endl;
            return 0;
          }
        }
        ++expands;
        if (expands % 100 == 0) {
         std::cout << "HL expanded: " << expands << " open: " << open.size() << " cost " << P.cost << " conflict at " << inter_robot_conflict.time << std::endl;
        }

        std::map<size_t, std::vector<Constraint>> constraints;
        createConstraintsFromConflicts(inter_robot_conflict, constraints);
        if(save_search_video){
          // get the plot of high-level node solution with conflicts
          auto filename = conflicts_folder + "/" + std::to_string(P.id) + ".yaml";
          std::cout << filename << std::endl;
          std::ofstream int_out(filename);
          export_intermediate_solutions(P.solution, P.constraints, inter_robot_conflict, &int_out);
        }
        
        for (const auto& c : constraints){
          id++;
          HighLevelNodeFocal newNode = P;
          size_t tmp_robot_id = c.first;
          newNode.id = id;
          std::cout << "Node ID is " << id << std::endl;
          newNode.constraints[tmp_robot_id].insert(newNode.constraints[tmp_robot_id].end(), c.second.begin(), c.second.end());
          newNode.cost -= newNode.solution[tmp_robot_id].trajectory.cost;
          newNode.LB -= newNode.solution[tmp_robot_id].trajectory.fmin;
          Out_info_tdb tmp_out_tdb; 
          expanded_trajs_tmp.clear();
          options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[tmp_robot_id]]; 
          tdbastar_epsilon(problem, options_tdbastar, 
                newNode.solution[tmp_robot_id].trajectory, newNode.constraints[tmp_robot_id],
                tmp_out_tdb, tmp_robot_id, /*reverse_search*/false, 
                expanded_trajs_tmp, newNode.solution, robot_motions,
                robots, col_mng_robots, robot_objs,
                heuristics[tmp_robot_id], nullptr, options_tdbastar.w, /*run_focal_heuristic*/true);
          if (tmp_out_tdb.solved){
              newNode.cost += newNode.solution[tmp_robot_id].trajectory.cost;
              newNode.LB += newNode.solution[tmp_robot_id].trajectory.fmin;
              newNode.focalHeuristic = highLevelfocalHeuristicState(newNode.solution, robots, col_mng_robots, robot_objs); 
              std::cout << "New node solution cost:  " << newNode.solution[tmp_robot_id].trajectory.cost << std::endl;
              std::cout << "New node cost: " << newNode.cost << " New node LB: " << newNode.LB << std::endl;
              std::cout << "New node focal heuristic: " << newNode.focalHeuristic << std::endl;
              auto handle = open.push(newNode);
              (*handle).handle = handle;
              // if (newNode.cost <= best_cost * options_tdbastar.w)
              if (newNode.cost <= open.top().LB * options_tdbastar.w){ 
                focal.push(handle);
              }
          }
        } 

      } 

    } 
   
  return 0;
}
