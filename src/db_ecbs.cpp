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
#include "dynoplan/ompl/robots.h"
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
using duration = std::chrono::duration<double>;

int main(int argc, char *argv[])
{

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string optimizationFile;
  std::string statsFile;
  std::string cfgFile;
  double timeLimit;

  desc.add_options()("help", "produce help message")("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)")("stats,s", po::value<std::string>(&statsFile)->required(), "stats file (yaml)")("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")("time_limit,t", po::value<double>(&timeLimit)->required(), "time limit for search");

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch (po::error &e)
  {
    std::cerr << e.what() << std::endl
              << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  create_dir_if_necessary(statsFile);
  std::ofstream stats(statsFile, std::ios::app);
  if (!stats)
  {
    std::cerr << "Failed to open stats.yaml file.\n";
    return 1;
  }
  auto dbecbs_start = std::chrono::steady_clock::now();
  duration duration_discrete, duration_opt;

  YAML::Node cfg = YAML::LoadFile(cfgFile);
  cfg = cfg["db-ecbs"]["default"];
  float alpha = cfg["alpha"].as<float>();
  bool filter_duplicates = cfg["filter_duplicates"].as<bool>();
  fs::path output_path(outputFile);
  std::string output_folder = output_path.parent_path().string();
  bool save_search_video = false;
  bool save_expanded_trajs = cfg["save_expanded_trajs"].as<bool>();
  std::string conflicts_folder = output_folder + "/conflicts";
  Eigen::Vector3d radii = Eigen::Vector3d(.12, .12, .3);
  // optimization-related params
  bool sum_robot_cost = true;
  bool feasible = false;
  // tdbstar options
  Options_tdbastar options_tdbastar;
  options_tdbastar.search_timelimit = timeLimit;
  options_tdbastar.cost_delta_factor = 0;
  options_tdbastar.fix_seed = 1;
  options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
  options_tdbastar.w = cfg["suboptimality_factor"].as<float>();
  options_tdbastar.rewire = cfg["rewire"].as<bool>();
  options_tdbastar.always_add_node = cfg["always_add_node"].as<bool>();
  // tdbastar problem
  dynobench::Problem problem(inputFile);
  dynobench::Problem problem_original(inputFile);
  std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
  problem.models_base_path = models_base_path;
  problem.is_residual = cfg["residual_force"].as<bool>();
  problem.is_conservative = cfg["conservative"].as<bool>();

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
  for (size_t i = 0; i < env_min.size(); ++i)
  {
    position_bounds.setLow(i, env_min[i].as<double>());
    position_bounds.setHigh(i, env_max[i].as<double>());
  }

  fcl::AABBf workspace_aabb(
      fcl::Vector3f(env_min[0].as<double>(),
                    env_min[1].as<double>(), -1),
      fcl::Vector3f(env_max[0].as<double>(), env_max[1].as<double>(), 1));
  size_t robot_id = 0;
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  std::string motionsFile;
  std::vector<std::string> all_motionsFile;

  if (problem.is_residual)
  { // considers only integrator2_3d dynamics
    std::vector<double> _start, _goal;
    for (auto &robotType : problem.robotTypes)
    {
      if (robotType == "integrator2_3d_large_v0")
        robotType = "integrator2_3d_res_large_v0";
      else
        robotType = "integrator2_3d_res_v0";
      // manually add the f to the state
      problem.starts.at(robot_id).conservativeResize(problem.starts.at(robot_id).size() + 1);
      problem.starts.at(robot_id)(problem.starts.at(robot_id).size() - 1) = 0;
      problem.goals.at(robot_id).conservativeResize(problem.goals.at(robot_id).size() + 1);
      problem.goals.at(robot_id)(problem.goals.at(robot_id).size() - 1) = 0;
      // problem.start, problem.goal need to change for joint-optimization
      std::vector<double> tmp_vec1(problem.starts.at(robot_id).data(), problem.starts.at(robot_id).data() + problem.starts.at(robot_id).size());
      _start.insert(_start.end(), tmp_vec1.begin(), tmp_vec1.end());
      std::vector<double> tmp_vec2(problem.goals.at(robot_id).data(), problem.goals.at(robot_id).data() + problem.goals.at(robot_id).size());
      _goal.insert(_goal.end(), tmp_vec2.begin(), tmp_vec2.end());
      ++robot_id;
    }
    problem.start = Eigen::VectorXd::Map(_start.data(), _start.size());
    problem.goal = Eigen::VectorXd::Map(_goal.data(), _goal.size());
  }

  for (auto &robotType : problem.robotTypes)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
    robots.push_back(robot);
    if (robotType == "unicycle1_v0" || robotType == "unicycle1_sphere_v0")
    {
      motionsFile = "../new_format_motions/unicycle1_v0/unicycle1_v0.msgpack";
    }
    else if (robotType == "unicycle1_3d_v0")
    {
      motionsFile = "../new_format_motions/unicycle1_3d_v0/unicycle1_3d_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "unicycle2_v0")
    {
      motionsFile = "../new_format_motions/unicycle2_v0/unicycle2_v0.msgpack";
    }
    else if (robotType == "car1_v0")
    {
      motionsFile = "../new_format_motions/car1_v0/car1_v0.msgpack";
    }
    else if (robotType == "integrator2_2d_v0")
    {
      motionsFile = "../new_format_motions/integrator2_2d_v0/integrator2_2d_v0.msgpack";
    }
    else if (robotType == "integrator2_3d_v0" || robotType == "integrator2_3d_large_v0")
    {
      motionsFile = "../new_format_motions/integrator2_3d_v0/long_50_5000/integrator2_3d_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "quad3d_v0")
    {
      motionsFile = "../new_format_motions/quad3d_v0/quad3d_v0.bin.im.bin.sp.bin";
    }
    else if (robotType.find("_res_") != std::string::npos)
    {
      motionsFile = "../new_format_motions/integrator2_3d_v0/residual/long_50_5000/integrator2_3d_v0.bin.im.bin.sp.bin";
    }
    else
    {
      throw std::runtime_error("Unknown motion filename for this robottype!");
    }
    all_motionsFile.push_back(motionsFile);
  }
  std::map<std::string, std::vector<Motion>> robot_motions; // all motions
  std::map<std::string, std::vector<Motion>> sub_motions;   // used for the search
  // allocate data for conflict checking, check for conflicts
  std::vector<fcl::CollisionObjectd *> robot_objs;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
  col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots->setup();
  size_t col_geom_id = 0;
  size_t i = 0;
  std::map<size_t, std::vector<size_t>> rob_obj_set;
  for (const auto &robot : robots)
  {
    if (problem.is_conservative)
    {
      collision_geometries.push_back(std::make_shared<fcl::Ellipsoidd>(radii));
    }
    else
      collision_geometries.insert(collision_geometries.end(),
                                  robot->collision_geometries.begin(), robot->collision_geometries.end());
    auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
    collision_geometries[col_geom_id]->setUserData((void *)i);
    robot_objs.push_back(robot_obj);
    if (robot_motions.find(problem.robotTypes[i]) == robot_motions.end())
    {
      options_tdbastar.motionsFile = all_motionsFile[i];
      load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions[problem.robotTypes[i]],
                                 /*options_tdbastar.max_motions*/ 1e6,
                                 options_tdbastar.cut_actions, /*shuffle*/ true, options_tdbastar.check_cols);
      // get the needed submotions for the search part
      motion_to_motion(robot_motions[problem.robotTypes[i]], sub_motions[problem.robotTypes[i]], *robot, options_tdbastar.max_motions);
    }
    rob_obj_set[i].push_back(col_geom_id);
    if (robot->name == "car_with_trailers")
    {
      col_geom_id++;
      auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
      collision_geometries[col_geom_id]->setUserData((void *)i); // for the trailer
      robot_objs.push_back(robot_obj);
      rob_obj_set[i].push_back(col_geom_id);
    }
    col_geom_id++;
    i++;
  }
  col_mng_robots->registerObjects(robot_objs);
  robot_id = 0;
  size_t num_robots = robots.size();
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics(num_robots, nullptr);
  std::vector<dynobench::Trajectory> expanded_trajs_tmp;
  std::vector<LowLevelPlan<dynobench::Trajectory>> tmp_solutions(num_robots);
  std::vector<double> upper_bounds(num_robots, std::numeric_limits<double>::max());
  std::vector<double> hs(num_robots, -1.0); // start->hScore

  double cost_tmp = 0;
  double lowest_cost = std::numeric_limits<double>::max();
  YAML::Node itr_cost_data;
  std::string itr_cost_file = output_folder + "/iteration_cost.yaml";
  bool check_anytime = false;

  if (cfg["heuristic1"].as<std::string>() == "reverse-search")
  {
    std::map<std::string, std::vector<Motion>> robot_motions_reverse;
    options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
    options_tdbastar.max_motions = cfg["heuristic1_num_primitives_0"].as<size_t>();

    std::cout << "Running the reverse search" << std::endl;
    auto reverse_start = std::chrono::steady_clock::now();
    for (const auto &robot : robots)
    {
      // load motions
      if (robot_motions_reverse.find(problem.robotTypes[robot_id]) == robot_motions_reverse.end())
      {
        options_tdbastar.motionsFile = all_motionsFile[robot_id];
        load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions_reverse[problem.robotTypes[robot_id]],
                                   options_tdbastar.max_motions,
                                   options_tdbastar.cut_actions, true, options_tdbastar.check_cols);
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
                       tmp_solution.trajectory, /*constraints*/ {},
                       out_tdb, robot_id, upper_bounds[robot_id], hs[robot_id], rob_obj_set, /*reverse_search*/ true,
                       expanded_trajs_tmp, tmp_solutions,
                       robots, col_mng_robots, robot_objs,
                       nullptr, &heuristics[robot_id], /*residual_force*/ false, options_tdbastar.w);
      std::cout << "computed heuristic with " << heuristics[robot_id]->size() << " entries." << std::endl;
      robot_id++;
    }

    auto reverse_end = std::chrono::steady_clock::now();
    duration duration_reverse = reverse_end - reverse_start;
    std::cout << "Time taken for the reverse search: " << duration_reverse.count() << " seconds" << std::endl;
  }
  if (save_search_video)
  {
    std::cout << "***Going to save all intermediate solutions with conflicts!***" << std::endl;
    if (!fs::exists(conflicts_folder))
    {
      fs::create_directory(conflicts_folder);
    }
  }
  bool solved_db = false;
  std::cout << "Running the main loop" << std::endl;
  // main loop
  problem.starts = problem_original.starts;
  problem.goals = problem_original.goals;
  options_tdbastar.delta = cfg["delta_0"].as<float>();
  options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
  // options_tdbastar.max_expands = 10000; // limit the low-level node expansion
  stats << "stats: " << "\n";
  for (size_t iteration = 0;; ++iteration)
  {
    std::cout << "iteration: " << iteration << std::endl;
    if (iteration > 0)
    {
      if (solved_db)
        options_tdbastar.delta *= cfg["delta_rate"].as<float>();
      // always add motions
      options_tdbastar.max_motions *= cfg["num_primitives_rate"].as<float>();
      options_tdbastar.max_motions = std::min<size_t>(options_tdbastar.max_motions, 1e6);
      for (auto &iter : robot_motions)
      {
        for (size_t i = 0; i < problem.robotTypes.size(); ++i)
        {
          if (iter.first == problem.robotTypes[i])
          {
            motion_to_motion(robot_motions[problem.robotTypes[i]], sub_motions[problem.robotTypes[i]], *robots[i], options_tdbastar.max_motions);
            break;
          }
        }
      }
    }
    // disable/enable motions
    for (auto &iter : sub_motions)
    {
      for (size_t i = 0; i < problem.robotTypes.size(); ++i)
      {
        if (iter.first == problem.robotTypes[i])
        {
          disable_motions(robots[i], problem.robotTypes[i], options_tdbastar.delta, filter_duplicates, alpha,
                          options_tdbastar.max_motions, iter.second);
          break;
        }
      }
    }
    std::cout << "ITR: " << iteration << ", MOTIONS: " << options_tdbastar.max_motions << ", delta: " << options_tdbastar.delta << std::endl;
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
    tmp_solutions.clear();
    std::cout << "Node ID is " << id << ", root" << std::endl;
    auto discrete_start = std::chrono::steady_clock::now();
    for (const auto &robot : robots)
    {
      expanded_trajs_tmp.clear();
      options_tdbastar.motions_ptr = &sub_motions[problem.robotTypes[robot_id]];
      tdbastar_epsilon(problem, options_tdbastar,
                       start.solution[robot_id].trajectory, start.constraints[robot_id],
                       out_tdb, robot_id, upper_bounds[robot_id], hs[robot_id], rob_obj_set, /*reverse_search*/ false,
                       expanded_trajs_tmp, tmp_solutions,
                       robots, col_mng_robots, robot_objs,
                       heuristics[robot_id], nullptr, problem.is_residual, options_tdbastar.w);
      if (!out_tdb.solved)
      {
        std::cout << "Couldn't find initial solution for robot " << robot_id << "." << std::endl;
        start_node_valid = false;
        solved_db = false;
        break;
      }
      start.cost += start.solution[robot_id].trajectory.cost;
      start.LB += start.solution[robot_id].trajectory.fmin;
      robot_id++;
    }
    if (!start_node_valid)
    {
      continue;
    }
    start.focalHeuristic = highLevelfocalHeuristicState(start.solution, robots, problem.robotTypes, col_mng_robots, robot_objs, problem.is_residual);

    openset_t open;
    focalset_t focal;

    auto handle = open.push(start);
    (*handle).handle = handle;
    focal.push(handle);

    size_t expands = 0;
    double best_cost = (*handle).cost;
    // compute the sum of start->hscore, since we don't re-compute heuristics
    double hs_total = std::accumulate(hs.begin(), hs.end(), 0);
    while (!open.empty())
    {
#ifdef REBUILT_FOCAL_LIST
      focal.clear();
      double LB = open.top().LB;
      auto iter = open.ordered_begin();
      auto iterEnd = open.ordered_end();
      for (; iter != iterEnd; ++iter)
      {
        auto cost = (*iter).cost;
        if (cost <= LB * options_tdbastar.w)
        {
          const HighLevelNodeFocal &n = *iter;
          focal.push(n.handle);
        }
        else
        {
          break;
        }
      }
#else
      {
        double oldbest_best_cost = best_cost;
        best_cost = open.top().cost;
        if (best_cost > oldbest_best_cost)
        {
          auto iter = open.ordered_begin();
          auto iterEnd = open.ordered_end();
          for (; iter != iterEnd; ++iter)
          {
            auto cost = (*iter).cost;
            if (cost > oldbest_best_cost * options_tdbastar.w && cost <= best_cost * options_tdbastar.w)
            {
              const HighLevelNodeFocal &n = *iter;
              focal.push(n.handle);
            }
            if (cost > best_cost * options_tdbastar.w)
            {
              break;
            }
          }
        }
      }
#endif
#ifdef CHECK_FOCAL_LIST
      bool mismatch = false;
      auto LB_test = open.top().LB;
      auto iter_test = open.ordered_begin();
      auto iterEnd_test = open.ordered_end();
      for (; iter_test != iterEnd_test; ++iter_test)
      {
        const auto &node_test = *iter_test;
        auto cost_test = node_test.cost;
        if (cost_test <= LB_test * options_tdbastar.w)
        {
          if (std::find(focal.begin(), focal.end(), node_test.handle) ==
              focal.end())
          {
            std::cout << "focal misses some nodes " << std::endl;
            mismatch = true;
          }
        }
        else
        {
          if (std::find(focal.begin(), focal.end(), node_test.handle) !=
              focal.end())
          {
            std::cout << "focalSet shouldn't have some nodes " << std::endl;
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
      std::map<size_t, std::vector<Constraint>> constraints;
      if (!getEarliestConflict(P.solution, robots, col_mng_robots, robot_objs, inter_robot_conflict) && (!problem.is_residual || !getEarliestViolations(P.solution, problem.robotTypes, constraints)))
      {
        std::cout << "Final solution from db-ecbs!" << std::endl;
        create_dir_if_necessary(outputFile);
        std::ofstream out_db(outputFile);
        export_solutions(P.solution, &out_db);
        // export_solutions_joint(P.solution, &out_db);
        auto discrete_end = std::chrono::steady_clock::now();
        duration_discrete = discrete_end - discrete_start;
        std::cout << "Time taken for discrete search: " << duration_discrete.count() << " seconds" << std::endl;
        // return 0;
        // read the discrete search as initial guess
        MultiRobotTrajectory discrete_search_sol;
        discrete_search_sol.read_from_yaml(outputFile.c_str());

        if (cfg["execute_joint_optimization"].as<bool>())
        {
          MultiRobotTrajectory optimization_sol;
          auto start = std::chrono::steady_clock::now();
          feasible = execute_optimizationMetaRobot(problem, // inputFile
                                                   /*initialGuess*/ discrete_search_sol,
                                                   /*solution*/ optimization_sol,
                                                   DYNOBENCH_BASE,
                                                   sum_robot_cost);
          if (feasible)
          {
            solved_db = true;
            std::cout << "Joint optimization is done" << std::endl;
            auto end = std::chrono::steady_clock::now();
            duration_opt = end - start;
            std::cout << "Time taken for joint optimization: " << duration_opt.count() << " seconds" << std::endl;
            auto now = std::chrono::steady_clock::now();
            duration t = now - dbecbs_start;
            // get the optimized solution cost
            // check for lower-bounds
            cost_tmp = 0;
            for (auto &traj : optimization_sol.trajectories)
            {
              cost_tmp += traj.cost;
            }
            for (size_t l = 0; l < num_robots; l++)
            {
              upper_bounds[l] = cost_tmp - (hs_total - hs[l]);
            }
            if (cost_tmp < lowest_cost)
            {
              lowest_cost = cost_tmp;
              optimization_sol.to_yaml_format(optimizationFile.c_str());
              std::cout << "Optimization better solution is saved!" << std::endl;
              stats << "  - t: " << t.count() << "\n";
              stats << "    cost: " << cost_tmp << "\n";
              stats << "    duration_tdbastar_eps: " << duration_discrete.count() << "\n";
              stats << "    duration_opt: " << duration_opt.count() << "\n";
              stats << "    discrete cost: " << P.cost << "\n";
              stats.flush();
              // return 0;
              // take out the time search data
              std::string time_stats = output_folder + "/time_search.yaml";
              if (std::filesystem::exists(time_stats))
              {
                std::cout << "time stats file already exists. Not creating it." << std::endl;
              }
              else
              {
                std::ofstream ofs(time_stats); // Create the file if it doesn't exist
                if (ofs)
                {
                  out_tdb.write_yaml(ofs); // only one robot
                }
                else
                {
                  std::cerr << "Failed to create the time stats file." << std::endl;
                }
              }
              if (check_anytime)
              {
                std::string tmp_File1 = output_folder + "/discrete_" + std::to_string(iteration) + ".yaml";
                discrete_search_sol.to_yaml_format(tmp_File1.c_str());
                // optimization
                std::string tmp_File2 = output_folder + "/optimization_" + std::to_string(iteration) + ".yaml";
                optimization_sol.to_yaml_format(tmp_File2.c_str());
              }
            }
            // extract motions from the solution. Lengths depend on the environment (2D-short(1,2), 3D wall-long(8 length for example))
            extract_motion_primitives(problem, optimization_sol, sub_motions, robots, /*length*/ 8);
            itr_cost_data["runs"].push_back(YAML::Node());
            itr_cost_data["runs"][iteration]["iteration"] = iteration;
            itr_cost_data["runs"][iteration]["lowest_cost"] = lowest_cost;

            std::ofstream file(itr_cost_file);
            if (file.is_open())
            {
              file << itr_cost_data;
              file.close();
              std::cout << "Iteration " << iteration << " and the lowest cost " << lowest_cost << std::endl;
            }
            else
            {
              std::cerr << "Error: Unable to open file for writing." << std::endl;
            }
            return 0;
          }
          // return 0;
          break; // continue with the next iteration
        }
      } // if no collision
      ++expands;
      if (expands % 100 == 0)
      {
        std::cout << "HL expanded: " << expands << " open: " << open.size() << " cost " << P.cost << " conflict at " << inter_robot_conflict.time << std::endl;
      }
      if (constraints.empty())
        createConstraintsFromConflicts(inter_robot_conflict, constraints);
      if (save_search_video)
      {
        // get the plot of high-level node solution with conflicts
        auto filename = conflicts_folder + "/" + std::to_string(P.id) + ".yaml";
        std::cout << filename << std::endl;
        std::ofstream int_out(filename);
        export_intermediate_solutions(P.solution, P.constraints, inter_robot_conflict, &int_out);
      }

      for (const auto &c : constraints)
      {
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
        options_tdbastar.motions_ptr = &sub_motions[problem.robotTypes[tmp_robot_id]];
        tdbastar_epsilon(problem, options_tdbastar,
                         newNode.solution[tmp_robot_id].trajectory, newNode.constraints[tmp_robot_id],
                         tmp_out_tdb, tmp_robot_id, upper_bounds[tmp_robot_id], hs[robot_id], rob_obj_set, /*reverse_search*/ false,
                         expanded_trajs_tmp, newNode.solution,
                         robots, col_mng_robots, robot_objs,
                         heuristics[tmp_robot_id], nullptr, problem.is_residual, options_tdbastar.w, /*run_focal_heuristic*/ true);
        if (tmp_out_tdb.solved)
        {
          newNode.cost += newNode.solution[tmp_robot_id].trajectory.cost;
          newNode.LB += newNode.solution[tmp_robot_id].trajectory.fmin;
          newNode.focalHeuristic = highLevelfocalHeuristicState(newNode.solution, robots, problem.robotTypes, col_mng_robots, robot_objs, problem.is_residual);
          std::cout << "New node solution cost:  " << newNode.solution[tmp_robot_id].trajectory.cost << std::endl;
          std::cout << "New node cost: " << newNode.cost << " New node LB: " << newNode.LB << std::endl;
          std::cout << "New node focal heuristic: " << newNode.focalHeuristic << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          if (newNode.cost <= open.top().LB * options_tdbastar.w)
          {
            focal.push(handle);
          }
        }
      }
    }
  }

  return 0;
}
