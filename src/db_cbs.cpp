#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <bits/stdc++.h>
// fcl
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// others
#include "dbcbs_utils.hpp"

namespace fs = std::filesystem;
using namespace dynoplan;
using duration = std::chrono::duration<double>;
// #define BASE "../../../../" // w.r.t db-CBS/build
#define BASE "../../" // mrmp_benchmark dir
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
  std::optional<unsigned int> seed;

  desc.add_options()("help", "produce help message")
  ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
  ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
  ("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)")
  ("stats", po::value<std::string>(&statsFile)->required(), "stats file (yaml)")
  ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)")
  ("time_limit,t", po::value<double>(&timeLimit)->required(), "time limit for search")
  ("seed", po::value<unsigned int>(), "random seed (default: deterministic)");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  try
  {
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

  // set seeds
  seed = vm.count("seed")
         ? std::optional<unsigned int>(vm["seed"].as<unsigned int>())
         : std::optional<unsigned int>(42); // determenistic behavior
  if (seed.has_value()) {
    std::cout << "db-CBS uses seed " << seed.value() << std::endl;
  } else {
      std::cout << "db-CBS uses seed (default) " << std::endl;
  }
  create_dir_if_necessary(statsFile);
  std::ofstream stats(statsFile, std::ios::app);
  if (!stats)
  {
    std::cerr << "Failed to open stats.yaml file.\n";
    return 1;
  }
  auto dbcbs_start = std::chrono::steady_clock::now();
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  std::cout << cfgFile << std::endl;
  std::cout << cfg << std::endl;
  cfg = cfg["db-cbs"]["default"];
  float alpha = cfg["alpha"].as<float>();
  bool filter_duplicates = cfg["filter_duplicates"].as<bool>();
  std::filesystem::path p(inputFile);
  std::string instanceName = p.filename().string(); // with .yaml
  std::cout << "instance name: " << instanceName << std::endl;
  bool feasible = false;
  // tdbstar options
  Options_tdbastar options_tdbastar;
  options_tdbastar.outFile = outputFile;
  options_tdbastar.search_timelimit = timeLimit;
  options_tdbastar.cost_delta_factor = 0;
  options_tdbastar.fix_seed = 1;
  options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();
  options_tdbastar.rewire = true;
  // tdbastar problem
  dynobench::Problem problem(inputFile);
  dynobench::Problem problem_original(inputFile);
  problem.models_base_path = BASE + std::string("robot_types/");
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
    if (obs["shape"]["type"].as<std::string>() == "box")
    {
      const auto &size = obs["shape"]["size"];
      std::shared_ptr<fcl::CollisionGeometryf> geom;
      geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
      const auto &center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
      co->computeAABB();
      obstacles.push_back(co);
    }
    else if (obs["shape"]["type"].as<std::string>() == "sphere")
    {
      const auto &radius_node = obs["shape"]["radius"];

      float radius = radius_node.as<float>();

      std::shared_ptr<fcl::CollisionGeometryf> geom;
      geom.reset(new fcl::Spheref(radius));

      const auto &center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      co->setTranslation(fcl::Vector3f(center[0].as<float>(),
                                        center[1].as<float>(),
                                        0.0));

      co->computeAABB();
      obstacles.push_back(co);
    }
    else
    {
      throw std::runtime_error("Unknown obstacle type!");
    }
  }
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  ob::RealVectorBounds position_bounds(env_min.size());
  for (size_t i = 0; i < env_min.size(); ++i)
  {
    position_bounds.setLow(i, env_min[i].as<double>());
    position_bounds.setHigh(i, env_max[i].as<double>());
  }

  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  std::vector<dynobench::Trajectory> ll_trajs;
  std::string motionsFile;
  std::vector<std::string> all_motionsFile;
  for (const auto &robotType : problem.robotTypes)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
    robots.push_back(robot);
    if (robotType == "unicycle_first_order" || robotType == "unicycle_sphere_first_order")
    {
      motionsFile = "db-CBS/new_format_motions/unicycle1_v0/spread/unicycle1_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "single_integrator")
    {
      motionsFile = "db-CBS/new_format_motions/integrator1_2d_v0/unit_length2/integrator1_2d_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "double_integrator_2d")
    {
      motionsFile = "db-CBS/new_format_motions/integrator2_2d_v0/integrator2_2d_v0.bin.im.bin.sp.bin.yaml";
    }
    else if (robotType == "double_integrator_3d")
    {
      motionsFile = "db-CBS/new_format_motions/integrator2_3d_v0/short/integrator2_3d_v0.bin.im.bin.sp.bin";
    }
    else
    {
      throw std::runtime_error("Unknown motion filename for this robottype!");
    }
    all_motionsFile.push_back(motionsFile);
  }
  std::map<std::string, std::vector<Motion>> robot_motions;
  // allocate data for conflict checking, check for conflicts
  std::vector<fcl::CollisionObjectd *> robot_objs;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
  col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  size_t col_geom_id = 0;
  col_mng_robots->setup();
  size_t i = 0;
  for (const auto &robot : robots)
  {
    collision_geometries.insert(collision_geometries.end(),
                                robot->collision_geometries.begin(), robot->collision_geometries.end());
    auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
    collision_geometries[col_geom_id]->setUserData((void *)i);
    robot_objs.push_back(robot_obj);
    if (robot_motions.find(problem.robotTypes[i]) == robot_motions.end())
    {
      options_tdbastar.motionsFile = all_motionsFile[i];
      load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions[problem.robotTypes[i]],
                                 options_tdbastar.max_motions,
                                 options_tdbastar.cut_actions, /*shuffle*/true, options_tdbastar.check_cols, seed);
    }
    if (robot->name == "car_with_trailers")
    {
      col_geom_id++;
      auto robot_obj = new fcl::CollisionObject(collision_geometries[col_geom_id]);
      collision_geometries[col_geom_id]->setUserData((void *)i); // for the trailer
      robot_objs.push_back(robot_obj);
    }

    col_geom_id++;
    i++;
  }
  col_mng_robots->registerObjects(robot_objs);
  // Heuristic computation
  size_t robot_id = 0;
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics(robots.size(), nullptr);
  if (cfg["heuristic1"].as<std::string>() == "reverse-search")
  {
    auto reverse_start = std::chrono::steady_clock::now();
    std::map<std::string, std::vector<Motion>> robot_motions_reverse;
    options_tdbastar.delta = cfg["heuristic1_delta"].as<float>();
    options_tdbastar.max_motions = cfg["heuristic1_num_primitives_0"].as<size_t>();
    std::cout << "Running the reverse search" << std::endl;
    for (const auto &robot : robots)
    {
      if (robot_motions_reverse.find(problem.robotTypes[robot_id]) == robot_motions_reverse.end())
      {
        options_tdbastar.motionsFile = all_motionsFile[robot_id];
        load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions_reverse[problem.robotTypes[robot_id]],
                                   options_tdbastar.max_motions,
                                   options_tdbastar.cut_actions, /*shuffle*/ false, options_tdbastar.check_cols);
      }
      // start to inf for the reverse search
      Eigen::VectorXd tmp_state = problem.starts[robot_id];
      problem.starts[robot_id] = problem.goals[robot_id];
      problem.goals[robot_id] = tmp_state;
      LowLevelPlan<dynobench::Trajectory> tmp_solution;
      options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id]];
      tdbastar(problem, options_tdbastar, tmp_solution.trajectory, /*constraints*/ {},
               out_tdb, robot_id, /*reverse_search*/ true, nullptr, &heuristics[robot_id]);
      std::cout << "computed heuristic with " << heuristics[robot_id]->size() << " entries." << std::endl;
      robot_id++;
    }
  }
  bool solved_db = false;
  // main loop
  problem.starts = problem_original.starts;
  problem.goals = problem_original.goals;
  options_tdbastar.delta = cfg["delta_0"].as<float>();
  options_tdbastar.max_motions = cfg["num_primitives_0"].as<size_t>();

  for (size_t iteration = 0;; ++iteration)
  {
    if (iteration > 0)
    {
      if (solved_db)
      {
        options_tdbastar.delta *= cfg["delta_0"].as<float>();
      }
      else
      {
        options_tdbastar.delta *= 0.99;
      }
      options_tdbastar.max_motions *= cfg["num_primitives_rate"].as<float>();
      options_tdbastar.max_motions = std::min<size_t>(options_tdbastar.max_motions, 1e6);
    }
    // disable/enable motions
    for (auto &iter : robot_motions)
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
    solved_db = false;
    HighLevelNode start;
    start.solution.resize(env["robots"].size());
    start.constraints.resize(env["robots"].size());
    start.cost = 0;
    start.id = 0;
    bool start_node_valid = true;
    robot_id = 0;
    for (const auto &robot : robots)
    {
      options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id]];
      tdbastar(problem, options_tdbastar, start.solution[robot_id].trajectory, start.constraints[robot_id],
               out_tdb, robot_id, /*reverse_search*/ false, heuristics[robot_id], nullptr);
      if (!out_tdb.solved)
      {
        std::cout << "Couldn't find initial solution for robot " << robot_id << "." << std::endl;
        start_node_valid = false;
        break;
      }

      start.cost += start.solution[robot_id].trajectory.cost;
      robot_id++;
    }
    if (!start_node_valid)
    {
      continue;
    }
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;
    auto handle = open.push(start);
    (*handle).handle = handle;
    int id = 1;
    size_t expands = 0;
    auto stop_search = [&]() -> bool
    {
        if (elapsed_ms(dbcbs_start) >
            options_tdbastar.search_timelimit)
        {
            return fail_and_write_stats(
                stats,
                "db-CBS",
                instanceName,
                "MAX_TIME");
        }
        if (open.empty())
        {
            return fail_and_write_stats(
                stats,
                "db-CBS",
                instanceName,
                "EMPTY_QUEUE");
        }
        return true;
    };
    while (stop_search())
    {
      HighLevelNode P = open.top();
      open.pop();
      Conflict inter_robot_conflict;
      if (!getEarliestConflict(P.solution, robots, col_mng_robots, robot_objs, inter_robot_conflict))
      {
        solved_db = true;
        std::cout << "Final solution!" << std::endl;
        create_dir_if_necessary(outputFile);
        std::ofstream out(outputFile);
        export_solutions(P.solution, &out);
        // read your discrete solution
        MultiRobotTrajectory discrete_search_sol;
        discrete_search_sol.read_from_yaml(outputFile.c_str());
        bool sum_robot_cost = true;
        MultiRobotTrajectory optimization_sol;
        auto start = std::chrono::steady_clock::now();
        feasible = execute_optimizationMetaRobot(problem, // inputFile
                                                   /*initialGuess*/ discrete_search_sol,
                                                   /*solution*/ optimization_sol,
                                                   BASE,
                                                   sum_robot_cost);

        if (feasible)
        {
          auto now = std::chrono::steady_clock::now();
          duration run_time = now - dbcbs_start;
          double cost = optimization_sol.get_cost();
          double makespan = optimization_sol.get_makespan_steps();
          double control_effort = optimization_sol.get_control_effort();
          optimization_sol.to_yaml_format(optimizationFile.c_str());
          // save stats
          stats << "stats: " << "\n";
          stats << "  - instance: " << instanceName << "\n";
          stats << "    success: " << true << "\n";
          stats << "    elapsed_time_sec: " << run_time.count() << "\n";
          stats << "    makespan_sec: " << makespan << "\n";
          stats << "    sum_of_costs_sec: " << cost * 0.1 << "\n";
          stats << "    total_control_effort: " << control_effort << "\n";
          stats.flush();
          return 0;
        }
        break;
      }
      ++expands;
      if (expands % 100 == 0)
      {
        std::cout << "HL expanded: " << expands << " open: " << open.size() << " cost " << P.cost << " conflict at " << inter_robot_conflict.time << std::endl;
      }

      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);
      for (const auto &c : constraints)
      {
        HighLevelNode newNode = P;
        size_t tmp_robot_id = c.first;
        newNode.id = id;
        std::cout << "Node ID is " << id << std::endl;
        newNode.constraints[tmp_robot_id].insert(newNode.constraints[tmp_robot_id].end(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[tmp_robot_id].trajectory.cost;
#ifdef DBG_PRINTS
        std::cout << "New node cost: " << newNode.cost << std::endl;
#endif
        Out_info_tdb tmp_out_tdb;
        options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[tmp_robot_id]];
        tdbastar(problem, options_tdbastar, newNode.solution[tmp_robot_id].trajectory,
                 newNode.constraints[tmp_robot_id], tmp_out_tdb, tmp_robot_id, /*reverse_search*/ false, heuristics[tmp_robot_id]);
        if (tmp_out_tdb.solved)
        {
          newNode.cost += newNode.solution[tmp_robot_id].trajectory.cost;
#ifdef DBG_PRINTS
          std::cout << "Updated New node cost: " << newNode.cost << std::endl;
#endif
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }
        id++;
      }
    }
  }
  return 0;
}