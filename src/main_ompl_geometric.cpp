#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>

#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string plannerDesc;
  int timelimit;
  float goalRegion;
  std::string robotType;
  desc.add_options()
    ("help", "produce help message")
    ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
    ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
    ("planner,p", po::value<std::string>(&plannerDesc)->default_value("rrt"), "Planner")
    ("timelimit", po::value<int>(&timelimit)->default_value(10), "Time limit for planner")
    ("goalregion", po::value<float>(&goalRegion)->default_value(0.1), "radius around goal to count success")
    ("robottype", po::value<std::string>(&robotType)->required(), "type to use for planning");

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

  // load problem description
  YAML::Node env = YAML::LoadFile(inputFile);

  std::vector<fcl::CollisionObjectf *> obstacles;
  for (const auto &obs : env["environment"]["obstacles"])
  {
    if (obs["type"].as<std::string>() == "box")
    {
      const auto &size = obs["size"];
      std::shared_ptr<fcl::CollisionGeometryf> geom;
      geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
      const auto &center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
      co->computeAABB();
      obstacles.push_back(co);
    }
    else
    {
      throw std::runtime_error("Unknown obstacle type!");
    }
  }
  std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(new fcl::DynamicAABBTreeCollisionManagerf());
  // std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(new fcl::NaiveCollisionManagerf());
  bpcm_env->registerObjects(obstacles);
  bpcm_env->setup();

  const auto& robot_node = env["robots"][0];
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  ob::RealVectorBounds position_bounds(env_min.size());
  for (size_t i = 0; i < env_min.size(); ++i) {
    position_bounds.setLow(i, env_min[i].as<double>());
    position_bounds.setHigh(i, env_max[i].as<double>());
  }
  std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);

  auto si = robot->getSpaceInformation();

  // set state validity checking for this space
  auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, bpcm_env, robot,true));
  si->setStateValidityChecker(stateValidityChecker);

  // set the state propagator
  std::shared_ptr<oc::StatePropagator> statePropagator(new RobotStatePropagator(si, robot));
  si->setStatePropagator(statePropagator);

  si->setup();

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // create and set a start state
  auto startState = si->allocState();
  std::vector<double> reals;
  for (const auto& v : robot_node["start"]) {
    reals.push_back(v.as<double>());
    if (reals.size() == si->getStateSpace()->getDimension()) break;
  }
  si->getStateSpace()->copyFromReals(startState, reals);
  pdef->addStartState(startState);
  si->freeState(startState);

  // set goal state
  auto goalState = si->allocState();
  reals.clear();
  for (const auto &v : robot_node["goal"]) {
    reals.push_back(v.as<double>());
    if (reals.size() == si->getStateSpace()->getDimension()) break;
  }
  si->getStateSpace()->copyFromReals(goalState, reals);
  pdef->setGoalState(goalState, goalRegion);
  si->freeState(goalState);

  // create a planner for the defined space
  std::shared_ptr<ob::Planner> planner;
  if (plannerDesc == "rrt*") {
    planner.reset(new og::RRTstar(si));
  } else if (plannerDesc == "sst") {
    planner.reset(new og::SST(si));
  }
  // rrt->setGoalBias(params["goalBias"].as<float>());
  // auto planner(rrt);

  // auto start = std::chrono::steady_clock::now();
  bool has_solution = false;
  std::chrono::steady_clock::time_point previous_solution;
  pdef->setIntermediateSolutionCallback(
      [&previous_solution, &has_solution](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost cost)
      {
        // double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        // double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - previous_solution).count();
        std::cout << "Intermediate solution! " << cost.value() << std::endl;
        has_solution = true;
        // last_solution_in_sec = dt / 1000.0f;
        previous_solution = std::chrono::steady_clock::now();
      });

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  // (this will set the optimization objective)
  planner->setup();

  // set a really high cost threshold, so that planner stops after first solution was found
  // pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(1e6));

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within timelimit
  ob::PlannerStatus solved;

  // solved = planner->ob::Planner::solve(timelimit);
  // terminate if no better solution is found within the timelimit
  solved = planner->solve(
    ob::PlannerTerminationCondition([&previous_solution, &has_solution]
    {
      if (!has_solution) {
        return false;
      }
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - previous_solution).count() / 1000.0f;
      return dt > 1.0;
    }));
  std::cout << solved << std::endl;

  if (solved)
  {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    // ob::PathPtr path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    // print the path to screen
    // path->print(std::cout);
  }
  else {
    std::cout << "No solution found" << std::endl;
  }

  // std::cout << solved << std::endl;
  auto path = pdef->getSolutionPath()->as<og::PathGeometric>();

  path->interpolate(); // normalize to a single control step
  std::cout << path->getStateCount() << std::endl;

  std::ofstream out(outputFile);
  out << "result:" << std::endl;
  out << "  - pathlength: " << path->length() << std::endl;
  out << "    states:" << std::endl;
  for (size_t i = 0; i < path->getStateCount(); ++i) {
    const auto state = path->getState(i);

    std::vector<double> reals;
    si->getStateSpace()->copyToReals(reals, state);
    out << "      - [";
    for (size_t i = 0; i < reals.size(); ++i) {
      out << reals[i];
      if (i < reals.size() - 1) {
        out << ",";
      }
    }
    out << "]" << std::endl;
  }

  return 0;
}
