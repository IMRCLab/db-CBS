#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>

#include <yaml-cpp/yaml.h>

// #include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/base/objectives/ControlDurationObjective.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string statsFile;
  std::string plannerDesc;
  std::string cfgFile;
  int timelimit;
  desc.add_options()
    ("help", "produce help message")
    ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
    ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
    ("stats", po::value<std::string>(&statsFile)->default_value("ompl_stats.yaml"), "output file (yaml)")
    ("planner,p", po::value<std::string>(&plannerDesc)->default_value("rrt"), "Planner")
    ("timelimit", po::value<int>(&timelimit)->default_value(60), "Time limit for planner")
    ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)");

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

  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  ob::RealVectorBounds position_bounds(env_min.size());
  for (size_t i = 0; i < env_min.size(); ++i) {
    position_bounds.setLow(i, env_min[i].as<double>());
    position_bounds.setHigh(i, env_max[i].as<double>());
  }

  std::vector<std::shared_ptr<Robot>> robots;
  std::vector<double> start_reals;
  std::vector<double> goal_reals;
  std::vector<size_t> state_lengths;
  for (const auto &robot_node : env["robots"]) {
    auto robotType = robot_node["type"].as<std::string>();
    std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);
    robots.push_back(robot);
    for (const auto& v : robot_node["start"]) {
      start_reals.push_back(v.as<double>());
    }
    state_lengths.push_back(start_reals.size());
    for (const auto& v : robot_node["goal"]) {
      goal_reals.push_back(v.as<double>());
    }
  }
  std::shared_ptr<Robot> robot = create_joint_robot(robots);
  // load config file
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  auto si = robot->getSpaceInformation();
  // set number of control steps (use 0.1s as increment -> 0.1 to 1s per Steer function)
  si->setPropagationStepSize(cfg["propagation_step_size"].as<double>());
  si->setMinMaxControlDuration(
    cfg["control_duration"][0].as<int>(),
    cfg["control_duration"][1].as<int>());

  // set state validity checking for this space
  auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, bpcm_env, robot));
  si->setStateValidityChecker(stateValidityChecker);
  // set the state propagator
  std::shared_ptr<oc::StatePropagator> statePropagator(new RobotStatePropagator(si, robot));
  si->setStatePropagator(statePropagator);
  si->setup();
  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // create and set a start state
  auto startState = si->allocState();
  si->getStateSpace()->copyFromReals(startState, start_reals);
  si->enforceBounds(startState);
  pdef->addStartState(startState);
  si->freeState(startState);

  // set goal state
  auto goalState = si->allocState();
  si->getStateSpace()->copyFromReals(goalState, goal_reals);
  std::cout<<goalState;
  si->enforceBounds(goalState);
  pdef->setGoalState(goalState, cfg["goal_epsilon"].as<double>());
  si->freeState(goalState);

  // create a planner for the defined space
  std::shared_ptr<ob::Planner> planner;
  if (plannerDesc == "rrt") {
    auto rrt = new oc::RRT(si);
    rrt->setGoalBias(cfg["goal_bias"].as<double>());
    planner.reset(rrt);
  } else if (plannerDesc == "sst") {
    auto sst = new oc::SST(si);
    sst->setGoalBias(cfg["goal_bias"].as<double>());
    sst->setSelectionRadius(cfg["selection_radius"].as<double>());
    sst->setPruningRadius(cfg["pruning_radius"].as<double>());
    planner.reset(sst);
  }
  // rrt->setGoalBias(params["goalBias"].as<float>());
  // auto planner(rrt);

  pdef->setOptimizationObjective(std::make_shared<ob::ControlDurationObjective>(si));
  // empty stats file
  std::ofstream stats(statsFile);
  stats << "stats:" << std::endl;

  auto start = std::chrono::steady_clock::now();

  pdef->setIntermediateSolutionCallback(
      [start, &stats](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost cost)
      {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        stats << "  - t: " << t/1000.0f << std::endl;
        stats << "    cost: " << cost.value() << std::endl;
        std::cout << "Intermediate solution! " << cost.value() << " " << t/1000.0f << std::endl;
      });

  // set the problem we are trying to solve for the plannerw
  planner->setProblemDefinition(pdef);
  // perform setup steps for the planner
  planner->setup();
  // print the settings for this space
  si->printSettings(std::cout);
  // print the problem settings
  pdef->print(std::cout);
  // attempt to solve the problem within timelimit
  ob::PlannerStatus solved;
  // for (int i = 0; i < 3; ++i) {
  solved = planner->ob::Planner::solve(timelimit);
  std::cout << solved << std::endl;
  // }

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

  std::cout << solved << std::endl;
  auto path = pdef->getSolutionPath()->as<oc::PathControl>();

  // update propagation step size to get right interpolation resolution
  si->setPropagationStepSize(robot->dt());
  path->interpolate(); // normalize to a single control step

  std::cout << path->getStateCount() << "," << path->getControlCount() << std::endl;
  assert(path->getStateCount() == path->getControlCount() + 1);

  std::ofstream out(outputFile);
  out << "result:" << std::endl;
  out << "  - states:" << std::endl;
  for (size_t i = 0; i < path->getStateCount(); ++i) {
    int k = 0;
    const auto state = path->getState(i);
    std::vector<double> reals;
    si->getStateSpace()->copyToReals(reals, state);
    out << "      - [[";
    for (size_t i = 0; i < reals.size(); ++i) {
      if (i < state_lengths[k]){
        out << reals[i];
        if (i < state_lengths[k]-1){
          out << ",";
        }
      }
      if (i == state_lengths[k]) {
        out << "], ";
        if (i < reals.size()-1){
          out << "[";
        }
        out << reals[i] << ",";
        k = k + 1;
      }
    }
    out << "]]" << std::endl;
  }
  out << "    actions:" << std::endl;
  for (size_t i = 0; i < path->getControlCount(); ++i) {

    const size_t dim = si->getControlSpace()->getDimension();
    out << "      - [";
    for (size_t d = 0; d < dim; ++d)
    {
      const auto action = path->getControl(i);
      double *address = si->getControlSpace()->getValueAddressAtIndex(action, d);
      out << *address;
      if (d < dim - 1)
      {
        out << ",";
      }
    }
    out << "]" << std::endl;
  }

  return 0;
}
