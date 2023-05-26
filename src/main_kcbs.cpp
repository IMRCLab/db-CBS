/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/planners/rrt/RRT.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <unordered_map>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"


namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;


/* 
When performing Multi-Robot Motion Planning, it is sometimes the case that 
robots are treated as "dynamic obstacles" by other robots (e.g. Prioritized Planning and Kinodynamic Conflict-Based Search). 
Thus, we have extended OMPL to account for dynamic obstacles. To do so, one must implement an additional method called 
``areStatesValid" from the StateValidityChecker class. The method should return true if state1 and state2 are not in collision.
state2 is a pair consisting of the actual state and the SpaceInformation from which the state lives. The SpaceInformation object is included 
so that heterogeneous robots can be properly accounted for (see the example below). Keep in mind that 
time-dependence is handled generically within OMPL. Please see ob::StateValidityChecker::isValid(const State *state, const double time) 
for details.
*/

// K-CBS and PP both work for multiple type of low-level planners. 
// Providing this function to the multi-agent space information will let you use any of them
ompl::base::PlannerPtr PlannerAllocator(const ompl::base::SpaceInformationPtr &si)
{
    const oc::SpaceInformationPtr siC = std::static_pointer_cast<ompl::control::SpaceInformation>(si);
    ompl::base::PlannerPtr planner = std::make_shared<oc::RRT>(siC);
    return planner;
}


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
    ("planner,p", po::value<std::string>(&plannerDesc)->default_value("k-cbs"), "Planner")
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
  std::vector<double> state_lengths;
  std::vector<double>::iterator its, itd;
  std::vector<double> startTemp;
  std::vector<double> goalTemp;
  // std::vector<std::vector<double>> robot_states;

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
  std::shared_ptr<Robot> joint_robots = create_joint_robot(robots);
  // load config file
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  std::vector<double>::iterator switchItr;
  auto ma_si(std::make_shared<omrc::SpaceInformation>());
  auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));
  int i = 0;
  int j = 0;
  for (auto robot : robots){
    auto si = robot->getSpaceInformation();
    // set number of control steps (use 0.1s as increment -> 0.1 to 1s per Steer function)
    si->setPropagationStepSize(cfg["k-cbs"]["default"]["propagation_step_size"].as<double>());
    si->setMinMaxControlDuration(
      cfg["k-cbs"]["default"]["control_duration"][0].as<int>(),
      cfg["k-cbs"]["default"]["control_duration"][1].as<int>());

    // set state validity checking for this space
    auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, bpcm_env, robot));
    si->setStateValidityChecker(stateValidityChecker);
    // set the state propagator
    std::shared_ptr<oc::StatePropagator> statePropagator(new RobotStatePropagator(si, robot));
    si->setStatePropagator(statePropagator);
    si->setup();
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
    its = start_reals.begin()+i;
    itd = goal_reals.begin()+i; 
    for (;  its != start_reals.begin() + state_lengths[j]; ++its, ++itd){
        startTemp.push_back(*its);
        goalTemp.push_back(*itd);
    }
    i += state_lengths[j];
    j++;
    // create and set a start state
    auto startState = si->allocState();
    si->getStateSpace()->copyFromReals(startState, startTemp);
    si->enforceBounds(startState);
    pdef->addStartState(startState);
    si->freeState(startState);
    startTemp.clear();

    // set goal state
    auto goalState = si->allocState();
    si->getStateSpace()->copyFromReals(goalState, goalTemp);
    si->enforceBounds(goalState);
    pdef->setGoalState(goalState, cfg["k-cbs"]["default"]["goal_epsilon"].as<double>());
    si->freeState(goalState);
    goalTemp.clear();
    
    ma_si->addIndividual(si);
    ma_pdef->addIndividual(pdef);
  }
  // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
  ma_si->lock();
  ma_pdef->lock();

  ompl::base::PlannerAllocator allocator = PlannerAllocator;
  ma_si->setPlannerAllocator(allocator);
  std::ofstream stats(statsFile);
  stats << "stats:" << std::endl;
  auto start = std::chrono::steady_clock::now();
  if (plannerDesc == "k-cbs")
  {
      // plan using Kinodynamic Conflict Based Search
      auto planner = std::make_shared<omrc::KCBS>(ma_si);
      planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition

      planner->setLowLevelSolveTime(0.5);

      bool solved = planner->as<omrb::Planner>()->solve(30.0);
      if (solved)
      {
          auto now = std::chrono::steady_clock::now();
          double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
          stats << "  - t: " << t/1000.0f << std::endl;
          std::cout << "Found solution!" << std::endl;
          omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
          std::ofstream MyFile(outputFile);
          MyFile << "result:" << std::endl;
          solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "- states:");
          // std::ofstream MyFile2("tree.txt");
          // planner->printConstraintTree(MyFile2);
      }
  }

  return 0;
}

