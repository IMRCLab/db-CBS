#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include "fclStateValidityChecker.hpp"
#include "robotStatePropagator.hpp"
#include "robots.h"

#include "db_astar.hpp"
#include "planresult.hpp"

#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
#include <idbastar/optimization/ocp.hpp>
#include "multirobot_trajectory.hpp"





int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile;
  std::string initFile;
  std::string outFile;
  bool sum_robots_cost = true;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      "init,i", po::value<std::string>(&initFile)->required())(
      "out,o", po::value<std::string>(&outFile)->required())(
      "sum,s", po::value<bool>(&sum_robots_cost)->required());

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  execute_optimizationQ(envFile, initFile, outFile, sum_robots_cost);
}
