#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// DYNOPLAN
#include <dynoplan/optimization/ocp.hpp>
#include "dynoplan/optimization/multirobot_optimization.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "planresult.hpp"
#include "dbcbs_utils.hpp"

using namespace dynoplan;

#define DYNOBENCH_BASE "../dynoplan/dynobench/"

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string optimizationFile;
    std::string cfgFile;

    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("optimization,opt", po::value<std::string>(&optimizationFile)->required(), "optimization file (yaml)")
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
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    cfg = cfg["db-cbs"]["default"];
    // tdbstar options
    Options_tdbastar options_tdbastar;
    options_tdbastar.outFile = outputFile;
    // options_tdbastar.max_motions = 50;
    options_tdbastar.search_timelimit = 10000000;
    options_tdbastar.cost_delta_factor = 0;
    options_tdbastar.delta = cfg["delta_0"].as<float>();
    options_tdbastar.fix_seed = 1;
    options_tdbastar.max_motions = 500;
    // tdbastar problem
    dynobench::Problem problem(inputFile);
    std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
    problem.models_base_path = models_base_path;
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
        if (obs["type"].as<std::string>() == "box"){
            const auto &size = obs["size"];
            std::shared_ptr<fcl::CollisionGeometryf> geom;
            geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), 1.0));
            const auto &center = obs["center"];
            auto co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), 0));
            co->computeAABB();
            obstacles.push_back(co);
        }
        else {
        throw std::runtime_error("Unknown obstacle type!");
        }
    }
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
    std::vector<dynobench::Trajectory> ll_trajs;
    std::string motionsFile;
    std::vector<std::string> all_motionsFile;
    for (const auto &robotType : problem.robotTypes){
        std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
                (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
        robots.push_back(robot);
        if (robotType == "unicycle1_v0" || robotType == "unicycle1_sphere_v0"){
            motionsFile = "../dynoplan/data/motion_primitives/unicycle1_v0/unicycle1_v0.bin.less.bin";
        } else if (robotType == "unicycle2_v0"){
            motionsFile = "../dynoplan/data/motion_primitives/unicycle2_v0/unicycle2_v0.bin.im.bin.im.bin.msgpack";
        } else if (robotType == "car1_v0"){
            motionsFile = "../dynoplan/data/motion_primitives/car_with_trailers/car_with_trailers.bin.sp.bin.msgpack";
        } else if (robotType == "integrator2_2d_v0"){
            motionsFile = "../dynoplan/data/motion_primitives/integrator2_2d_v0/integrator2_2d_v0.bin.im.bin.sp.bin";
        } else{
            throw std::runtime_error("Unknown motion filename for this robottype!");
        }
        all_motionsFile.push_back(motionsFile);
    }
    // Initialize the root node
    bool solved_db = false;
    HighLevelNode start;
    start.solution.resize(env["robots"].size());
    start.constraints.resize(env["robots"].size());
    start.cost = 0;
    start.id = 0;
    // read motions, run tbastar
    create_dir_if_necessary(outputFile);
    std::ofstream out(outputFile);
    std::map<std::string, std::vector<Motion>> robot_motions;
    size_t robot_id = 0;
    // Get the root node solutions
    for (const auto &robot : robots){
        // collision_geometries.insert(collision_geometries.end(),
                                // robot->collision_geometries.begin(),
                                // robot->collision_geometries.end());
        collision_geometries.insert(collision_geometries.end(),
                                robot->collision_geometries.back());

        if (robot_motions.find(problem.robotTypes[robot_id]) == robot_motions.end()){
            options_tdbastar.motionsFile = all_motionsFile[robot_id];
            load_motion_primitives_new(options_tdbastar.motionsFile, *robot, robot_motions[problem.robotTypes[robot_id]], 
                                      options_tdbastar.max_motions,
                                      options_tdbastar.cut_actions, false, options_tdbastar.check_cols);
            options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[robot_id]]; 
        }
        
        tdbastar(problem, options_tdbastar, start.solution[robot_id].trajectory, start.constraints[robot_id],
                  out_tdb, robot_id);
        if(!out_tdb.solved){
          std::cout << "Couldn't find initial solution for robot " << robot_id << "." << std::endl;
          break;
        }
        start.cost += start.solution[robot_id].trajectory.cost;
        robot_id++;
    }
    // allocate data for conflict checking, check for conflicts
    std::vector<fcl::CollisionObjectd*> robot_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
    col_mng_robots->setup();
    for (size_t i = 0; i < collision_geometries.size(); i++){
          size_t userData = i;
          auto robot_obj = new fcl::CollisionObject(collision_geometries[i]);
          collision_geometries[i]->setUserData((void*)userData);
          robot_objs.push_back(robot_obj);
    }
    col_mng_robots->registerObjects(robot_objs);
    
    // OPEN set
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                        boost::heap::mutable_<true> > open;
    auto handle = open.push(start);
    (*handle).handle = handle;
    int id = 1;
    while (!open.empty()){
      HighLevelNode P = open.top();
      open.pop();
      Conflict inter_robot_conflict;
      if (!getEarliestConflict(P.solution, robots, col_mng_robots, robot_objs, inter_robot_conflict)){
          solved_db = true;
          std::cout << "Final solution!" << std::endl; 
          export_solutions(P.solution, robots.size(), &out);
          bool sum_robot_cost = true;
          bool feasible = execute_optimizationMultiRobot(inputFile,
                                        outputFile, 
                                        optimizationFile,
                                        DYNOBENCH_BASE,
                                        sum_robot_cost);
          if (feasible) {
            return 0;
          }
          break;
      }
      std::map<size_t, std::vector<Constraint>> constraints;
      createConstraintsFromConflicts(inter_robot_conflict, constraints);
      for (const auto& c : constraints){
        HighLevelNode newNode = P;
        size_t tmp_robot_id = c.first;
        newNode.id = id;
        std::cout << "Node ID is " << id << std::endl;
        newNode.constraints[tmp_robot_id].insert(newNode.constraints[tmp_robot_id].end(), c.second.begin(), c.second.end());
        newNode.cost -= newNode.solution[tmp_robot_id].trajectory.cost;
        std::cout << "New node cost: " << newNode.cost << std::endl;
        Out_info_tdb tmp_out_tdb; // should I keep the old one ?
        options_tdbastar.motions_ptr = &robot_motions[problem.robotTypes[tmp_robot_id]]; 
        tdbastar(problem, options_tdbastar, newNode.solution[tmp_robot_id].trajectory, newNode.constraints[tmp_robot_id], tmp_out_tdb, tmp_robot_id);
        if (tmp_out_tdb.solved){
            newNode.cost += newNode.solution[tmp_robot_id].trajectory.cost;
            std::cout << "Updated New node cost: " << newNode.cost << std::endl;
            auto handle = open.push(newNode);
            (*handle).handle = handle;
            id++;
        }
      }   
  } // end of while loop
  return 0;
}
