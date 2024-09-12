#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"

// #define DBG_PRINTS
#include "db_astar.hpp"
#include "planresult.hpp"

#include <dynoplan/optimization/ocp.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "nlopt.hpp"
// #include "multirobot_trajectory.hpp"
#include "dynoplan/optimization/multirobot_optimization.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Conflicts 
struct Conflict {
  float time;
  size_t robot_idx_i;
  ob::State* robot_state_i;
  size_t robot_idx_j;
  ob::State* robot_state_j;
};

// Constraints
struct Constraint {
//   Constraint(float time, ob::State* state) : time(time), constrained_state(state) {}
  float time;
  ob::State* constrained_state;
};

struct HighLevelNode {
    std::vector<LowLevelPlan<AStarNode*, ob::State*, oc::Control*>> solution;
    std::vector<std::vector<Constraint>> constraints;
    // std::map<size_t, std::vector<Constraint>> constraints;

    float cost; 
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
  };

void print_solution(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, const std::vector<std::shared_ptr<Robot>>& all_robots){
    size_t max_t = 0;
    ob::State *node_state;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.size() - 1);
    }
    for (size_t t = 0; t <= max_t; ++t){
        std::cout << "/////////// "<< "time: " << t << "///////////" << std::endl;
        for (size_t i = 0; i < all_robots.size(); ++i){
            std::cout << "robot " << i << std::endl;
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
            }
            const auto transform = all_robots[i]->getTransform(node_state,0);
            std::cout << transform.translation() << std::endl;
        }
    }
}
void export_solution_p0(const std::vector<Eigen::VectorXf> &p0_opt, std::string outputFile_payload) { 
    
    std::ofstream out(outputFile_payload);
    out << "payload:" << std::endl;
    for (const auto& p0 : p0_opt){ 
        out << "    - [";
        out << p0(0) << ", " << p0(1) << "]";      
        out << std::endl;
        }
}
// export path to .yaml file
void export_solutions(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, 
                        const std::vector<std::shared_ptr<Robot>>& robots, std::string outputFile, int & expansions){
    std::ofstream out(outputFile);
    std::vector<double> reals;
    float cost = 0;
    for (auto& n : solution)
      cost += n.cost;
    out << "expansions: " << expansions << std::endl; 
    out << "cost: " << cost << std::endl; 
    out << "result:" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
        auto si = robots[i]->getSpaceInformation(); 
        out << "  - states:" << std::endl;
        for (size_t j = 0; j < solution[i].trajectory.size(); ++j){
            const auto node_state = solution[i].trajectory[j];
            out << "      - ";
            printState(out, si, node_state);
            out << std::endl;
        }
        out << "    actions:" << std::endl;
        for (size_t j = 0; j < solution[i].actions.size(); ++j){
            const auto& node_action = solution[i].actions[j];
            // out << "      # ";
            out << "      - ";
            printAction(out, si, node_action);
            out << std::endl;
        }
    }
}


inline Eigen::VectorXf create_vector(const std::vector<double> &v) {
//   Eigen::VectorXf out(v.size());
//   for (size_t i = 0; i < v.size(); ++i) {
//     out(i) = v[i];
//   }
//   return out;
    std::vector<float> vf(v.begin(), v.end());
    return Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>((vf.data()), vf.size());
}

inline std::vector<double> eigentoStd(const Eigen::VectorXf &eigenvec)
{
    // std::vector<double> stdvec;
    // for (const auto& i : eigenvec)
    // {
    //     stdvec.push_back(i);
    // }
    Eigen::VectorXd eigenVecD = eigenvec.cast<double>();
    std::vector<double> stdvec(&eigenVecD[0], eigenVecD.data()+eigenVecD.cols()*eigenVecD.rows());
    return stdvec;
}


// inline Eigen::VectorXf stdtoEigen(const std::vector<double>& stdvec) {
//     Eigen::VectorXf eigenvec(stdvec.size());
//     for (size_t i = 0; i < stdvec.size(); ++i)
//     {
//         eigenvec(i) = stdvec[i];
//     }
//     // Eigen::VectorXd eigenvec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(stdvec.data(), stdvec.size());
//     return eigenvec;
// }


typedef struct {
    std::vector<Eigen::VectorXf> pi;   // positions of the robots 
    std::vector<double> l; // cable lengths
    double mu;  // regularization weight
    Eigen::VectorXf p0_d; // Desired payload position (likely to be the previous solution)
} cost_data;


double cost(const std::vector<double> &p0, std::vector<double> &/*grad*/, void *data) {
    
    cost_data *d = (cost_data *) data; 
    const auto& pi = d -> pi;
    const auto& p0_d = d -> p0_d;
    const auto& l = d -> l;
    const auto& mu = d -> mu;

    double cost = 0;
    double dist  = 0;
    Eigen::VectorXf p0_eigen = create_vector(p0);
    int i = 0;
    for(const auto& p : pi) {
        double dist = (p0_eigen - p).norm() - l[i];
        cost += dist*dist;
        ++i;
    }
    cost += mu*(p0_d - p0_eigen).norm();
    return cost;
}

Eigen::VectorXf optimizePayload(Eigen::VectorXf &p0_opt,
                                       size_t dim, 
                                       const Eigen::VectorXf &p0_guess, // initial guess
                                       cost_data &data
                                ) {

    // create the optimization problem
    nlopt::opt opt(nlopt::LN_COBYLA, dim);
    std::vector<double> p0_vec = eigentoStd(p0_guess);
    // set the initial guess
    opt.set_min_objective(cost, &data);
    opt.set_xtol_rel(1e-4); // Tolerance

    double minf; // Variable to store the minimum value found
    try {
        nlopt::result result = opt.optimize(p0_vec, minf);
        // std::cout << "Found minimum at f(" << p0_vec[0] << ", " << p0_vec[1] << ") = "
                //   << minf << std::endl;
        p0_opt = create_vector(p0_vec);
        return p0_opt;
    } catch (std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

}

bool getEarliestConflict(
    const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution,
    const std::vector<std::shared_ptr<Robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots,
    const std::vector<fcl::CollisionObjectf*>& col_mng_objs,
    Conflict& early_conflict,
    std::vector<Eigen::VectorXf>& p0_sol)
{
    size_t max_t = 0;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.size() - 1);
    }

    ob::State* node_state;
    std::vector<ob::State*> node_states;
    Eigen::VectorXf p0_opt;
    std::vector<Eigen::VectorXf> p0_tmp;
    bool solve_p0 = true;
    Eigen::VectorXf p0_init_guess = create_vector({-1.0, 0.0});
    for (size_t t = 0; t <= max_t; ++t) {
        // std::cout << "TIMESTAMP: " << t << std::endl;
        node_states.clear();
        size_t obj_idx = 0;
        for (size_t i = 0; i < all_robots.size(); ++i){
            // std::cout << "ROBOT " << i << std::endl;
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
            }
            node_states.push_back(node_state);
            for (size_t p = 0; p < all_robots[i]->numParts(); ++p) {
                const auto transform = all_robots[i]->getTransform(node_state,p);
                col_mng_objs[obj_idx]->setTranslation(transform.translation());
                col_mng_objs[obj_idx]->setRotation(transform.rotation());
                col_mng_objs[obj_idx]->computeAABB();
                ++obj_idx;
            }
        }
        col_mng_robots->update(col_mng_objs);
        fcl::DefaultCollisionData<float> collision_data;
        col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
        if (collision_data.result.isCollision()) {
            assert(collision_data.result.numContacts() > 0);
            const auto& contact = collision_data.result.getContact(0);

            early_conflict.time = t * all_robots[0]->dt();
            early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
            early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
            assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
            early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
            early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];

#ifdef DBG_PRINTS
            std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;
            auto si_i = all_robots[early_conflict.robot_idx_i]->getSpaceInformation();
            si_i->printState(early_conflict.robot_state_i);
            auto si_j = all_robots[early_conflict.robot_idx_j]->getSpaceInformation();
            si_j->printState(early_conflict.robot_state_j);
#endif
            return true;
        }
    
        if (solve_p0) {
            Eigen::Vector3f robot1_pos = col_mng_objs[0]->getTranslation();
            Eigen::Vector3f robot2_pos = col_mng_objs[1]->getTranslation();
            float l = 0.5;
            
            size_t dim = 2;
            std::vector<double> li = {0.5, 0.5};
            double mu = 0.18;

            std::vector<Eigen::VectorXf> pi = {
                create_vector({robot1_pos(0), robot1_pos(1)}),
                create_vector({robot2_pos(0), robot2_pos(1)})
            };        
            cost_data data {pi, li, mu, p0_init_guess}; // prepare the data for the opt
            optimizePayload(p0_opt, dim, p0_init_guess, data);
            p0_init_guess << p0_opt(0), p0_opt(1);

            for (const auto& p : pi) {
                float distance = (p - p0_opt).norm();
                if (abs(distance - l) >= 0.1) { // assumed 2 robots
                    early_conflict.time = t * all_robots[0]->dt();
                    early_conflict.robot_idx_i = 0; 
                    early_conflict.robot_idx_j = 1;
                    assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
                    early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                    early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
                    return true;
                }
            }
        }  
    if (solve_p0) {
        p0_tmp.push_back(p0_opt);
    }
    }
    if (solve_p0) {
        for (const auto& p0i : p0_tmp) {
            p0_sol.push_back(p0i);
        }
    }


    return false;
}

// Constraints from Conflicts
void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<Constraint>>& constraints){
    constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
    constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
}

void export_joint_solutions(const std::vector<LowLevelPlan<AStarNode*,ob::State*, oc::Control*>>& solution, 
                        const std::vector<std::shared_ptr<Robot>>& robots, std::string outputFile){
    std::ofstream out(outputFile);
    std::vector<double> reals;
    ob::State *node_state;
    oc::Control *node_action;
    float cost = 0;
    size_t max_t = 0;
    size_t max_a = 0;
    for (auto& sol : solution){
      cost += sol.cost;
      max_t = std::max(max_t, sol.trajectory.size());
      max_a = std::max(max_a, sol.actions.size());
    }

    out << "cost: " << cost << std::endl; 
    out << "result:" << std::endl;
    out << "  - states:" << std::endl;
    std::vector<double> joint_state;
    std::vector<double> joint_action;
    std::vector<double> last_state;
    for (size_t t = 0; t < max_t; ++t){
        out << "      - [";
        for (size_t i = 0; i < robots.size(); ++i){
            std::vector<double> reals;
            auto si = robots[i]->getSpaceInformation(); 
            if (t >= solution[i].trajectory.size()){
                node_state = solution[i].trajectory.back();    
            }
            else {
                node_state = solution[i].trajectory[t];
            }
            si->getStateSpace()->copyToReals(reals, node_state);
            joint_state.insert(joint_state.end(), reals.begin(), reals.end());
        }
        for (size_t k = 0; k < joint_state.size(); ++k) {
                out << joint_state[k];
                if (k < joint_state.size() - 1) {
                    out << ",";
                }
        }
        out << "]" << std::endl;
        joint_state.clear();

    }

    // for the action
    out << "    actions:" << std::endl;
    for (size_t t = 0; t < max_a; ++t){
        out << "      - ";
        out << "[";
        for (size_t i = 0; i < robots.size(); ++i){
            std::vector<double> reals;
            auto si = robots[i]->getSpaceInformation(); 
            const size_t dim = si->getControlSpace()->getDimension();
            if (t >= solution[i].actions.size()){
                if (solution[i].actions.size() > 0) {
                    node_action = solution[i].actions.back();
                } else {
                    node_action = nullptr;
                }
            }
            else {
                node_action = solution[i].actions[t];
            }
            for (size_t d = 0; d < dim; ++d)
            {
                if (node_action) {
                    double *address = si->getControlSpace()->getValueAddressAtIndex(node_action, d);
                    reals.push_back(*address);
                } else {
                    reals.push_back(0);
                }
            }
            joint_action.insert(joint_action.end(), reals.begin(), reals.end());
        }
        for (size_t k = 0; k < joint_action.size(); ++k) {
                out << joint_action[k];
                if (k < joint_action.size() - 1) {
                    out << ",";
                }
        }
        out << "]" << std::endl;
        joint_action.clear();
    }

    
}

#define dynobench_base "../dynoplan/dynobench/"

int main(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string jointFile;
    std::string optimizationFile;
    std::string cfgFile;

    // std::string outputFileSimple;
    desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
      ("joint,jnt", po::value<std::string>(&jointFile)->required(), "joint output file (yaml)")
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

    // load config file
    YAML::Node cfg = YAML::LoadFile(cfgFile);
    // cfg = cfg["default"];
    float alpha = cfg["alpha"].as<float>();
    bool filter_duplicates = cfg["filter_duplicates"].as<bool>();

    // load problem description
    YAML::Node env = YAML::LoadFile(inputFile);
    std::vector<fcl::CollisionObjectf *> obstacles;
    std::vector<std::vector<fcl::Vector3f>> positions;
    std::vector<Eigen::VectorXf> p0_sol;
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

    // std::vector<Agent> agents;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<std::string> robot_types;

    std::map<std::string, Motions> robot_motions;


    std::vector<std::shared_ptr<Robot>> robots;
    for (const auto &robot_node : env["robots"]) {
        auto robotType = robot_node["type"].as<std::string>();
        std::shared_ptr<Robot> robot = create_robot(robotType, position_bounds);
        robots.push_back(robot);

        std::vector<double> start_reals;
        for (const auto& v : robot_node["start"]) {
            start_reals.push_back(v.as<double>());
        }
        starts.push_back(start_reals);

        std::vector<double> goal_reals;
        for (const auto& v : robot_node["goal"]) {
            goal_reals.push_back(v.as<double>());
        }
        goals.push_back(goal_reals);
        robot_types.push_back(robotType);

        auto iter = robot_motions.find(robotType);
        if (iter == robot_motions.end()) {
            std::string motionsFile;
            if (robotType == "unicycle_first_order_0" || robotType == "unicycle_first_order_0_sphere") {
                motionsFile = "../motions/unicycle_first_order_0_sorted.msgpack";
             } else if (robotType == "unicycle_second_order_0") {
                motionsFile = "../motions/unicycle_second_order_0_sorted.msgpack";
            } else if (robotType == "double_integrator_0") {
                motionsFile = "../motions/double_integrator_0_sorted.msgpack";
            } else if (robotType == "car_first_order_with_1_trailers_0") {
                motionsFile = "../motions/car_first_order_with_1_trailers_0_sorted.msgpack";
            } else {
                throw std::runtime_error("Unknown motion filename for this robottype!");
            }

            // load the msgpck
            std::ifstream is( motionsFile.c_str(), std::ios::in | std::ios::binary );
            // get length of file
            is.seekg (0, is.end);
            int length = is.tellg();
            is.seekg (0, is.beg);
            //
            msgpack::unpacker unpacker;
            unpacker.reserve_buffer(length);
            is.read(unpacker.buffer(), length);
            unpacker.buffer_consumed(length);
            msgpack::object_handle oh;
            unpacker.next(oh);
            load_motions(oh.get(), robot, robotType, env_min.size(), /*delta, filterDuplicates, alpha,*/ robot_motions[robotType]);

            std::cout << "loaded motions for " << robotType << std::endl;
        }
    }

    // Heuristic computation
    std::vector<ompl::NearestNeighbors<AStarNode*>*> heuristics(robots.size(), nullptr);

    if (cfg["heuristic1"].as<std::string>() == "reverse-search") {
        // disable/enable motions
        for (auto& iter : robot_motions) {
            for (size_t i = 0; i < robot_types.size(); ++i) {
                if (iter.first == robot_types[i]) {
                    disable_motions(robots[i], cfg["heuristic1_delta"].as<float>(), filter_duplicates, alpha, 99999, iter.second);
                    break;
                }
            }
        }

        DBAstar<Constraint> llplanner(cfg["heuristic1_delta"].as<float>(), alpha);
        for (size_t i = 0; i < robots.size(); ++i) {

            LowLevelPlan<AStarNode*,ob::State*,oc::Control*> ll_result;
            std::vector<double> v_nanf(starts[i].size(), nanf(""));
            llplanner.search(robot_motions.at(robot_types[i]), v_nanf, goals[i], 
                obstacles, workspace_aabb, robots[i], {}, /*reverse_search*/true, ll_result, nullptr, &heuristics[i]);
            std::cout << "computed heuristic with " << heuristics[i]->size() << " entries." << std::endl;
        }
    }

    // allocate data for conflict checking
    std::vector<fcl::CollisionObjectf*> col_mng_objs;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mng_robots;
    col_mng_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    col_mng_robots->setup();

    for (size_t i = 0; i < robots.size(); ++i) {
        for (size_t p = 0; p < robots[i]->numParts(); ++p) {
            auto coll_obj = new fcl::CollisionObjectf(robots[i]->getCollisionGeometry(p));
            size_t userData = i;
            robots[i]->getCollisionGeometry(p)->setUserData((void*)userData);
            col_mng_objs.push_back(coll_obj);
        }
    }
    col_mng_robots->registerObjects(col_mng_objs);

    // actual search

    float delta = cfg["delta_0"].as<float>();
    size_t max_motions = cfg["num_primitives_0"].as<size_t>();
    bool solved_db = false;

    for (size_t iteration = 0; ; ++iteration) {

        if (iteration > 0) {
            if (solved_db) {
                delta *= cfg["delta_rate"].as<float>();
            } else {
                delta *= 0.99;
            }
            max_motions *= cfg["num_primitives_rate"].as<float>();
            max_motions = std::min<size_t>(max_motions, 1e6);
        }

        std::cout << "Search with delta=" << delta << " and motions=" << max_motions << std::endl;

        // disable/enable motions
        for (auto& iter : robot_motions) {
            for (size_t i = 0; i < robot_types.size(); ++i) {
                if (iter.first == robot_types[i]) {
                    disable_motions(robots[i], delta, filter_duplicates, alpha, max_motions, iter.second);
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
        int i = 0;
        bool start_node_valid = true;
        for (const auto &robot_node : env["robots"]) {
            DBAstar<Constraint> llplanner(delta, alpha);
            bool success = llplanner.search(robot_motions.at(robot_types[i]), starts[i], goals[i], 
                obstacles, workspace_aabb, robots[i], start.constraints[i], /*reverse_search*/false, start.solution[i], heuristics[i]);
            if (!success) {
                std::cout << "Couldn't find initial solution for robot " << i << "." << std::endl;
                start_node_valid = false;
                break;
            }

            start.cost += start.solution[i].cost;
            std::cout << "High Level Node Cost: " << start.cost << std::endl;
            i++;
        } 
        if (!start_node_valid) {
            continue;
        }
        
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                        boost::heap::mutable_<true> > open;
        auto handle = open.push(start);
        (*handle).handle = handle;
        int id = 1;

        size_t expands = 0;
        while (!open.empty()) {
            HighLevelNode P = open.top();
            open.pop();
            Conflict inter_robot_conflict;
            if (!getEarliestConflict(P.solution, robots, col_mng_robots, col_mng_objs, inter_robot_conflict, p0_sol)) {
                solved_db = true;
                std::cout << "Final solution! cost: " << P.cost << std::endl;
                std::ofstream out(outputFile);
                out << "expansions: " << id << std::endl;
                export_solutions(P.solution, robots, outputFile, id);
                size_t pos = outputFile.rfind(".yaml");
                std::string outputFile_payload = "../result_dbcbs_payload.yaml";
                // Check if ".yaml" is found at the end of the string
                if (pos != std::string::npos) {
                    // Remove ".yaml" and add "_payload.yaml"
                    outputFile_payload = outputFile.substr(0, pos) + "_payload.yaml";
                    std::cout << "outputFile_payload: " << outputFile_payload << std::endl;
                }
                export_solution_p0(p0_sol, outputFile_payload);
                export_joint_solutions(P.solution, robots, jointFile);
            
                std::cout << "warning: using new multirobot optimization" << std::endl;
            
                const bool sum_robot_cost = true;
                bool run_opt  = false;
                if (run_opt) { 
                    // switch off the optimization                   
                    bool feasible = execute_optimizationMultiRobot(inputFile,
                                                        outputFile, 
                                                        optimizationFile,
                                                        dynobench_base,
                                                        sum_robot_cost);
                    if (feasible) {
                        return 0;
                    }
                }
                return 0;
                break;
            }

            ++expands;
            if (expands % 100 == 0) {
                std::cout << "HL expanded: " << expands << " open: " << open.size() << " cost " << P.cost << " conflict at " << inter_robot_conflict.time << std::endl;
            }
        
            std::map<size_t, std::vector<Constraint>> constraints;
            createConstraintsFromConflicts(inter_robot_conflict, constraints);
            for (const auto& c : constraints){
                HighLevelNode newNode = P;
                size_t i = c.first;
                newNode.id = id;
#ifdef DBG_PRINTS
                std::cout << "Node ID is " << id << std::endl;
#endif
                newNode.constraints[i].insert(newNode.constraints[i].end(), c.second.begin(), c.second.end());
                newNode.cost -= newNode.solution[i].cost;
#ifdef DBG_PRINTS
                std::cout << "New node cost: " << newNode.cost << std::endl;
#endif

                // run the low level planner
                DBAstar<Constraint> llplanner(delta, alpha);
                bool success = llplanner.search(robot_motions.at(robot_types[i]), starts[i], goals[i], 
            		obstacles, workspace_aabb, robots[i], newNode.constraints[i], /*reverse_search*/false, newNode.solution[i], heuristics[i]); 

                if (success) {
                    newNode.cost += newNode.solution[i].cost;
#ifdef DBG_PRINTS
                    std::cout << "Updated New node cost: " << newNode.cost << std::endl;
#endif
                    //   print_solution(newNode.solution, robots);

                    auto handle = open.push(newNode);
                    (*handle).handle = handle;
                    
                    id++;
                }
            }
        }
        std::cout << "HL expansions id: " << id << std::endl;
    }
    return 0;
}
