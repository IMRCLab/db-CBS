#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include "planresult.hpp"
#include "nlopt.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"


void export_solution_p0(const std::vector<Eigen::VectorXf> &p0_opt, std::string outputFile_payload) { 
    
    std::ofstream out(outputFile_payload);
    out << "payload:" << std::endl;
    for (const auto& p0 : p0_opt){ 
        out << "    - [";
        out << p0(0) << ", " << p0(1) << "]";      
        out << std::endl;
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



// Conflicts 
struct Conflict {
  double time;
  size_t robot_idx_i;
  Eigen::VectorXd robot_state_i;
  size_t robot_idx_j;
  Eigen::VectorXd robot_state_j;
};
// Constraints
struct HighLevelNode {
    std::vector<LowLevelPlan<dynobench::Trajectory>> solution;
    std::vector<std::vector<dynoplan::Constraint>> constraints;
    float cost; 
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;
    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }
};

bool getEarliestConflict(
    const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>>& all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd*>& robot_objs,
    Conflict& early_conflict,
    std::vector<Eigen::VectorXf>& p0_sol){
    size_t max_t = 0;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.states.size() - 1);
    }
    Eigen::VectorXd node_state;
    std::vector<Eigen::VectorXd> node_states;
    Eigen::VectorXf p0_opt;
    std::vector<Eigen::VectorXf> p0_tmp;
    bool solve_p0 = true;
    Eigen::VectorXf p0_init_guess = create_vector({-1.0, 0.0});

    for (size_t t = 0; t <= max_t; ++t){
        node_states.clear();
        size_t robot_idx = 0;
        size_t obj_idx = 0;
        std::vector<fcl::Transform3d> ts_data;
        for (auto &robot : all_robots){
          if (t >= solution[robot_idx].trajectory.states.size()){
              node_state = solution[robot_idx].trajectory.states.back();    
          }
          else {
              node_state = solution[robot_idx].trajectory.states[t];
          }
          node_states.push_back(node_state);
          std::vector<fcl::Transform3d> tmp_ts(1);
          if (robot->name == "car_with_trailers") {
            tmp_ts.resize(2);
          }
          robot->transformation_collision_geometries(node_state, tmp_ts);
          ts_data.insert(ts_data.end(), tmp_ts.begin(), tmp_ts.end());
          // ts_data.insert(ts_data.end(), tmp_ts.back()); // just trailer
          ++robot_idx;
        }
        for (size_t i = 0; i < ts_data.size(); i++) {
          fcl::Transform3d &transform = ts_data[i];
          robot_objs[obj_idx]->setTranslation(transform.translation());
          robot_objs[obj_idx]->setRotation(transform.rotation());
          robot_objs[obj_idx]->computeAABB();
          ++obj_idx;
        }
        col_mng_robots->update(robot_objs);
        fcl::DefaultCollisionData<double> collision_data;
        col_mng_robots->collide(&collision_data, fcl::DefaultCollisionFunction<double>);
        if (collision_data.result.isCollision()) {
            assert(collision_data.result.numContacts() > 0);
            const auto& contact = collision_data.result.getContact(0);

            early_conflict.time = t * all_robots[0]->ref_dt;
            early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
            early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
            assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
            early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
            early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
            std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;

// #ifdef DBG_PRINTS
//             std::cout << "CONFLICT at time " << t << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;
//             auto si_i = all_robots[early_conflict.robot_idx_i]->getSpaceInformation();
//             si_i->printState(early_conflict.robot_state_i);
//             auto si_j = all_robots[early_conflict.robot_idx_j]->getSpaceInformation();
//             si_j->printState(early_conflict.robot_state_j);
// #endif
            return true;
        } 
        if (solve_p0) {
            Eigen::Vector3f robot1_pos = robot_objs[0]->getTranslation().cast<float>();
            Eigen::Vector3f robot2_pos = robot_objs[1]->getTranslation().cast<float>();
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
                if (abs(distance - l) >= 0.2) { // assumed 2 robots
                    early_conflict.time = t * all_robots[0]->ref_dt;
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

void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<dynoplan::Constraint>>& constraints){
    constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
    constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
}

void export_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, 
                        const int robot_numx, std::ofstream *out){
    float cost = 0;
    for (auto& n : solution)
      cost += n.trajectory.cost;
    *out << "cost: " << cost << std::endl; 
    *out << "result:" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i){ 
        std::vector<Eigen::VectorXd> tmp_states = solution[i].trajectory.states;
        std::vector<Eigen::VectorXd> tmp_actions = solution[i].trajectory.actions;
        *out << "  - states:" << std::endl;
        for (size_t j = 0; j < tmp_states.size(); ++j){
            *out << "      - ";
            *out << tmp_states.at(j).format(dynobench::FMT)<< std::endl;
        }
        *out << "    actions:" << std::endl;
        for (size_t j = 0; j < tmp_actions.size(); ++j){
            *out << "      - ";
            *out << tmp_actions.at(j).format(dynobench::FMT)<< std::endl;
            
        }
    }
}
