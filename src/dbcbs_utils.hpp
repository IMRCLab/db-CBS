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


void add_motion_primitives(dynobench::Problem &problem, 
                           dynobench::Trajectory &trajectory, 
                           std::map<std::string, std::vector<dynoplan::Motion>> &robot_motions,
                           const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots, double &sum_cost) {
    std::random_device rd;
    std::mt19937 gen(rd());

    if (startsWith(all_robots[0]->name, "quad3d")) {
        std::cout << "Computing primitives for quadrotors..." << std::endl;
        std::uniform_int_distribution<> random_length(20, 30); 
        int min_length = 10;

        size_t num_robots = all_robots.size();
        double l = 0.5; // Assume fixed cable length

        for (size_t robot_id = 0; robot_id < num_robots; ++robot_id) {
            size_t current_idx = 0;
            Eigen::Vector3d init_state = Eigen::Vector3d::Zero();
            
            while (current_idx < trajectory.states.size()) {    
                int len = random_length(gen);
                len = std::min(len, static_cast<int>(trajectory.states.size() - current_idx));
                if (trajectory.states.size() - current_idx <= min_length) {
                    len = static_cast<int>(trajectory.states.size() - current_idx);
                }

                std::vector<Eigen::VectorXd> uavStates;
                std::vector<Eigen::VectorXd> uavActions;

                for (size_t i = current_idx; i < current_idx + len; ++i) {
                    const Eigen::VectorXd &state = trajectory.states[i];
                    if (i == 0) {
                        init_state = state.segment<3>(0); 
                    }

                    // Normalize payload position
                    Eigen::Vector3d payloadPos = state.segment<3>(0) - init_state;
                    Eigen::Vector3d payloadVel = state.segment<3>(3);

                    // state: p0 v0 qc1 wc1 qc2 wc2,...,qcn,wcn, quat1, w1, quat2, w2, ..., quatn, wn
                    // state_size 6 + 6 + 6 *num_robots + 7*num_robots 
                    Eigen::Vector3d cableUnitVec = state.segment<3>(6 + 6 * robot_id);
                    Eigen::Vector3d cableAngularVel = state.segment<3>(6 + 6 * robot_id + 3);
                    Eigen::Vector4d uavQuat = state.segment<4>(6 + 6 * num_robots + 7 * robot_id);
                    Eigen::Vector3d uavAngularVel = state.segment<3>(6 + 6 * num_robots + 7 * robot_id + 4);

                    // Compute UAV position and velocity
                    Eigen::Vector3d uavPos = payloadPos - l * cableUnitVec;
                    Eigen::Vector3d cableUnitVec_der = cableAngularVel.cross(cableUnitVec);
                    Eigen::Vector3d uavVel = payloadVel - l * cableUnitVec_der;

                    // UAV state: [position, quaternion, velocity, angular velocity]
                    Eigen::VectorXd uavState(13);
                    uavState << uavPos, uavQuat, uavVel, uavAngularVel;
                    uavStates.push_back(uavState);
                    
                    // Add action if not the last state
                    if (i < current_idx + len - 1) {
                        const Eigen::VectorXd &action = trajectory.actions[i];
                        uavActions.push_back(action.segment<4>(4 * robot_id));
                    }
                }

                // Create a new trajectory segment
                dynobench::Trajectory new_robot_traj;
                new_robot_traj.states = uavStates;
                new_robot_traj.actions = uavActions;
                std::cout << "cost of robot: " << robot_id <<", " << uavActions.size()*all_robots[0]->ref_dt << std::endl;
                sum_cost += uavActions.size()*all_robots[0]->ref_dt;
                for (size_t k = 0; k < num_robots; ++k) {
                    // Convert trajectory to motion and add to robot_motions
                    dynoplan::Motion new_motion;
                    traj_to_motion(new_robot_traj,  *all_robots[k], new_motion, /*check collision*/ true);
                    new_motion.traj = new_robot_traj;
                    new_motion.idx = robot_motions[problem.robotTypes[k]].size();
                    robot_motions[problem.robotTypes[k]].push_back(std::move(new_motion));
                }

                current_idx += len;
            }

        }
    } else if (startsWith(all_robots[0]->name, "unicycle")) {
        std::cout << "Computing primitives for unicycles..." << std::endl;
        std::uniform_int_distribution<> random_length(10, 15); 
        int min_length = 5;

        size_t num_robots = all_robots.size();
        for (size_t robot_id = 0; robot_id < num_robots; ++robot_id) {

            size_t current_idx = 0;
            double px_init = 0.0;
            double py_init = 0.0;

            while (current_idx < trajectory.states.size()) {    
                int len = random_length(gen);
                len = std::min(len, static_cast<int>(trajectory.states.size() - current_idx));
                if (trajectory.states.size() - current_idx <= min_length) {
                    len = static_cast<int>(trajectory.states.size() - current_idx);
                }
                std::vector<Eigen::VectorXd> unicycleStates;
                std::vector<Eigen::VectorXd> unicycleActions;

                for (size_t i = current_idx; i < current_idx + len; ++i) {
                    const Eigen::VectorXd &state = trajectory.states[i];
                    if (i == 0) {
                        px_init = state[0];
                        py_init = state[1];
                    }

                    double px = state[0] - px_init;
                    double py = state[1] - py_init;

                    for (size_t j = 0; j < robot_id; ++j) {
                        double theta = state[2 + num_robots + j];
                        px += 0.5 * cos(theta);
                        py += 0.5 * sin(theta);
                    }

                    double alpha = state[2 + robot_id];
                    Eigen::VectorXd unicycleState(3);
                    unicycleState << px, py, alpha;
                    unicycleStates.push_back(unicycleState);

                    if (i < current_idx + len - 1) {
                        const Eigen::VectorXd &action = trajectory.actions[i];
                        unicycleActions.push_back(action.segment<2>(2 * robot_id));
                    }
                }

                // Create a new trajectory segment
                dynobench::Trajectory new_robot_traj;
                new_robot_traj.states = unicycleStates;
                new_robot_traj.actions = unicycleActions;
                std::cout << "cost of robot: " << robot_id <<", " << unicycleActions.size()*all_robots[0]->ref_dt << std::endl;
                sum_cost += unicycleActions.size()*all_robots[0]->ref_dt;

                for (size_t k = 0; k < num_robots; ++k) {
                    // Convert trajectory to motion and add to robot_motions
                    dynoplan::Motion new_motion;
                    traj_to_motion(new_robot_traj,  *all_robots[k], new_motion, /*check collision*/ true);
                    new_motion.traj = new_robot_traj;
                    new_motion.idx = robot_motions[problem.robotTypes[k]].size();
                    robot_motions[problem.robotTypes[k]].push_back(std::move(new_motion));
                }
                // Move to the next segment
                current_idx += len;
            }
        }
    }
}



void export_solution_p0(const std::vector<Eigen::VectorXf> &p0_opt, std::string outputFile_payload) { 
    
    std::ofstream out(outputFile_payload);
    out << "payload:" << std::endl;
    for (const auto& p0 : p0_opt){ 
        out << "    - [";
        out << p0(0) << ", " << p0(1) << ", " << p0(2) << "]";      
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
    double lambda; // penalty term
    Eigen::VectorXf p0_d; // Desired payload position (likely to be the previous solution)
} cost_data;


double cost(const std::vector<double> &p0, std::vector<double> &/*grad*/, void *data) {
    
    cost_data *d = (cost_data *) data; 
    const auto& pi = d -> pi;
    const auto& p0_d = d -> p0_d;
    const auto& l = d -> l;
    const auto& mu = d -> mu;
    const auto& lambda = d -> lambda;

    double cost = 0;
    double dist  = 0;
    Eigen::VectorXf p0_eigen = create_vector(p0);
    int i = 0;
    double minZ = std::numeric_limits<double>::max();
    double l_min = 0;
    for(const auto& p : pi) {
        double dist = (p0_eigen - p).norm() - l[i];
        cost += dist*dist;
        if (p(2) < minZ) {
            minZ = p(2);
            l_min = l[i];
        }
        ++i;
    }
    cost += mu*(p0_d - p0_eigen).norm() + lambda*(minZ - p0_eigen(2) - l_min)*(minZ - p0_eigen(2) - l_min);
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

// Cable shapes for 
struct cableShapes {
        std::vector<fcl::CollisionObjectd*> cablesObj;
        std::vector<fcl::CollisionObjectd*> robotsObj;
        std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mgr_cables;
        // std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mgr_robots;
        std::vector<size_t> robot_indices;

    void addCableShapes(const size_t& num_robots, const std::vector<double>& l, std::string& robot_name) {
        col_mgr_cables = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        // col_mgr_robots = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        cablesObj.clear();
        robotsObj.clear();
        if (startsWith(robot_name, "quad3d")) {
            for (size_t i=0; i < num_robots; ++i) {
                std::shared_ptr<fcl::CollisionGeometryd> cablegeom;
                cablegeom.reset(new fcl::Capsuled(0.01, l[i]));
                cablegeom->setUserData((void*) i);
                auto cableco = new fcl::CollisionObject(cablegeom);
                cableco->computeAABB();
                cablesObj.push_back(cableco);
            }
        } else if (startsWith(robot_name, "unicycle")) {
            for (size_t i=0; i < num_robots-1; ++i) {
                std::shared_ptr<fcl::CollisionGeometryd> cablegeom;
                // cablegeom.reset(new fcl::Boxd(0.6*0.5,0.01, 0.01));
                cablegeom.reset(new fcl::Boxd(0.6*0.37,0.01, 0.01));
                cablegeom->setUserData((void*) i);
                auto cableco = new fcl::CollisionObject(cablegeom);
                cableco->computeAABB();
                cablesObj.push_back(cableco);


                // std::shared_ptr<fcl::CollisionGeometryd> robotgeom_0;               
                // robotgeom_0.reset(new fcl::Boxd(0.1, 0.05, 0.05));
                // robotgeom_0->setUserData((void*) i);
                // auto robot0co = new fcl::CollisionObject(robotgeom_0);
                // robot0co->computeAABB();
                // robotsObj.push_back(robot0co);
                // if (i == num_robots-2) {
                //     std::shared_ptr<fcl::CollisionGeometryd> robotgeom_1;
                //     robotgeom_1.reset(new fcl::Boxd(0.1, 0.05, 0.05));
                //     robotgeom_1->setUserData((void*) i+1);
                //     auto robot1co = new fcl::CollisionObject(robotgeom_1);
                //     robot1co->computeAABB();
                //     robotsObj.push_back(robot1co);
                // }

            }
        }
    
        // col_mgr_robots->registerObjects(robotsObj);
        // col_mgr_robots->setup();
        col_mgr_cables->registerObjects(cablesObj);
        col_mgr_cables->setup();


    }
};

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
    std::vector<double>& p0_init_guess_std,
    std::vector<Eigen::VectorXf>& p0_sol,
    const bool& solve_p0,
    const float& max_tol){
    size_t max_t = 0;
    for (const auto& sol : solution){
      max_t = std::max(max_t, sol.trajectory.states.size() - 1);
    }
    Eigen::VectorXd node_state;
    std::vector<Eigen::VectorXd> node_states;
    Eigen::VectorXf p0_opt;
    std::vector<Eigen::VectorXf> p0_tmp;
    cableShapes cables;

    std::vector<double> cable_lengths;
    for (size_t i = 0; i < all_robots.size();++i) {
        cable_lengths.push_back(0.5);
    }
    cables.addCableShapes(all_robots.size(), cable_lengths, all_robots[0]->name);
    Eigen::VectorXf p0_init_guess = create_vector(p0_init_guess_std);

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
            std::cout << "contact point 1 REGULAR CONFLICT: " << (size_t)contact.o1->getUserData() << std::endl;
            std::cout << "contact point 2 REGULAR CONFLICT: " << (size_t)contact.o2->getUserData() << std::endl;

            assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
            early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
            early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
            std::cout << "CONFLICT at time " << t*all_robots[0]->ref_dt << " " << early_conflict.robot_idx_i << " " << early_conflict.robot_idx_j << std::endl;

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
            if (startsWith(all_robots[0]->name, "quad3d")) { // Assuming Homogenous robot team
                size_t dim = 3;
                double mu = 0.18;
                double lambda = 1.;
                std::vector<Eigen::VectorXf> pi;
                std::vector<double> li;
                for (const auto& robot_obj : robot_objs) {
                    Eigen::Vector3f robot_pos = robot_obj->getTranslation().cast<float>();
                    pi.push_back(create_vector({robot_pos(0), robot_pos(1), robot_pos(2)}));
                    li.push_back(0.5); // TODO: this needs to be provided as an input
                }
                cost_data data {pi, li, mu, lambda, p0_init_guess}; // prepare the data for the opt
                optimizePayload(p0_opt, dim, p0_init_guess, data);
                p0_init_guess << p0_opt(0), p0_opt(1), p0_opt(2);
                size_t robot_counter = 0;
                for (const auto& p : pi) {
                    float distance = (p - p0_opt).norm();
                    double tol = abs(distance - li[robot_counter]);
                    if (tol > max_tol) {
                        std::cout << "robot_id: "<< robot_counter << ", tol: " << tol  << std::endl;
                        std::cout << "p: \n" << pi[robot_counter] << "\np0: \n" << p0_opt << std::endl;
                        early_conflict.time = t * all_robots[0]->ref_dt;
                        early_conflict.robot_idx_i = robot_counter; 
                        early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                        std::cout << "CONFLICT at time " << t*all_robots[0]->ref_dt << " " << early_conflict.robot_idx_i << std::endl;
                        early_conflict.robot_idx_j = 99;
                        // early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
                        assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
                        return true;
                    }
                ++robot_counter;
                }
                // artificial cables collision checker
                // creates a conflict if the artificial cable is in collision with an obstacle in the environment
                // The length of the cable is adjusted based on the position of the robot and the 
                // optimized payload position. 
                size_t num_robots = all_robots.size(); 
                cables.cablesObj.clear();
                for (size_t i=0; i < num_robots; ++i) {
                    Eigen::Vector3d qi = (p0_opt.cast<double>() - pi[i].cast<double>()).normalized(); 
                    double cable_l =  (p0_opt.cast<double>() - pi[i].cast<double>()).norm();
                    std::shared_ptr<fcl::CollisionGeometryd> cablegeom(new fcl::Capsuled(0.01, cable_l)); //length of cable + diameter of robot (radius = 0.1m)
                    cablegeom->setUserData((void*) i);
                    auto cableco = new fcl::CollisionObject(cablegeom);
                    Eigen::Vector3d cable_pos = p0_opt.cast<double>() - 0.5*cable_l*qi;
                    Eigen::Vector3d from(0., 0., -1.);
                    Eigen::Vector4d cable_quat(Eigen::Quaternion<double>::FromTwoVectors(from, qi).coeffs());
                    
                    fcl::Transform3d result;
                    result = Eigen::Translation<double, 3>(cable_pos);
                    result.rotate(Eigen::Quaterniond(cable_quat));
                    
                    cables.cablesObj.push_back(cableco);
                    cables.cablesObj[i]->setTransform(result);
                    cables.cablesObj[i]->computeAABB();                

                }

                cables.col_mgr_cables->clear();
                cables.col_mgr_cables->registerObjects(cables.cablesObj);
                cables.col_mgr_cables->setup();
                cables.col_mgr_cables->update(cables.cablesObj);
                fcl::DefaultCollisionData<double> cable_collision_data;

                cables.col_mgr_cables->collide(all_robots[0]->env.get(), &cable_collision_data,
                                        fcl::DefaultCollisionFunction<double>);

                if (cable_collision_data.result.isCollision()) {
                    const auto& contact = cable_collision_data.result.getContact(0);
                    std::cout << "cable collision exists: \n" << "cables: " <<  cable_collision_data.result.isCollision()  << std::endl;
                    early_conflict.time = t * all_robots[0]->ref_dt;
                    early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
                    early_conflict.robot_idx_j = 99;
                    std::cout << "cable id 1:" << (size_t)contact.o1->getUserData() << std::endl;
                    std::cout << "cable id 2:" << (size_t)contact.o2->getUserData() << std::endl;
                    assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
                    early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                    return true;
                }
                p0_tmp.push_back(p0_opt);        
            } else if (startsWith(all_robots[0]->name, "unicycle")) {
                std::vector<Eigen::VectorXf> pi;
                std::vector<double> li;

                for (const auto& robot_obj : robot_objs) {
                    Eigen::Vector3f robot_pos = robot_obj->getTranslation().cast<float>();
                    pi.push_back(create_vector({robot_pos(0), robot_pos(1), 0}));
                    li.push_back(0.5); // TODO: this needs to be provided as an input
                }

                size_t num_robots = all_robots.size(); 
                for (size_t i=0; i < num_robots-1; ++i) {
                    double distance1 = (pi[i+1] - pi[i]).norm();
                    // double tol1 = abs(distance1 - 0.5); // Length of cable is assumed to be 0.5
                    double tol1 = abs(distance1 - 0.37); // Length of cable is assumed to be 0.5
                    std::cout << "tol: " << tol1  << std::endl;
                    if (tol1 > max_tol) {
                        early_conflict.time = t * all_robots[0]->ref_dt;
                        early_conflict.robot_idx_i = i; 
                        early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                        std::cout << "CONFLICT at time joint system: " << t*all_robots[0]->ref_dt << " " << early_conflict.robot_idx_i << std::endl;
                        early_conflict.robot_idx_j = i+1;
                        early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
                        assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
                        return true;
                    }
                }
                cables.cablesObj.clear();
                cables.robotsObj.clear();
                for (size_t i = 0; i < num_robots-1; ++i) {
                    
                    Eigen::Vector3d robot0_state =  pi[i].cast<double>();
                    Eigen::Vector3d robot1_state =   pi[i+1].cast<double>();
                    Eigen::Vector2d robot0_pos   =  robot0_state.segment(0, 2);
                    Eigen::Vector2d robot1_pos   =  robot1_state.segment(0, 2);
                    
                    Eigen::Vector2d qi((robot1_pos - robot0_pos).normalized());
                    double rod_l = (robot1_pos - robot0_pos).norm();
                    Eigen::Vector2d cable_pos = robot0_pos + 0.5*rod_l*qi; // rod length is updated

                    double thi = atan2(qi[1], qi[0]);

                    std::shared_ptr<fcl::CollisionGeometryd> cablegeom(new fcl::Boxd(0.6*rod_l,0.01,0.01)); //0.6 * length of rod  
                    cablegeom->setUserData((void*) i);
                    auto cableco = new fcl::CollisionObject(cablegeom);

                    fcl::Transform3d result;                    
                    result = Eigen::Translation<double, 3>(Eigen::Vector3d(cable_pos(0), cable_pos(1), 0.));
                    result.rotate(Eigen::AngleAxisd(thi, Eigen::Vector3d::UnitZ()));
                    
                    cables.cablesObj.push_back(cableco);
                    cables.cablesObj[i]->setTransform(result);
                    cables.cablesObj[i]->computeAABB();                

                }

                cables.col_mgr_cables->clear();
                cables.col_mgr_cables->registerObjects(cables.cablesObj);
                cables.col_mgr_cables->setup();
                cables.col_mgr_cables->update(cables.cablesObj);
                fcl::DefaultCollisionData<double> cable_collision_data;
                fcl::DefaultCollisionData<double> cable_cable_collision_data;

                cables.col_mgr_cables->collide(all_robots[0]->env.get(), &cable_collision_data,
                                        fcl::DefaultCollisionFunction<double>);

                cables.col_mgr_cables->collide( &cable_cable_collision_data,
                                        fcl::DefaultCollisionFunction<double>);

                // if (cable_collision_data.result.isCollision() || cable_cable_collision_data.result.isCollision()) {
                if (cable_collision_data.result.isCollision()) {

                    early_conflict.time = t * all_robots[0]->ref_dt;
                    // if (cable_cable_collision_data.result.isCollision()) { // cable/cable conflict
                    //     std::cout << "cable/cable collision exists: " << cable_cable_collision_data.result.isCollision()<< std::endl;
                    //     const auto& contact = cable_cable_collision_data.result.getContact(0);
                    //     early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
                    //     early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                    //     early_conflict.robot_idx_j = (size_t)contact.o2->getUserData();
                    //     early_conflict.robot_state_j = node_states[early_conflict.robot_idx_j];
                    //     std::cout << "robot i: " << node_states[early_conflict.robot_idx_i] << std::endl;
                    //     std::cout << "robot j: " << node_states[early_conflict.robot_idx_j] << std::endl;
                    //     std::cout << "cable id 1:" << (size_t)contact.o1->getUserData() << std::endl;
                    //     std::cout << "cable id 2:" << (size_t)contact.o2->getUserData() << std::endl;

                    // } else { // cable/env conflict
                    //     std::cout << "cable/env collision exists: " << cable_collision_data.result.isCollision()  << std::endl;
                    //     const auto& contact = cable_collision_data.result.getContact(0);
                    //     early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
                    //     early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                    //     early_conflict.robot_idx_j = 99;
                    //     std::cout << "cable id 1:" << (size_t)contact.o1->getUserData() << std::endl;

                    // }
                    std::cout << "cable/env collision exists: " << cable_collision_data.result.isCollision()  << std::endl;
                    const auto& contact = cable_collision_data.result.getContact(0);
                    early_conflict.robot_idx_i = (size_t)contact.o1->getUserData();
                    early_conflict.robot_state_i = node_states[early_conflict.robot_idx_i];
                    early_conflict.robot_idx_j = 99;
                    std::cout << "cable id 1:" << (size_t)contact.o1->getUserData() << std::endl;

                    assert(early_conflict.robot_idx_i != early_conflict.robot_idx_j);
                    return true;
                }
            }
        }
    }
    if (solve_p0) {
        if (startsWith(all_robots[0]->name, "quad3d")) {
            for (const auto& p0i : p0_tmp) {
                p0_sol.push_back(p0i);
            }
        }
    }
    return false;

}

void createConstraintsFromConflicts(const Conflict& early_conflict, std::map<size_t, std::vector<dynoplan::Constraint>>& constraints){
    constraints[early_conflict.robot_idx_i].push_back({early_conflict.time, early_conflict.robot_state_i});
    if (early_conflict.robot_idx_j != 99) {
        constraints[early_conflict.robot_idx_j].push_back({early_conflict.time, early_conflict.robot_state_j});
    } 
}

void export_solutions(const std::vector<LowLevelPlan<dynobench::Trajectory>>& solution, 
                        const int robot_numx, std::ofstream *out, int & expansions){
    float cost = 0;
    for (auto& n : solution)
      cost += n.trajectory.cost;
    *out << "cost: " << cost << std::endl; 
    *out << "expansions: " << expansions << std::endl;
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
