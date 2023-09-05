#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <msgpack.hpp>

// #include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"
#include "fclHelper.hpp"

#include "planresult.hpp"


namespace ob = ompl::base;
namespace oc = ompl::control;

ob::State* allocAndFillState(std::shared_ptr<ompl::control::SpaceInformation> si, const YAML::Node& node)
{
  ob::State* state = si->allocState();
  std::vector<double> reals;
  for (const auto &value : node) {
    reals.push_back(value.as<double>());
  }
  si->getStateSpace()->copyFromReals(state, reals);
  return state;
}

std::ofstream& printState(std::ofstream &stream, std::shared_ptr<ompl::control::SpaceInformation> si, const ob::State* state)
{
  std::vector<double> reals;
  si->getStateSpace()->copyToReals(reals, state);
  stream << "[";
  for (size_t d = 0; d < reals.size(); ++d)
  {
    stream << reals[d];
    if (d < reals.size() - 1)
    {
      stream << ",";
    }
  }
  stream << "]";
  return stream;
}

oc::Control *allocAndFillControl(std::shared_ptr<ompl::control::SpaceInformation> si, const YAML::Node &node)
{
  oc::Control *control = si->allocControl();
  for (size_t idx = 0; idx < node.size(); ++idx)
  {
    double* address = si->getControlSpace()->getValueAddressAtIndex(control, idx);
    if (address) {
      *address = node[idx].as<double>();
    }
  }
  return control;
}

std::ofstream& printAction(std::ofstream &stream, std::shared_ptr<ompl::control::SpaceInformation> si, oc::Control *action)
{
  const size_t dim = si->getControlSpace()->getDimension();
  stream << "[";
  for (size_t d = 0; d < dim; ++d)
  {
    double *address = si->getControlSpace()->getValueAddressAtIndex(action, d);
    stream << *address;
    if (d < dim - 1)
    {
      stream << ",";
    }
  }
  stream << "]";
  return stream;
}

class Motion
{
public:
  std::vector<ob::State*> states;
  std::vector<oc::Control*> actions;

  // Last state translated to origin (to support reverse search)
  ob::State* last_state_translated;

  std::shared_ptr<ShiftableDynamicAABBTreeCollisionManager<float>> collision_manager;
  std::vector<fcl::CollisionObjectf *> collision_objects;

  float cost;

  size_t idx;
  bool disabled;
};

// forward declaration
struct AStarNode;

struct compareAStarNode
{
  bool operator()(const AStarNode *a, const AStarNode *b) const;
};

// open type
typedef typename boost::heap::d_ary_heap<
    AStarNode *,
    boost::heap::arity<2>,
    boost::heap::compare<compareAStarNode>,
    boost::heap::mutable_<true>>
    open_t;

// Node type (used for open and explored states)
struct AStarNode
{
  // const ob::State *state;
  ob::State *state;

  float fScore;
  float gScore;

  // can arrive at this node at time gScore, starting from came_from, using motion used_motion
  struct arrival {
    float gScore;
    AStarNode* came_from;
    size_t used_motion;
    size_t arrival_idx;
  };
  std::vector<arrival> arrivals;

  open_t::handle_type handle;
  bool is_in_open;
  size_t current_arrival_idx;
  bool reaches_goal;
};

bool compareAStarNode::operator()(const AStarNode *a, const AStarNode *b) const
{
  // Sort order
  // 1. lowest fScore
  // 2. highest gScore

  // Our heap is a maximum heap, so we invert the comperator function here
  if (a->fScore != b->fScore)
  {
    return a->fScore > b->fScore;
  }
  else
  {
    return a->gScore < b->gScore;
  }
}

float heuristic(std::shared_ptr<Robot> robot, const ob::State *s, const ob::State *g, float delta, ompl::NearestNeighbors<AStarNode*>* heuristic_nn)
{
  if (heuristic_nn) {
    AStarNode node;
    node.state = const_cast<ob::State*>(s);
    return heuristic_nn->nearest(&node)->gScore;
  } else {
    // heuristic is the time it might take to get to the goal
    Eigen::Vector3f current_pos = robot->getTransform(s).translation();
    Eigen::Vector3f goal_pos = robot->getTransform(g).translation();

    float dist = (current_pos - goal_pos).norm();
    const float max_vel = robot->maxSpeed(); // m/s./db_    
    // const float time = dist / max_vel;
    const float time = std::max((dist-delta) / max_vel, 0.0f);
    return time;
    // return 0;
  }
}

struct Motions
{
  std::vector<Motion> motions;
  // Kd-tree for the first state of each primitive
  ompl::NearestNeighbors<Motion*>* T_m_start;
  // Kd-tree for the last state of each primitive
  ompl::NearestNeighbors<Motion*>* T_m_end;
};

void load_motions(
  msgpack::object msg_obj,
  std::shared_ptr<Robot> robot,
  std::string robot_type,
  size_t env_size,
  // float delta,
  // bool filterDuplicates,
  // float alpha,
  Motions& result)
{
    auto si = robot->getSpaceInformation();

    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(new fcl::DynamicAABBTreeCollisionManagerf());
    bpcm_env->setup();

    // set state validity checking for this space
    auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, bpcm_env, robot, false));

    std::shared_ptr<oc::StatePropagator> statePropagator(new RobotStatePropagator(si, robot));

    si->setPropagationStepSize(1);
    si->setMinMaxControlDuration(1, 1);
    si->setStateValidityChecker(stateValidityChecker);
    si->setStatePropagator(statePropagator);
    si->setup();

    // create a robot with no position bounds
    ob::RealVectorBounds position_bounds_no_bound(env_size);
    position_bounds_no_bound.setLow(-1e6);//std::numeric_limits<double>::lowest());
    position_bounds_no_bound.setHigh(1e6);//std::numeric_limits<double>::max());
    std::shared_ptr<Robot> robot_no_pos_bound = create_robot(robot_type, position_bounds_no_bound);
    auto si_no_pos_bound = robot_no_pos_bound->getSpaceInformation();
    si_no_pos_bound->setPropagationStepSize(1);
    si_no_pos_bound->setMinMaxControlDuration(1, 1);
    si_no_pos_bound->setStateValidityChecker(stateValidityChecker);
    si_no_pos_bound->setStatePropagator(statePropagator);
    si_no_pos_bound->setup();

    size_t num_states = 0;
    size_t num_invalid_states = 0;

    if (msg_obj.type != msgpack::type::ARRAY) {
      throw msgpack::type_error();
    }

    for (size_t i = 0; i < msg_obj.via.array.size; ++i) {  
      Motion m;
      // find the states
      auto item = msg_obj.via.array.ptr[i]; 
      if (item.type != msgpack::type::MAP) {
        throw msgpack::type_error();
      }
      // load the states
      for (size_t j = 0; j < item.via.map.size; ++j) { 
        auto key = item.via.map.ptr[j].key.as<std::string>();
        if (key == "states") {
          auto val = item.via.map.ptr[j].val;
          for (size_t k = 0; k < val.via.array.size; ++k) {
            ob::State* state = si->allocState();
            std::vector<double> reals;
            val.via.array.ptr[k].convert(reals);
            si->getStateSpace()->copyFromReals(state, reals);
            m.states.push_back(state);
            if (!si_no_pos_bound->satisfiesBounds(m.states.back())) {
              si_no_pos_bound->enforceBounds(m.states.back());
              ++num_invalid_states;
            }
          }
          break;
        }
      }
      num_states += m.states.size();
      // load the actions
      for (size_t j = 0; j < item.via.map.size; ++j) {
        auto key = item.via.map.ptr[j].key.as<std::string>();
        if (key == "actions") {
          auto val = item.via.map.ptr[j].val;
          for (size_t k = 0; k < val.via.array.size; ++k) {
            oc::Control *control = si->allocControl();
            std::vector<double> reals;
            val.via.array.ptr[k].convert(reals);
            for (size_t idx = 0; idx < reals.size(); ++idx) {
              double* address = si->getControlSpace()->getValueAddressAtIndex(control, idx);
              if (address) {
                *address = reals[idx];
              }
            }
            m.actions.push_back(control);
          }
          break;
        }
      }
      m.cost = m.actions.size() * robot->dt(); 
      m.idx = result.motions.size();

      // add the last state translated to the origin
      m.last_state_translated = si->cloneState(m.states.back());
      robot->setPosition(m.last_state_translated, fcl::Vector3f(0,0,0));

      // generate collision objects and collision manager for saved motion
      for (const auto &state : m.states)
      {
        for (size_t part = 0; part < robot->numParts(); ++part) {
          const auto &transform = robot->getTransform(state, part);

          auto co = new fcl::CollisionObjectf(robot->getCollisionGeometry(part));
          co->setTranslation(transform.translation());
          co->setRotation(transform.rotation());
          co->computeAABB();
          m.collision_objects.push_back(co);
        }
      }
      m.collision_manager.reset(new ShiftableDynamicAABBTreeCollisionManager<float>());
      m.collision_manager->registerObjects(m.collision_objects);

      m.disabled = false; 

      result.motions.push_back(m); 
    } // end of for loop, looping over all 5k motions
    std::cout << "Info: " << num_invalid_states << " states are invalid of " << num_states << std::endl;

    auto rng = std::default_random_engine{};
    std::shuffle(std::begin(result.motions), std::end(result.motions), rng);
    for (size_t idx = 0; idx < result.motions.size(); ++idx) {
      result.motions[idx].idx = idx;
    }

    // build kd-tree for motion primitives (start)
    if (si->getStateSpace()->isMetricSpace())
    {
      result.T_m_start = new ompl::NearestNeighborsGNATNoThreadSafety<Motion*>();
    } else {
      result.T_m_start = new ompl::NearestNeighborsSqrtApprox<Motion*>();
    }
    result.T_m_start->setDistanceFunction([si](const Motion* a, const Motion* b) { return si->distance(a->states[0], b->states[0]); });

    for (auto& motion : result.motions) {
      result.T_m_start->add(&motion); // keep initial states
    }

    // build kd-tree for motion primitives (end)
    if (si->getStateSpace()->isMetricSpace())
    {
      result.T_m_end = new ompl::NearestNeighborsGNATNoThreadSafety<Motion*>();
    } else {
      result.T_m_end = new ompl::NearestNeighborsSqrtApprox<Motion*>();
    }
    result.T_m_end->setDistanceFunction([si](const Motion* a, const Motion* b) { return si->distance(a->last_state_translated, b->last_state_translated); });

    for (auto& motion : result.motions) {
      result.T_m_end->add(&motion); // keep initial states
    }

    std::cout << "There are " << result.motions.size() << " motions!" << std::endl;
}

void disable_motions(
  std::shared_ptr<Robot> robot,
  float delta,
  bool filterDuplicates,
  float alpha,
  size_t num_max_motions,
  Motions& result)
{
  auto si = robot->getSpaceInformation();

  #if 0
  if (delta < 0) {
    Motion fakeMotion; 
    fakeMotion.idx = -1;
    fakeMotion.states.push_back(si->allocState());
    std::vector<Motion *> neighbors_m;
    size_t num_desired_neighbors = (size_t)-delta; 
    size_t num_samples = std::min<size_t>(1000, result.motions.size());

    auto state_sampler = si->allocStateSampler();
    float sum_delta = 0.0;
    for (size_t k = 0; k < num_samples; ++k) { 
      do {
        state_sampler->sampleUniform(fakeMotion.states[0]);
      } while (!si->isValid(fakeMotion.states[0]));
      robot->setPosition(fakeMotion.states[0], fcl::Vector3f(0, 0, 0));

        result.T_m_start->nearestK(&fakeMotion, num_desired_neighbors+1, neighbors_m); 

      float max_delta = si->distance(fakeMotion.states[0], neighbors_m.back()->states.front());
      sum_delta += max_delta;
    }
    float adjusted_delta = (sum_delta / num_samples) / alpha;
    std::cout << "Automatically adjusting delta to: " << adjusted_delta << std::endl;
    delta = adjusted_delta;
  }
  #endif

  // enable all motions
  for (size_t i = 0; i < result.motions.size(); ++i) {
      result.motions[i].disabled = false;
  }

  // disable duplicates
  if (filterDuplicates)
  {
    size_t num_duplicates = 0;
    Motion fakeMotion;
    fakeMotion.idx = -1;
    fakeMotion.states.push_back(si->allocState());
    std::vector<Motion *> neighbors_m;
    for (const auto& m : result.motions) {
      if (m.disabled) {
        continue;
      }

      si->copyState(fakeMotion.states[0], m.states[0]);
      result.T_m_start->nearestR(&fakeMotion, delta*alpha, neighbors_m); // finding applicable motions with discont.

      for (Motion* nm : neighbors_m) {
        if (nm == &m || nm->disabled) { 
          continue;
        }
        float goal_delta = si->distance(m.states.back(), nm->states.back());
        if (goal_delta < delta*(1-alpha)) {
          nm->disabled = true;
          ++num_duplicates;
        }
      }
    }
    std::cout << "There are " << num_duplicates << " duplicate motions!" << std::endl;

  }

  // limit to num_max_motions
  size_t num_enabled_motions = 0;
  for (size_t i = 0; i < result.motions.size(); ++i) {
    if (!result.motions[i].disabled) {
      if (num_enabled_motions >= num_max_motions) {
        result.motions[i].disabled = true;
      } else {
        ++num_enabled_motions;
      }
    }
  }

  std::cout << "There are " << num_enabled_motions << " motions enabled." << std::endl;
}

template <typename Constraint>
class DBAstar
{
public:
  DBAstar(float delta, float alpha)
    : delta(delta)
    , alpha(alpha)
    , epsilon(1.0)
    , maxCost(1e6)
  {
  }

  bool search(
    const Motions& motions,
    const std::vector<double>& robot_start,
    const std::vector<double>& robot_goal,
    const std::vector<fcl::CollisionObjectf *>& obstacles, 
    std::shared_ptr<Robot> robot,
    const std::vector<Constraint>& constraints,
    bool reverse_search,
    LowLevelPlan<AStarNode*,ob::State*,oc::Control*>& ll_result,
    ompl::NearestNeighbors<AStarNode*>* heuristic_nn = nullptr,
    ompl::NearestNeighbors<AStarNode*>** heuristic_result = nullptr)
  {
    auto si = robot->getSpaceInformation();

#ifdef DBG_PRINTS
    std::cout << "Running dbA*" << std::endl;
    for (const auto& constraint : constraints){
      std::cout << "constraint at time: " << constraint.time << std::endl;
      si->printState(constraint.constrained_state);
    }
#endif

    // ll_result.plan.clear();
    ll_result.trajectory.clear();
    ll_result.actions.clear();
    ll_result.cost = 0;

    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(new fcl::DynamicAABBTreeCollisionManagerf());
    bpcm_env->registerObjects(obstacles);
    bpcm_env->setup();

    // set number of control steps
    si->setPropagationStepSize(1);
    si->setMinMaxControlDuration(1, 1);

    // set state validity checking for this space
    auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, bpcm_env, robot, false));
    si->setStateValidityChecker(stateValidityChecker);

    // set the state propagator
    std::shared_ptr<oc::StatePropagator> statePropagator(new RobotStatePropagator(si, robot));
    si->setStatePropagator(statePropagator);

    si->setup();
    auto startState = si->allocState();
    if (!reverse_search) {
      si->getStateSpace()->copyFromReals(startState, robot_start);
    } else {
      si->getStateSpace()->copyFromReals(startState, robot_goal);
    }
    si->enforceBounds(startState);
    
    // set goal state
    auto goalState = si->allocState();
    if (!reverse_search) {
      si->getStateSpace()->copyFromReals(goalState, robot_goal);
    } else {
      si->getStateSpace()->copyFromReals(goalState, robot_start);
    }

    if (isnan(robot_start[0])) {
      si->freeState(goalState);
      goalState = nullptr;
    } else {
      si->enforceBounds(goalState);
    }

#ifdef DBG_PRINTS
    std::cout << "Max cost is " << maxCost << std::endl;
#endif
    if (alpha <= 0 || alpha >= 1) {
#ifdef DBG_PRINTS
      std::cerr << "Alpha needs to be between 0 and 1!" << std::endl;
#endif
      return 1;
    }



//// db-A* search
  open_t open;

  // kd-tree for nodes
  ompl::NearestNeighbors<AStarNode*> *T_n;
  if (si->getStateSpace()->isMetricSpace())
  {
    T_n = new ompl::NearestNeighborsGNATNoThreadSafety<AStarNode*>();
  }
  else
  {
    T_n = new ompl::NearestNeighborsSqrtApprox<AStarNode*>();
  }
  if (heuristic_result) {
    *heuristic_result = T_n;
  }
  T_n->setDistanceFunction([si](const AStarNode* a, const AStarNode* b)
                           { return si->distance(a->state, b->state); });

  auto start_node = new AStarNode();
  start_node->state = startState;
  start_node->gScore = 0;
  if (goalState) {
    start_node->fScore = goalState ? epsilon * heuristic(robot, startState, goalState, delta, heuristic_nn) : 0;
  } else {
    start_node->fScore = 0;
  }
  start_node->arrivals.push_back({.gScore = 0, .came_from = nullptr, .used_motion = (size_t)-1, .arrival_idx = (size_t)-1});

  auto handle = open.push(start_node); 
  start_node->handle = handle;
  start_node->is_in_open = true;
  start_node->current_arrival_idx = 0;
  start_node->reaches_goal = (goalState && si->distance(startState, goalState) <= delta);

  T_n->add(start_node);

  Motion fakeMotion;
  fakeMotion.idx = -1;
  fakeMotion.states.push_back(si->allocState());
  fakeMotion.last_state_translated = si->allocState();

  AStarNode* query_n = new AStarNode();

  ob::State* tmpState = si->allocState();
  ob::State* tmpStateconst = si->allocState();
  std::vector<Motion*> neighbors_m; // applicable
  std::vector<AStarNode*> neighbors_n; // explored

  float last_f_score = start_node->fScore;
  size_t expands = 0;
  // Entering loop while Open set is non-empty
  while (!open.empty())
  {
    AStarNode* current = open.top();
    // std::cout << expands << " " << current->fScore << std::endl;
    ++expands;
    if (expands % 1000 == 0) {
      std::cout << "expanded: " << expands << " open: " << open.size() << " nodes: " << T_n->size() << " f-score " << current->fScore << std::endl;
    }
    
    // assert(current->fScore >= last_f_score);
    last_f_score = current->fScore;
    bool is_at_goal = current->reaches_goal;
    if (is_at_goal) {
      // check if we violate any constraint if we stay there
      for (const auto& constraint : constraints) {
        if (constraint.time >= current->gScore - 1e-6) {
          bool violation = si->distance(current->state, constraint.constrained_state) <= delta;
          if (violation) {
            is_at_goal = false;
            break;
          }
        }
      }
    }

    if (is_at_goal) {
#ifdef DBG_PRINTS
      std::cout << "SOLUTION FOUND !!!! cost: " << current->gScore << " dist " << si->distance(current->state, goalState) << std::endl;
#endif

      std::vector<std::pair<AStarNode*, size_t>> result;
      AStarNode* n = current;

      size_t arrival_idx = current->current_arrival_idx;
      while (n != nullptr) {
        result.push_back(std::make_pair(n, arrival_idx));
        const auto& arrival = n->arrivals[arrival_idx];
        n = arrival.came_from;
        arrival_idx = arrival.arrival_idx;
      }
      std::reverse(result.begin(), result.end());
      // ll_result.plan = result;
      ll_result.cost = current->gScore;

      for (size_t i = 0; i < result.size() - 1; ++i)
      {
        // Compute intermediate states
        const auto node_state = result[i].first->state;
        const fcl::Vector3f current_pos = robot->getTransform(node_state).translation();
        const auto &motion = motions.motions.at(result[i+1].first->arrivals[result[i+1].second].used_motion);
        
        for (size_t k = 0; k < motion.states.size()-1; ++k) // skipping the last state
        {
          const auto state = motion.states[k];
          ob::State* motion_state = si->allocState(); // alternative 
          si->copyState(motion_state, state);
          const fcl::Vector3f relative_pos = robot->getTransform(state).translation();
          robot->setPosition(motion_state, current_pos + relative_pos);
#ifdef DBG_PRINTS
          si->printState(motion_state);
#endif
          ll_result.trajectory.push_back(motion_state);
        }
      } // writing result states
#ifdef DBG_PRINTS
      si->printState(result.back().first->state);
#endif
      ll_result.trajectory.push_back(si->cloneState(result.back().first->state));

      for (size_t i = 0; i < result.size() - 1; ++i)
      {
        const auto &motion = motions.motions.at(result[i+1].first->arrivals[result[i+1].second].used_motion);

        for (size_t k = 0; k < motion.actions.size(); ++k)
        {
          const auto& action = motion.actions[k];
          oc::Control* motion_action = si->allocControl(); 
          si->copyControl(motion_action, action);
          ll_result.actions.push_back(motion_action);
        }
      } // write actions to yaml file

      // sanity check on the sizes
      assert(ll_result.actions.size() + 1 == ll_result.trajectory.size());

      // sanity check on the bounds
      double largest_dist = si->distance(startState, ll_result.trajectory[0]);
      for (size_t i = 1; i < ll_result.trajectory.size(); ++i) {
        double dist = si->distance(ll_result.trajectory[i-1], ll_result.trajectory[i]);
        // std::cout << i << " " << dist << std::endl;
        largest_dist = std::max(largest_dist, dist);
      }
      largest_dist = std::max(largest_dist, si->distance(ll_result.trajectory.back(), goalState));
      if (largest_dist > delta){
        std::cerr << "Warning: delta violation " << std::to_string(largest_dist) << std::endl;
      }

      // #ifndef NDEBUG
      // Sanity check here, that verifies that we obey all constraints
#ifdef DBG_PRINTS
      std::cout << "checking constraints " << std::endl;
#endif
      for (const auto& constraint : constraints) {
#ifdef DBG_PRINTS
        std::cout << "constraint t=" << constraint.time << std::endl;
        si->printState(constraint.constrained_state);
#endif
        int time_index = std::lround(constraint.time / robot->dt());
        assert(time_index >= 0);
        time_index = std::min<int>(time_index, (int)ll_result.trajectory.size()-1);
        assert(time_index < (int)ll_result.trajectory.size());

        float dist = si->distance(ll_result.trajectory.at(time_index), constraint.constrained_state);

        if (dist <= delta){
          std::cout << "VIOLATION " << dist << " " << time_index << " " << ll_result.trajectory.size() << std::endl;
          si->printState(ll_result.trajectory.at(time_index));
          throw std::runtime_error("Internal error: constraint violation in solution!");
        }

        assert(dist > delta);
        #if 0
        auto transform = robot->getTransform(ll_result.trajectory.at(time_index), 0);
        fcl::CollisionObjectf motion_state_co(robot->getCollisionGeometry(0)); 
        motion_state_co.setTranslation(transform.translation());
        motion_state_co.setRotation(transform.rotation());
        motion_state_co.computeAABB();

        const auto& other_state = constraint.constrained_state;
        auto other_transform = robot->getTransform(other_state, 0);
        fcl::CollisionObjectf other_robot_co(robot->getCollisionGeometry(0)); 
        other_robot_co.setTranslation(other_transform.translation());
        other_robot_co.setRotation(other_transform.rotation());
        other_robot_co.computeAABB();

        fcl::CollisionRequest<float> request;
        fcl::CollisionResult<float> result;
        // check two states for collision
        collide(&motion_state_co, &other_robot_co, request, result);

        if (result.isCollision()) {
          std::cout << "VIOLATION";
          si->printState(ll_result.trajectory.at(time_index));
        }

        assert(!result.isCollision());
        #endif
      }
      // #endif

      return true;
      // break;
    } // if solution found things

    // If no solution then continue
    current->is_in_open = false;
    open.pop();

    // find relevant motions (within delta/2 of current state)
    si->copyState(fakeMotion.states[0], current->state);
    robot->setPosition(fakeMotion.states[0], fcl::Vector3f(0,0,0));

    if (!reverse_search) {
      motions.T_m_start->nearestR(&fakeMotion, delta*alpha, neighbors_m);
    } else {
      si->copyState(fakeMotion.last_state_translated, fakeMotion.states[0]);
      motions.T_m_end->nearestR(&fakeMotion, delta*alpha, neighbors_m);
    }
    // Loop over all potential applicable motions
    for (const Motion* motion : neighbors_m) {
      if (motion->disabled) {
        continue;
      }

      // compute estimated cost
      float tentative_gScore = current->gScore + motion->cost;
      Eigen::Vector3f relative_pos;
      if (!reverse_search) {
        si->copyState(tmpState, motion->states.back());
        relative_pos = robot->getTransform(motion->states.back()).translation();
      } else {
        si->copyState(tmpState, motion->states.front());
        relative_pos = -robot->getTransform(motion->states.back()).translation();
      }
      Eigen::Vector3f current_pos = robot->getTransform(current->state).translation();
      Eigen::Vector3f offset = current_pos;
      robot->setPosition(tmpState, offset + relative_pos);
      // compute estimated fscore
      float tentative_hScore = goalState ? epsilon * heuristic(robot, tmpState, goalState, delta, heuristic_nn) : 0;
      float tentative_fScore = tentative_gScore + tentative_hScore;

      // skip motions that would exceed cost bound
      if (tentative_fScore > maxCost)
      {
        continue;
      }
      // skip motions that are invalid
      if (!si->satisfiesBounds(tmpState))
      {
        continue;
      }

      // Compute intermediate states and check their validity
#if 0
      bool motionValid = true;
      for (const auto& state : motion->states)
      {
        // const auto& state = motion->states.back();
        si->copyState(tmpState, state);
        const auto relative_pos = robot->getTransform(state).translation();
        robot->setPosition(tmpState, offset + relative_pos);

        if (!si->isValid(tmpState)) {
          motionValid = false;
          // std::cout << "invalid";
          break;
        }
      }
      #else
      motion->collision_manager->shift(offset); 
      fcl::DefaultCollisionData<float> collision_data;
      motion->collision_manager->collide(bpcm_env.get(), &collision_data, fcl::DefaultCollisionFunction<float>);
      bool motionValid = !collision_data.result.isCollision();
      motion->collision_manager->shift(-offset);
    
      if (!motionValid) {
        // std::cout << "skip invalid motion" << std::endl;
        continue;
      }
      // now check with dynamic constraints
      bool reachesGoal = goalState && si->distance(tmpState, goalState) <= delta;

      for (const auto& constraint : constraints) {
        // a constraint violation can only occur between t in [current->gScore, tentative_gScore]
        float time_offset = constraint.time - current->gScore;
        int time_index = std::lround(time_offset / robot->dt());

        // if this motion reaches the goal and we have a constraint in the future, check with the last state
        ob::State* state_to_check = nullptr;
        if (reachesGoal && time_index >= (int)motion->states.size() - 1) {
          state_to_check = motion->states.back();
        }

        if (time_index >= 0 && time_index < (int)motion->states.size() - 1) {
          state_to_check = motion->states[time_index];
        }

        if (state_to_check) {
          // compute translated state
          si->copyState(tmpStateconst, state_to_check);
          const auto relative_pos = robot->getTransform(state_to_check).translation();
          robot->setPosition(tmpStateconst, offset + relative_pos);
          bool violation = si->distance(tmpStateconst, constraint.constrained_state) <= delta;
          if (violation) {
            motionValid = false;
            break;
          }

        }
      } 

#endif

      // Skip this motion, if it isn't valid
      if (!motionValid) {
        // std::cout << "skip invalid motion" << std::endl;
        continue;
      }
      // Check if we have this state (or any within delta/2) already
      query_n->state = tmpState;  
      // avoid considering this an old state for very short motions
      float radius = delta*(1-alpha);
      T_n->nearestR(query_n, radius, neighbors_n);

      if (neighbors_n.size() == 0)
      // if (nearest_distance > radius)
      {
        // new state -> add it to open and T_n
        auto node = new AStarNode();
        node->state = si->cloneState(tmpState);
        node->gScore = tentative_gScore;
        node->fScore = tentative_fScore;
        node->arrivals.push_back({.gScore = tentative_gScore, .came_from = current, .used_motion = motion->idx, .arrival_idx = current->current_arrival_idx});
        node->is_in_open = true;
        auto handle = open.push(node);
        node->handle = handle;
        node->current_arrival_idx = 0;
        node->reaches_goal = reachesGoal;
        T_n->add(node);

      }
      else
      {
        // check if we have a better path now
        for (AStarNode* entry : neighbors_n) {
        // AStarNode* entry = nearest;
          assert(si->distance(entry->state, tmpState) <= delta);
          float delta_score = entry->gScore - tentative_gScore;
          if (delta_score > 0) {
// #ifdef DBG_PRINTS
//             std::cout << "attempt to update node-id " << entry->id << " (from " << entry->gScore << " to " << tentative_gScore << " rG " << entry->reaches_goal << ")" << std::endl;
// #endif
            // check if an update would not violate our additional constraints
            // check if we now violate a final constraint
            bool update_valid = true;
            if (entry->reaches_goal) {
              for (const auto& constraint : constraints) {
// #ifdef DBG_PRINTS
//                 std::cout << "check ctr " << constraint.time << " " << tentative_gScore << " " << (constraint.time >= tentative_gScore - 1e-6) << " " << si->distance(entry->state, constraint.constrained_state) << std::endl;
// #endif
                if (constraint.time >= tentative_gScore - 1e-6) {
                  bool violation = si->distance(entry->state, constraint.constrained_state) <= delta;
                  if (violation) {
                    update_valid = false;
// #ifdef DBG_PRINTS
//                     std::cout << "would violate future constraint!" << std::endl;
// #endif
                    break;
                  }
                }
              }
            }

            if (update_valid) {
// #ifdef DBG_PRINTS
//               std::cout << "update node-id " << entry->id << " " << affected_nodes.size() << std::endl;
// #endif
              // std::cout << "update " << entry->gScore << " to " << tentative_gScore << " " << entry->arrivals.size() << std::endl;

              entry->gScore = tentative_gScore;
              entry->fScore -= delta_score;
              assert(entry->fScore >= 0);
              entry->arrivals.push_back({.gScore = tentative_gScore, .came_from = current, .used_motion = motion->idx, .arrival_idx = current->current_arrival_idx});
              ++entry->current_arrival_idx;
              if (entry->is_in_open) {
                open.increase(entry->handle);
              } else {
                // TODO: is this correct?
                auto handle = open.push(entry);
                entry->handle = handle;
                entry->is_in_open = true;
              }
            }
          }
        }
      }
    }

    } // While OpenSet not empyt ends here

    std::cout << "No solution found!" << std::endl;
    return false;
  } // end of search function

private:
  float delta;
  float alpha;
  float epsilon;
  float maxCost;


}; // end of DBAstar class

// TO DO:
// delete New to clear the memory

