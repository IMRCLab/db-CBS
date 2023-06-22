#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

// FCL
//#include <fcl/fcl.h>

// YAML
#include <yaml-cpp/yaml.h>

// OMPL
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

// local
#include "robots.h"
#include "robotStatePropagator.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace ob = ompl::base;
namespace oc = ompl::control;

class RobotHelper
{
public:
  RobotHelper(const std::string& robotType, float pos_limit = 2)
  {
    size_t dim = 2;
    if (robotType == "quadrotor_0") {
      dim = 3;
    }

    ob::RealVectorBounds position_bounds(dim);
    position_bounds.setLow(-pos_limit);
    position_bounds.setHigh(pos_limit);
    robot_ = create_robot(robotType, position_bounds);

    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->setup();
    state_sampler_ = si->allocStateSampler();
    control_sampler_ = si->allocControlSampler();

    tmp_state_a_ = si->allocState();
    tmp_state_b_ = si->allocState();
    tmp_control_ = si->allocControl();
  }

  ~RobotHelper()
  {
    auto si = robot_->getSpaceInformation();
    si->freeState(tmp_state_a_);
    si->freeState(tmp_state_b_);
    si->freeControl(tmp_control_);
  }

  float distance(const std::vector<double> &stateA, const std::vector<double> &stateB)
  {
    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->copyFromReals(tmp_state_a_, stateA);
    si->getStateSpace()->copyFromReals(tmp_state_b_, stateB);
    return si->distance(tmp_state_a_, tmp_state_b_);
  }

  std::vector<double> sampleStateUniform()
  {
    auto si = robot_->getSpaceInformation();
    do {
      state_sampler_->sampleUniform(tmp_state_a_);
    } while(!si->satisfiesBounds(tmp_state_a_));
    std::vector<double> reals;
    si->getStateSpace()->copyToReals(reals, tmp_state_a_);
    return reals;
  }

  std::vector<double> sampleControlUniform()
  {
    control_sampler_->sample(tmp_control_);
    auto si = robot_->getSpaceInformation();
    const size_t dim = si->getControlSpace()->getDimension();
    std::vector<double> reals(dim);
    for (size_t d = 0; d < dim; ++d)
    {
      double *address = si->getControlSpace()->getValueAddressAtIndex(tmp_control_, d);
      reals[d] = *address;
    }
    return reals;
  }

  std::vector<double> step(
    const std::vector<double> &state,
    const std::vector<double>& action,
    double duration)
  {
    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->copyFromReals(tmp_state_a_, state);

    const size_t dim = si->getControlSpace()->getDimension();
    assert(dim == action.size());
    for (size_t d = 0; d < dim; ++d) {
      double *address = si->getControlSpace()->getValueAddressAtIndex(tmp_control_, d);
      *address = action[d];
    }
    robot_->propagate(tmp_state_a_, tmp_control_, duration, tmp_state_b_);

    std::vector<double> reals;
    si->getStateSpace()->copyToReals(reals, tmp_state_b_);
    return reals;
  }

  std::vector<double> interpolate(
    const std::vector<double>& stateFrom,
    const std::vector<double>& stateTo,
    double t)
  {
    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->copyFromReals(tmp_state_a_, stateFrom);
    si->getStateSpace()->copyFromReals(tmp_state_b_, stateTo);

    si->getStateSpace()->interpolate(tmp_state_a_, tmp_state_b_, t, tmp_state_a_);

    std::vector<double> reals;
    si->getStateSpace()->copyToReals(reals, tmp_state_a_);
    return reals;
  }

  bool is2D() const
  {
    return robot_->is2D();
  }

  std::vector<size_t> sortMotions(
    const std::vector<std::vector<double>> &x0s,
    const std::vector<std::vector<double>> &xfs,
    size_t top_k)
  {
    assert(x0s.size() == xfs.size());
    assert(x0s.size() > 0);
    auto si = robot_->getSpaceInformation();
    struct Motion
    {
      ob::State* x0;
      ob::State* xf;
      size_t idx;
    };
    // create vector of motions
    std::vector<Motion> motions;
    for (size_t i = 0; i < x0s.size(); ++i) {
      Motion m;
      m.x0 = si->allocState();
      si->getStateSpace()->copyFromReals(m.x0, x0s[i]);
      si->enforceBounds(m.x0);
      m.xf = si->allocState();
      si->getStateSpace()->copyFromReals(m.xf, xfs[i]);
      si->enforceBounds(m.xf);
      m.idx = i;
      motions.push_back(m);
    }

    // build kd-tree for Tx0
    ompl::NearestNeighbors<Motion*>* Tx0;
    if (si->getStateSpace()->isMetricSpace())
    {
      Tx0 = new ompl::NearestNeighborsGNATNoThreadSafety<Motion*>();
    } else {
      Tx0 = new ompl::NearestNeighborsSqrtApprox<Motion*>();
    }
    Tx0->setDistanceFunction([si](const Motion* a, const Motion* b) { return si->distance(a->x0, b->x0); });

    // build kd-tree for Txf
    ompl::NearestNeighbors<Motion*>* Txf;
    if (si->getStateSpace()->isMetricSpace())
    {
      Txf = new ompl::NearestNeighborsGNATNoThreadSafety<Motion*>();
    } else {
      Txf = new ompl::NearestNeighborsSqrtApprox<Motion*>();
    }
    Txf->setDistanceFunction([si](const Motion* a, const Motion* b) { return si->distance(a->xf, b->xf); });

    // use as first/seed motion the one that moves furthest
    std::vector<size_t> used_motions;
    size_t best_motion = 0;
    double largest_d = 0;
    for (const auto& m : motions) {
      double d = si->distance(m.x0, m.xf);
      if (d > largest_d) {
        largest_d = d;
        best_motion = m.idx;
      }
    }
    used_motions.push_back(best_motion);
    Tx0->add(&motions[best_motion]);
    Txf->add(&motions[best_motion]);

    std::set<size_t> unused_motions;
    for (const auto& m : motions) {
      unused_motions.insert(m.idx);
    }
    unused_motions.erase(best_motion);

    for (size_t k = 1; k < top_k; ++k) {
      std::cout << "sorting " << k << std::endl;
      double best_d = -1;
      size_t best_motion = -1;
      for (size_t m1 : unused_motions) {
        // find smallest distance to existing neighbors
        auto m2 = Tx0->nearest(&motions[m1]);
        double smallest_d_x0 = si->distance(motions[m1].x0, m2->x0);

        m2 = Txf->nearest(&motions[m1]);
        double smallest_d_xf = si->distance(motions[m1].xf, m2->xf);

        double smallest_d = smallest_d_x0 + smallest_d_xf;
        if (smallest_d > best_d) {
          best_motion = m1;
          best_d = smallest_d;
        }
      }
      used_motions.push_back(best_motion);
      unused_motions.erase(best_motion);
      Tx0->add(&motions[best_motion]);
      Txf->add(&motions[best_motion]);
    }

    // clean-up memory
    for (const auto& m : motions) {
      si->freeState(m.x0);
      si->freeState(m.xf);
    }

    return used_motions;
  }

private: 
  std::shared_ptr<Robot> robot_;
  ob::StateSamplerPtr state_sampler_;
  oc::ControlSamplerPtr control_sampler_;
  ob::State* tmp_state_a_;
  ob::State* tmp_state_b_;
  oc::Control* tmp_control_;
};

class CollisionChecker
{
public:
  CollisionChecker()
    : tmp_state_(nullptr)
  {

  }

  ~CollisionChecker()
  {
    if (robot_ && tmp_state_) {
      auto si = robot_->getSpaceInformation();
      si->freeState(tmp_state_);
    }
  }

  void load(const std::string& filename)
  {
    if (robot_ && tmp_state_) {
      auto si = robot_->getSpaceInformation();
      si->freeState(tmp_state_);
    }

    YAML::Node env = YAML::LoadFile(filename);

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
    env_.reset(new fcl::DynamicAABBTreeCollisionManagerf());
    // std::shared_ptr<fcl::BroadPhaseCollisionManagerf> bpcm_env(new fcl::NaiveCollisionManagerf());
    env_->registerObjects(obstacles);
    env_->setup();

    const auto &robot_node = env["robots"][0];
    auto robotType = robot_node["type"].as<std::string>();
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
      position_bounds.setLow(i, env_min[i].as<double>());
      position_bounds.setHigh(i, env_max[i].as<double>());
    }
    robot_ = create_robot(robotType, position_bounds);

    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->setup();

    tmp_state_ = si->allocState();
  }

  auto distance(const std::vector<double>& state)
  {

    auto si = robot_->getSpaceInformation();
    si->getStateSpace()->copyFromReals(tmp_state_, state);

    std::vector<fcl::DefaultDistanceData<float>> distance_data(robot_->numParts());
    size_t min_idx = 0;
    for (size_t part = 0; part < robot_->numParts(); ++part) {
      const auto &transform = robot_->getTransform(tmp_state_, part);
      fcl::CollisionObjectf robot(robot_->getCollisionGeometry(part)); //, robot_->getTransform(state));
      robot.setTranslation(transform.translation());
      robot.setRotation(transform.rotation());
      robot.computeAABB();
      distance_data[part].request.enable_signed_distance = true;
      env_->distance(&robot, &distance_data[part], fcl::DefaultDistanceFunction<float>);
      if (distance_data[part].result.min_distance < distance_data[min_idx].result.min_distance) {
        min_idx = part;
      }
    }

    return std::make_tuple(
      distance_data[min_idx].result.min_distance,
      distance_data[min_idx].result.nearest_points[0],
      distance_data[min_idx].result.nearest_points[1]);
  }

private:
  std::shared_ptr<fcl::CollisionGeometryf> geom_;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerf> env_;
  std::shared_ptr<Robot> robot_;
  ob::State *tmp_state_;
};

PYBIND11_MODULE(motionplanningutils, m)
{
  pybind11::class_<CollisionChecker>(m, "CollisionChecker")
      .def(pybind11::init())
      .def("load", &CollisionChecker::load)
      .def("distance", &CollisionChecker::distance);

  pybind11::class_<RobotHelper>(m, "RobotHelper")
      .def(pybind11::init<const std::string &, float>(), py::arg("robot_type"), py::arg("pos_limit") = 2)
      .def("distance", &RobotHelper::distance)
      .def("sampleUniform", &RobotHelper::sampleStateUniform)
      .def("sampleControlUniform", &RobotHelper::sampleControlUniform)
      .def("step", &RobotHelper::step)
      .def("interpolate", &RobotHelper::interpolate)
      .def("is2D", &RobotHelper::is2D)
      .def("sortMotions", &RobotHelper::sortMotions);
}
