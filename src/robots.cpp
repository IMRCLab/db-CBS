#include "robots.h"

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class RobotUnicycleFirstOrder : public Robot
{
public:
  RobotUnicycleFirstOrder(
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, float v_max,
    float w_min, float w_max)
  {
    geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));

    auto space(std::make_shared<ob::SE2StateSpace>());
    space->setBounds(position_bounds);

    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, v_min);
    cbounds.setHigh(0, v_max);
    cbounds.setLow(1, w_min);
    cbounds.setHigh(1, w_max);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.1;
    is2D_ = true;
    max_speed_ = std::max(fabsf(v_min), fabsf(v_max));
  }

  void propagate(
    const ompl::base::State *start,
    const ompl::control::Control *control,
    const double duration,
    ompl::base::State *result) override
  {
    auto startTyped = start->as<ob::SE2StateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    auto resultTyped = result->as<ob::SE2StateSpace::StateType>();

    // use simple Euler integration
    float x = startTyped->getX();
    float y = startTyped->getY();
    float yaw = startTyped->getYaw();
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      yaw += ctrl[1] * dt;
      x += ctrl[0] * cosf(yaw) * dt;
      y += ctrl[0] * sinf(yaw) * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->setX(x);
    resultTyped->setY(y);
    resultTyped->setYaw(yaw);

    // Normalize orientation
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(resultTyped->as<ob::SO2StateSpace::StateType>(1));
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
    float yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////

class RobotUnicycleSecondOrder : public Robot
{
public:
  RobotUnicycleSecondOrder(
      const ompl::base::RealVectorBounds &position_bounds,
      float v_limit,      // max velocity in m/s
      float w_limit,      // max angular velocity in rad/s
      float a_limit,      // max accelleration in m/s^2
      float w_dot_limit) // max angular acceleration in rad/s^2
  {
    geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));

    auto space(std::make_shared<StateSpace>());
    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vel_bounds(1);
    vel_bounds.setLow(-v_limit);
    vel_bounds.setHigh(v_limit);
    space->setVelocityBounds(vel_bounds);

    ob::RealVectorBounds w_bounds(1);
    w_bounds.setLow(-w_limit);
    w_bounds.setHigh(w_limit);
    space->setAngularVelocityBounds(w_bounds);

    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -a_limit);
    cbounds.setHigh(0, a_limit);
    cbounds.setLow(1, -w_dot_limit);
    cbounds.setHigh(1, w_dot_limit);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.1;
    is2D_ = true;
    max_speed_ = v_limit;
  }

  void propagate(
      const ompl::base::State *start,
      const ompl::control::Control *control,
      const double duration,
      ompl::base::State *result) override
  {
    auto startTyped = start->as<StateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    auto resultTyped = result->as<StateSpace::StateType>();

    // use simple Euler integration
    float x = startTyped->getX();
    float y = startTyped->getY();
    float yaw = startTyped->getYaw();
    float v = startTyped->getVelocity();
    float w = startTyped->getAngularVelocity();
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      // For compatibility with KOMO, update v and yaw first
      v += ctrl[0] * dt;
      w += ctrl[1] * dt;
      yaw += w * dt;
      x += v * cosf(yaw) * dt;
      y += v * sinf(yaw) * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->setX(x);
    resultTyped->setY(y);
    resultTyped->setYaw(yaw);
    resultTyped->setVelocity(v);
    resultTyped->setAngularVelocity(w);

    // Normalize orientation
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(resultTyped->as<ob::SO2StateSpace::StateType>(1));
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
    float yaw = stateTyped->getYaw();
    result.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    return result;
  }

  virtual void setPosition(
      ompl::base::State *state,
      const fcl::Vector3f position) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getYaw() const
      {
        return as<ob::SO2StateSpace::StateType>(1)->value;
      }

      double getVelocity() const
      {
        return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
      }

      double getAngularVelocity() const
      {
        return as<ob::RealVectorStateSpace::StateType>(3)->values[0];
      }

      void setX(double x)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setYaw(double yaw)
      {
        as<ob::SO2StateSpace::StateType>(1)->value = yaw;
      }

      void setVelocity(double velocity)
      {
        as<ob::RealVectorStateSpace::StateType>(2)->values[0] = velocity;
      }

      void setAngularVelocity(double angularVelocity)
      {
        as<ob::RealVectorStateSpace::StateType>(3)->values[0] = angularVelocity;
      }
    };

    StateSpace()
    {
      setName("CarSO" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);  // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);          // orientation
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 0.25); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 0.25); // angular velocity
      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const
    {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getVelocityBounds() const
    {
      return as<ob::RealVectorStateSpace>(2)->getBounds();
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getAngularVelocityBounds() const
    {
      return as<ob::RealVectorStateSpace>(3)->getBounds();
    }

    ob::State *allocState() const override
    {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override
    {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override
    {
      class DefaultProjection : public ob::ProjectionEvaluator
      {
      public:
        DefaultProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
          return 2;
        }

        void defaultCellSizes() override
        {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };
};

////////////////////////////////////////////////////////////////////////////////////////////////

class RobotCarFirstOrderWithTrailers : public Robot
{
public:
  RobotCarFirstOrderWithTrailers(
      const ompl::base::RealVectorBounds &position_bounds,
      float v_min,
      float v_max,
      float phi_min,
      float phi_max,
      float L,
      const std::vector<float>& hitch_lengths)
      : Robot()
      , L_(L)
      , hitch_lengths_(hitch_lengths)
  {
    geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));
    for (size_t i = 0; i < hitch_lengths.size(); ++i) {
      geom_.emplace_back(new fcl::Boxf(0.3, 0.25, 1.0));
    }

    auto space(std::make_shared<StateSpace>(hitch_lengths.size()));
    space->setPositionBounds(position_bounds);

    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, v_min);
    cbounds.setHigh(0, v_max);
    cbounds.setLow(1, phi_min);
    cbounds.setHigh(1, phi_max);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.1;
    is2D_ = true;
    max_speed_ = std::max(fabsf(v_min), fabsf(v_max));
  }

  virtual size_t numParts()
  {
    return hitch_lengths_.size() + 1;
  }

  void propagate(
      const ompl::base::State *start,
      const ompl::control::Control *control,
      const double duration,
      ompl::base::State *result) override
  {
    auto startTyped = start->as<StateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    auto resultTyped = result->as<StateSpace::StateType>();

    // use simple Euler integration
    float x = startTyped->getX();
    float y = startTyped->getY();
    std::vector<float> theta(hitch_lengths_.size() + 1);
    for (size_t i = 0; i < hitch_lengths_.size() + 1; ++i) {
      theta[i] = startTyped->getTheta(i);
    }
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      // TODO: loop over this in reverse, to avoid changing dependenies
      //       (for a single trailer it shouldn't matter)
      for (size_t i = 1; i < hitch_lengths_.size() + 1; ++i) {
        float theta_dot = ctrl[0] / hitch_lengths_[i-i];
        for (size_t j = 1; j < i; ++j) {
          theta_dot *= cosf(theta[j-1] - theta[j]);
        }
        theta_dot *= sinf(theta[i-1] - theta[i]);
        theta[i] += theta_dot * dt;
      }
      theta[0] += ctrl[0] / L_ * tanf(ctrl[1]) * dt;
      x += ctrl[0] * cosf(theta[0]) * dt;
      y += ctrl[0] * sinf(theta[0]) * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result
    resultTyped->setX(x);
    resultTyped->setY(y);
    for (size_t i = 0; i < hitch_lengths_.size() + 1; ++i) {
      resultTyped->setTheta(i, theta[i]);
    }
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t part) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;

    if (part == 0) {
      result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
      float yaw = stateTyped->getTheta(0);
      result.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    } else if (part == 1) {
      fcl::Vector3f pos0(stateTyped->getX(), stateTyped->getY(), 0);
      float theta1 = stateTyped->getTheta(1);
      fcl::Vector3f delta(cosf(theta1), sinf(theta1), 0);
      result = Eigen::Translation<float, 3>(pos0 - delta * hitch_lengths_[0]);
      result.rotate(Eigen::AngleAxisf(theta1, Eigen::Vector3f::UnitZ()));
    } else {
      assert(false);
    }
    return result;
  }

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      // 0 means theta of pulling car
      double getTheta(size_t trailer) const
      {
        return as<ob::SO2StateSpace::StateType>(1+trailer)->value;
      }

      void setX(double x)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setTheta(size_t trailer, double yaw)
      {
        auto s = as<ob::SO2StateSpace::StateType>(1+trailer);
        s->value = yaw;

        // Normalize orientation
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(s);
      }
    };

    StateSpace(size_t numTrailers)
    {
      setName("CarWithTrailerSO" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 1;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);  // position
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);          // orientation
      for (size_t i = 0; i < numTrailers; ++i) {
        addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);        // orientation
      }
      lock();
    }

    ~StateSpace() override = default;

    bool satisfiesBounds(const ob::State *state) const override
    {
      auto stateTyped = state->as<StateSpace::StateType>();
      double th0 = stateTyped->getTheta(0);
      double th1 = stateTyped->getTheta(1);
      double delta = th1 - th0;
      double angular_change = atan2(sin(delta), cos(delta));
      if (fabs(angular_change) > M_PI / 4) {
        return false;
      }
      return ob::CompoundStateSpace::satisfiesBounds(state);
    }

    void enforceBounds(ob::State *state) const override
    {
      auto stateTyped = state->as<StateSpace::StateType>();
      double th0 = stateTyped->getTheta(0);
      double th1 = stateTyped->getTheta(1);
      double delta = th1 - th0;
      double angular_change = atan2(sin(delta), cos(delta));
      if (fabs(angular_change) > M_PI / 4) {
        stateTyped->setTheta(1, th0 + angular_change / fabs(angular_change) * M_PI/4);
      }
      ob::CompoundStateSpace::enforceBounds(state);
    }

    void setPositionBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const
    {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    ob::State *allocState() const override
    {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override
    {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections() override
    {
      class DefaultProjection : public ob::ProjectionEvaluator
      {
      public:
        DefaultProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
          return 2;
        }

        void defaultCellSizes() override
        {
          cellSizes_.resize(2);
          bounds_ = space_->as<ob::SE2StateSpace>()->getBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<ob::SE2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 2);
        }
      };

      registerDefaultProjection(std::make_shared<DefaultProjection>(this));
    }
  };

protected:
  float L_;
  std::vector<float> hitch_lengths_;

};

////////////////////////////////////////////////////////////////////////////////////////////////

// See also https://github.com/ompl/omplapp/blob/main/src/omplapp/apps/QuadrotorPlanning.cpp
// In the ompl.app example, the control seems to be force + moments
// rather than raw motor forces

class RobotQuadrotor : public Robot
{
public:
  RobotQuadrotor(
      const ompl::base::RealVectorBounds &position_bounds
      )
      : Robot()
  {
    // Parameters based on Bitcraze Crazyflie 2.0
    mass_ = 0.034;                  // kg
    const float arm_length = 0.046; // m
    const float arm = 0.707106781 * arm_length;
    const float t2t = 0.006; // thrust-to-torque ratio
    const float max_v = 2; // m/s
    const float max_omega = 4; //rad/s
    B0_ << 1, 1, 1, 1,
        -arm, -arm, arm, arm,
        -arm, arm, arm, -arm,
        -t2t, t2t, -t2t, t2t;
    g_ = 9.81; // gravity; not signed
    J_ << 16.571710e-6, 16.655602e-6, 29.261652e-6;
    inverseJ_ << 1 / J_(0), 1 / J_(1), 1 / J_(2);

    geom_.emplace_back(new fcl::Ellipsoidf(0.15, 0.15, 0.3)); // includes safety margins for downwash


    auto space(std::make_shared<StateSpace>());
    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vbounds(3);
    vbounds.setLow(-max_v); // m/s
    vbounds.setHigh(max_v); // m/s

    // vbounds.setLow(-0.5); // m/s
    // vbounds.setHigh(0.5); // m/s

    space->setVelocityBounds(vbounds);

    ob::RealVectorBounds wbounds(3);
    wbounds.setLow(-max_omega); // rad/s
    wbounds.setHigh(max_omega); // rad/s

    // wbounds.setLow(-2); // rad/s
    // wbounds.setHigh(2); // rad/s
    // wbounds.setLow(2, -0.5); // no yaw movement
    // wbounds.setHigh(2, 0.5);
    space->setAngularVelocityBounds(wbounds);

    // create a control space
    // R^4: forces of the 4 rotors
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));
    cspace->setControlSamplerAllocator(
        [this](const oc::ControlSpace *space)
        {
          return std::make_shared<ControlSampler>(space, mass_ / 4.0 * g_, 2.0 / 1000.0 * g_);
        });

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4);
    // // version to control thrust + moments
    // // cbounds.setLow(0);
    // // cbounds.setHigh(0);
    // cbounds.setLow(1,-1e-4); // roll
    // cbounds.setHigh(1,1e-4);
    // cbounds.setLow(2,-1e-4); // pitch
    // cbounds.setHigh(2,1e-4);
    // cbounds.setLow(3,0); // yaw
    // cbounds.setHigh(3,0);
    // cbounds.setLow(0, 0.0 * mass_ * g_);//4.0 * 3.0 / 1000.0 * g_);
    // cbounds.setHigh(0, 1.4 * mass_ * g_);//4.0 * 12.0 / 1000.0 * g_);

    // version to control force
    cbounds.setLow(0);
    cbounds.setHigh(12.0 / 1000.0 * g_);
    // const float t2w = 3.0; // thrust-to-weight
    // cbounds.setHigh(t2w * mass_ / 4.0 * g_);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.01;
    is2D_ = false;
    max_speed_ = sqrtf(powf(vbounds.high[0], 2) + powf(vbounds.high[1], 2) + powf(vbounds.high[2], 2));
  }

  void propagate(
      const ompl::base::State *start,
      const ompl::control::Control *control,
      const double duration,
      ompl::base::State *result) override
  {
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // Version where control is motor forces
    Eigen::Vector4f force(ctrl[0], ctrl[1], ctrl[2], ctrl[3]);
    auto eta = B0_ * force;
    Eigen::Vector3f f_u(0,0, eta(0));
    Eigen::Vector3f tau_u(eta(1), eta(2), eta(3));

    // // version where control is force + moments
    // Eigen::Vector3f f_u(0,0, ctrl[0]);
    // Eigen::Vector3f tau_u(ctrl[1], ctrl[2], ctrl[3]);

    // use simple Euler integration
    auto startTyped = start->as<StateSpace::StateType>();
    Eigen::Vector3f pos(startTyped->getX(), startTyped->getY(), startTyped->getZ());
    Eigen::Quaternionf q(startTyped->rotation().w, startTyped->rotation().x, startTyped->rotation().y, startTyped->rotation().z);
    Eigen::Vector3f vel(startTyped->velocity()[0], startTyped->velocity()[1], startTyped->velocity()[2]);
    Eigen::Vector3f omega(startTyped->angularVelocity()[0], startTyped->angularVelocity()[1], startTyped->angularVelocity()[2]);

    const Eigen::Vector3f gravity(0,0,-g_);
    
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      pos += vel * dt;

      vel += (gravity + q._transformVector(f_u) / mass_) * dt;

      q = qintegrate(q, omega, dt).normalized();

      omega += inverseJ_.cwiseProduct(J_.cwiseProduct(omega).cross(omega) + tau_u) * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result
    auto resultTyped = result->as<StateSpace::StateType>();
    resultTyped->setX(pos(0));
    resultTyped->setY(pos(1));
    resultTyped->setZ(pos(2));

    resultTyped->rotation().w = q.w();
    resultTyped->rotation().x = q.x();
    resultTyped->rotation().y = q.y();
    resultTyped->rotation().z = q.z();
    // Normalize orientation
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(&resultTyped->rotation());

    resultTyped->velocity()[0] = vel(0);
    resultTyped->velocity()[1] = vel(1);
    resultTyped->velocity()[2] = vel(2);

    resultTyped->angularVelocity()[0] = omega(0);
    resultTyped->angularVelocity()[1] = omega(1);
    resultTyped->angularVelocity()[2] = omega(2);

    // std::cout << "=====================" <<  duration << std::endl;
    // si_->printState(startTyped);
    // si_->printControl(control);
    // si_->printState(resultTyped);
    // std::cout << si_->satisfiesBounds(resultTyped) << std::endl;
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;

    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), stateTyped->getZ()));
    result.rotate(Eigen::Quaternionf(stateTyped->rotation().w, stateTyped->rotation().x, stateTyped->rotation().y, stateTyped->rotation().z));
    return result;
  }

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
    stateTyped->setZ(position(2));
  }

protected:
  // based on https://ignitionrobotics.org/api/math/6.9/Quaternion_8hh_source.html,
  // which is based on http://physicsforgames.blogspot.com/2010/02/quaternions.html
  Eigen::Quaternionf qintegrate(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt)
  {
    Eigen::Quaternionf deltaQ;
    auto theta = omega * dt * 0.5;
    float thetaMagSq = theta.squaredNorm();
    float s;
    if (thetaMagSq * thetaMagSq / 24.0 < std::numeric_limits<float>::min()) {
      deltaQ.w() = 1.0 - thetaMagSq / 2.0;
      s = 1.0 - thetaMagSq / 6.0;
    } else {
      float thetaMag = sqrtf(thetaMagSq);
      deltaQ.w() = cosf(thetaMag);
      s = sinf(thetaMag) / thetaMag;
    }
    deltaQ.x() = theta.x() * s;
    deltaQ.y() = theta.y() * s;
    deltaQ.z() = theta.z() * s;
    return deltaQ * q;
  }

protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
      }

      double getY() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
      }

      double getZ() const
      {
        return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
      }

      void setX(double x)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setZ(double z)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z;
      }

      // const double *position() const
      // {
      //   return as<ob::SE3StateSpace::StateType>(0)->as<ob::RealVectorStateSpace::StateType>(0)->values;
      // }

      // double *position()
      // {
      //   return as<ob::SE3StateSpace::StateType>(0)->as<ob::RealVectorStateSpace::StateType>(0)->values;
      // }

      const ob::SO3StateSpace::StateType &rotation() const
      {
        return *as<ob::SO3StateSpace::StateType>(1);
      }

      ob::SO3StateSpace::StateType &rotation()
      {
        return *as<ob::SO3StateSpace::StateType>(1);
      }

      const double* velocity() const
      {
        return as<ob::RealVectorStateSpace::StateType>(2)->values;
      }

      double *velocity()
      {
        return as<ob::RealVectorStateSpace::StateType>(2)->values;
      }

      const double *angularVelocity() const
      {
        return as<ob::RealVectorStateSpace::StateType>(3)->values;
      }

      double *angularVelocity()
      {
        return as<ob::RealVectorStateSpace::StateType>(3)->values;
      }
    };

    StateSpace()
    {
      setName("Quadrotor" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 2;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 1.0);      // position
      addSubspace(std::make_shared<ob::SO3StateSpace>(), 1.0);              // orientation
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 0.1); // velocity
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 0.05); // angular velocity
      lock();
    }

    ~StateSpace() override = default;

    // bool satisfiesBounds(const ob::State *state) const override
    // {
    //   bool result = ob::CompoundStateSpace::satisfiesBounds(state);
    //   if (!result) {
    //     return false;
    //   }

    //   auto stateTyped = state->as<StateType>();
    //   Eigen::Vector3f up(0,0,1);
    //   Eigen::Quaternionf q(stateTyped->rotation().w, stateTyped->rotation().x, stateTyped->rotation().y, stateTyped->rotation().z);
    //   const auto& up_transformed = q._transformVector(up);
    //   float angle = acosf(up.dot(up_transformed));
    //   return fabs(angle) < M_PI / 6;
    // }

    void setPositionBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    void setVelocityBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
    }

    void setAngularVelocityBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(3)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getPositionBounds() const
    {
      return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    ob::State *allocState() const override
    {
      auto *state = new StateType();
      allocStateComponents(state);
      return state;
    }

    void freeState(ob::State *state) const override
    {
      CompoundStateSpace::freeState(state);
    }

    void registerProjections()
    {
      class SE3DefaultProjection : public ob::ProjectionEvaluator
      {
      public:
        SE3DefaultProjection(const StateSpace *space) : ob::ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
          return 3;
        }

        void defaultCellSizes() override
        {
          cellSizes_.resize(3);
          bounds_ = space_->as<StateSpace>()->getPositionBounds();
          cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
          cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
          projection = Eigen::Map<const Eigen::VectorXd>(
              state->as<StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3);
        }
      };

      registerDefaultProjection(std::make_shared<SE3DefaultProjection>(this));
    }
  };

protected:
  class ControlSampler : public oc::ControlSampler
  {
  public:
    ControlSampler(const oc::ControlSpace *space, double mean, double stddev) 
      : oc::ControlSampler(space)
      , mean_(mean)
      , stddev_(stddev)
    {
    }

    void sample(oc::Control *control) override
    {
      const unsigned int dim = space_->getDimension();
      const ob::RealVectorBounds &bounds = static_cast<const oc::RealVectorControlSpace *>(space_)->getBounds();

      auto *rcontrol = static_cast<oc::RealVectorControlSpace::ControlType *>(control);
      for (unsigned int i = 0; i < dim; ++i) {
        // rcontrol->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
        rcontrol->values[i] = clamp(rng_.gaussian(mean_, stddev_), bounds.low[i], bounds.high[i]);
      }
    }

  protected:
    double clamp(double val, double min, double max)
    {
      if (val < min) return min;
      if (val > max) return max;
      return val; 
    }

  protected:
    double mean_;
    double stddev_;

  };

protected:
  float mass_;
  float g_;
  Eigen::Matrix<float, 4, 4> B0_;
  Eigen::Vector3f J_; // only diagonal entries 
  Eigen::Vector3f inverseJ_; // only diagonal entries 
};

////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Robot> create_robot(
  const std::string &robotType,
  const ob::RealVectorBounds &positionBounds)
{
  std::shared_ptr<Robot> robot;
  if (robotType == "unicycle_first_order_0")
  {
    robot.reset(new RobotUnicycleFirstOrder(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.5 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "unicycle_first_order_1")
  {
    // 2D plane-like (with a minimum positive speed)
    robot.reset(new RobotUnicycleFirstOrder(
        positionBounds,
        /*v_min*/ 0.25 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.5 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "unicycle_first_order_2")
  {
    // only forward movement, with easier right turns
    robot.reset(new RobotUnicycleFirstOrder(
        positionBounds,
        /*v_min*/ 0.25 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.25 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "unicycle_second_order_0")
  {
    robot.reset(new RobotUnicycleSecondOrder(
        positionBounds,
        /*v_limit*/ 0.5 /*m/s*/,
        /*w_limit*/ 0.5 /*rad/s*/,
        /*a_limit*/ 0.25 /*m/s^2*/,
        /*w_dot_limit*/ 0.25 /*rad/s^2*/
        ));
  }
  else if (robotType == "car_first_order_0")
  {
    robot.reset(new RobotCarFirstOrderWithTrailers(
        positionBounds,
        /*v_min*/ -0.1 /*m/s*/,
        /*v_max*/ 0.5 /*m/s*/,
        /*phi_min*/ -M_PI/3.0f /*rad*/,
        /*phi_max*/ M_PI/3.0f /*rad*/,
        /*L*/ 0.25 /*m*/,
        /*hitch_lengths*/ {} /*m*/
        ));
  }
  else if (robotType == "car_first_order_with_1_trailers_0")
  {
    robot.reset(new RobotCarFirstOrderWithTrailers(
        positionBounds,
        /*v_min*/ -0.1 /*m/s*/,
        /*v_max*/ 0.5 /*m/s*/,
        /*phi_min*/ -M_PI/3.0f /*rad*/,
        /*phi_max*/ M_PI/3.0f /*rad*/,
        /*L*/ 0.25 /*m*/,
        /*hitch_lengths*/ {0.5} /*m*/
        ));
  }
  else if (robotType == "quadrotor_0")
  {
    robot.reset(new RobotQuadrotor(
        positionBounds));
  }
  else
  {
    throw std::runtime_error("Unknown robot type!");
  }
  return robot;
}