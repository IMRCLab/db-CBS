#include "robots.h"

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class RobotSingleIntegrator2D : public Robot
{
public:
  RobotSingleIntegrator2D(
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, float v_max)
  {
    geom_.emplace_back(new fcl::Spheref(0.1));

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setBounds(position_bounds);

    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(v_min);
    cbounds.setHigh(v_max);
    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.1;
    is2D_ = true;
    max_speed_ = std::sqrt(2) * std::max(fabsf(v_min), fabsf(v_max));
  }

  void propagate(
    const ompl::base::State *start,
    const ompl::control::Control *control,
    const double duration,
    ompl::base::State *result) override
  {
    auto startTyped = start->as<ob::RealVectorStateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    auto resultTyped = result->as<ob::RealVectorStateSpace::StateType>();

    // use simple Euler integration
    float x = startTyped->values[0];
    float y = startTyped->values[1];
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      x += ctrl[0] * dt;
      y += ctrl[1] * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->values[0]=x;
    resultTyped->values[1]=y;

  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<ob::RealVectorStateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->values[0], stateTyped->values[1], 0));
    return result;
  }
  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
  {
    auto stateTyped = state->as<ob::RealVectorStateSpace::StateType>();
    stateTyped->values[0]=position(0);
    stateTyped->values[1]=position(1);
  }
};
/////////////////////////////////////////////////////////////////////////////////////////

class RobotDoubleIntegrator2D : public Robot
{
public:
  RobotDoubleIntegrator2D(
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, 
    float v_max,
    float a_min,
    float a_max)
  {
    geom_.emplace_back(new fcl::Spheref(0.1));
    auto space(std::make_shared<StateSpace>());
    space->setPositionBounds(position_bounds);

    ob::RealVectorBounds vel_bounds(2);
    vel_bounds.setLow(v_min);
    vel_bounds.setHigh(v_max);
    space->setVelocityBounds(vel_bounds);

    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(a_min);
    cbounds.setHigh(a_max);
    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    dt_ = 0.1;
    is2D_ = true;
    max_speed_ = std::sqrt(2) * std::max(fabsf(v_min), fabsf(v_max));
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
    float v_x = startTyped->getVx();
    float v_y = startTyped->getVy();

    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      x += v_x * dt;
      y += v_y * dt;
      v_x += ctrl[0] * dt;
      v_y += ctrl[1] * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->setX(x);
    resultTyped->setY(y);
    resultTyped->setVx(v_x);
    resultTyped->setVy(v_y);

  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
    return result;
  }
  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
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

      double getVx() const
      {
        return as<ob::RealVectorStateSpace::StateType>(1)->values[0];
      }
      double getVy() const
      {
        return as<ob::RealVectorStateSpace::StateType>(1)->values[1];
      }

      void setX(double x)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
      }

      void setY(double y)
      {
        as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
      }

      void setVx(double vx)
      {
        as<ob::RealVectorStateSpace::StateType>(1)->values[0] = vx;
      }

      void setVy(double vy)
      {
        as<ob::RealVectorStateSpace::StateType>(1)->values[1] = vy;
      }
    };

    StateSpace()
    {
      setName("RobotDoubleIntegrator2D" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 1;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);  // position
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 0.25);  // velocity
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
      as<ob::RealVectorStateSpace>(1)->setBounds(bounds);
    }
    
    const ob::RealVectorBounds &getVelocityBounds() const 
    {
      return as<ob::RealVectorStateSpace>(1)->getBounds();
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

  };

};

//////////////////////////////////////////////////////////////////////////////////////////
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

      x += ctrl[0] * cosf(yaw) * dt;
      y += ctrl[0] * sinf(yaw) * dt;
      yaw += ctrl[1] * dt;

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

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }
};
//////////////////////////////////////////////////////////////////////////////////////////////////
class RobotUnicycleFirstOrderSphere : public Robot
{
public:
  RobotUnicycleFirstOrderSphere(
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, float v_max,
    float w_min, float w_max)
  {
    geom_.emplace_back(new fcl::Spheref(0.4));

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

      x += ctrl[0] * cosf(yaw) * dt;
      y += ctrl[0] * sinf(yaw) * dt;
      yaw += ctrl[1] * dt;

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

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();
    stateTyped->setX(position(0));
    stateTyped->setY(position(1));
  }
};
/////////////////////////////////////////////////////////////////////////////////////////////////
class RobotCarFirstOrder : public Robot
{
public:
  RobotCarFirstOrder(
      const ompl::base::RealVectorBounds &position_bounds,
      float v_min,
      float v_max,
      float phi_min,
      float phi_max,
      float L): L_(L)
  {
    
    geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));
    auto space(std::make_shared<StateSpace>());
    space->setPositionBounds(position_bounds);  

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

  void propagate(
      const ompl::base::State *start,
      const ompl::control::Control *control,
      const double duration,
      ompl::base::State *result) override
  {
    auto startTyped = start->as<StateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    auto resultTyped = result->as<StateSpace::StateType>();

    float x = startTyped->getX();
    float y = startTyped->getY();
    float theta = startTyped->getTheta();

    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);
      theta += ctrl[0] / L_ * tanf(ctrl[1]) * dt;
      x += ctrl[0] * cosf(theta) * dt;
      y += ctrl[0] * sinf(theta) * dt;  

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    resultTyped->setX(x);
    resultTyped->setY(y);
    resultTyped->setTheta(theta);

  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t /*part*/) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
    float theta = stateTyped->getTheta();
    result.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    return result;
  }

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
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
        auto sub = as<ob::SE2StateSpace::StateType>(0);
        return sub->getX();

      }
 
      double getY() const
      {

        auto sub = as<ob::SE2StateSpace::StateType>(0);
        return sub->getY();

      }

      double getTheta() const
      {
        auto sub = as<ob::SE2StateSpace::StateType>(0);
        return sub->getYaw();

      }

      void setX(double x)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(0);
        sub->setX(x);
        
      }

      void setY(double y)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(0);
        sub->setY(y);

      }

      void setTheta(double theta)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(0);
        sub->setYaw(theta);

      }
    };

    StateSpace()
    {
      setName("RobotCarFirstOrder" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);  
      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(const ob::RealVectorBounds &bounds)
    {
      as<ob::SE2StateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getBounds() const
    {
      return as<ob::SE2StateSpace>(0)->getBounds();
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

  };        
protected:
  float L_;
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
      x += ctrl[0] * cosf(theta[0]) * dt;
      y += ctrl[0] * sinf(theta[0]) * dt;
      theta[0] += ctrl[0] / L_ * tanf(ctrl[1]) * dt;

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

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position, size_t /*part*/) override
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

//////////////////////////////////////////////////////////////////////////////////////////////////
class MultiRobot : public Robot
{
public:
 MultiRobot(
    const std::vector<std::shared_ptr<Robot>>& robots)
    : robots_(robots)
  {

    // create state space
    auto space(std::make_shared<ob::CompoundStateSpace>());
    for (auto robot : robots) {
      auto rsi = robot->getSpaceInformation();
      auto rss = rsi->getStateSpace();
      space->addSubspace(rss, 1.0);
    }

    // create a control space
    auto cspace(std::make_shared<oc::CompoundControlSpace>(space));
    for (auto robot : robots) {
      auto rsi = robot->getSpaceInformation();
      auto rcs = rsi->getControlSpace();
      cspace->addSubspace(rcs);
    }

    // construct an instance of  space information from this control space
    si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

    // Compute dt, max_speed, and is2D
    std::set<float> dts;
    std::set<bool> is2Ds;
    std::set<float> max_speeds;
    for (auto robot : robots) {
      dts.insert(robot->dt());
      is2Ds.insert(robot->is2D());
      max_speeds.insert(robot->maxSpeed());
    }

    dt_ = *std::min_element(dts.begin(), dts.end());
    max_speed_ = *std::min_element(max_speeds.begin(), max_speeds.end());
    if (is2Ds.size() != 1) {
      throw std::runtime_error("is2D doesn't match!");
    }
  }

  void propagate(
      const ompl::base::State *start,
      const ompl::control::Control *control,
      const double duration,
      ompl::base::State *result) override
  {
    auto startTyped = start->as<ob::CompoundStateSpace::StateType>();
    auto controlTyped = const_cast<oc::CompoundControlSpace::ControlType*>(control->as<oc::CompoundControlSpace::ControlType>());
    auto resultTyped = result->as<ob::CompoundStateSpace::StateType>();

    for (size_t i = 0; i < robots_.size(); ++i) {
      // if (!goals_->isSatisfied(startTyped, i)) {
        robots_[i]->propagate(startTyped->components[i], (*controlTyped)[i], duration, (*resultTyped)[i]);
      // } else {
      //   // if we are at the goal for this robot, just copy the previous state
      //   auto csi = dynamic_cast<ompl::control::SpaceInformation*>(si_.get()); 
      //   auto csp = csi->getStateSpace()->as<ompl::base::CompoundStateSpace>();
      //   auto si_k = csp->getSubspace(i);

      //   // option 1
      //   si_k->copyState((*resultTyped)[i], (*startTyped)[i]);

      //   // option 2
      //   // std::vector<double> reals(si_k->getDimension(), nan(""));
      //   // si_k->copyFromReals((*resultTyped)[i], reals); 
      // }
    }
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t part) override
  {
    auto stateTyped = state->as<ob::CompoundStateSpace::StateType>();

    fcl::Transform3f result;
    result = robots_[part]->getTransform((*stateTyped)[part],0);
    return result;
  }

  virtual void setPosition(
      ompl::base::State *state, 
      const fcl::Vector3f position,
      size_t part) override
  {
    auto stateTyped = state->as<ob::CompoundStateSpace::StateType>();
    robots_[part]->setPosition((*stateTyped)[part],position,0);
  }

  virtual size_t numParts() override
  {
    return robots_.size();
  }

  virtual std::shared_ptr<fcl::CollisionGeometryf> getCollisionGeometry(size_t part) override
  {
    return robots_[part]->getCollisionGeometry(0);
  }

  void setGoals(std::shared_ptr<MultiRobotGoalState> goals)
  {
    goals_ = goals;
  }

protected:
  std::vector<std::shared_ptr<Robot>> robots_;
  std::shared_ptr<MultiRobotGoalState> goals_;
};

// ////////////////////////////////////////////////////////////////////////////////////////////////

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
  else if (robotType == "unicycle_first_order_0_sphere")
  {
    robot.reset(new RobotUnicycleFirstOrderSphere(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.5 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "car_first_order_0")
  {
    robot.reset(new RobotCarFirstOrder(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*phi_min*/ -M_PI/3.0f /*rad*/,
        /*phi_max*/ M_PI/3.0f /*rad*/,
        /*L*/ 0.25 /*m*/
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
  else if (robotType == "single_integrator_0")
  {
    robot.reset(new RobotSingleIntegrator2D(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/
        ));
  }
  else if (robotType == "double_integrator_0")
  {
    robot.reset(new RobotDoubleIntegrator2D(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*a_min*/ -2.0 /* m/s^2*/,
        /*a_max*/ 2.0 /* m/s^2*/
        ));
  }
  
  else
  {
    throw std::runtime_error("Unknown robot type!");
  }
  return robot;
}
std::shared_ptr<Robot> create_joint_robot(
  std::vector<std::shared_ptr<Robot>> robots)
{
  std::shared_ptr<Robot> robot;
  robot.reset(new MultiRobot(robots));
  return robot;
}

// HACK for multi-robot
void setMultiRobotGoals(
  std::shared_ptr<Robot> robot,
  std::shared_ptr<MultiRobotGoalState> goals)
{
  MultiRobot* r = dynamic_cast<MultiRobot*>(robot.get());
  r->setGoals(goals);
}