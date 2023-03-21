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
    float x_1 = startTyped->getX(0);
    float y_1 = startTyped->getY(0);
    float yaw_1 = startTyped->getYaw(0);

    float x_2 = startTyped->getX(1);
    float y_2 = startTyped->getY(1);
    float yaw_2 = startTyped->getYaw(1);

    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      yaw_1 += ctrl[1] * dt;
      x_1 += ctrl[0] * cosf(yaw_1) * dt;
      y_1 += ctrl[0] * sinf(yaw_1) * dt;

      yaw_2 += ctrl[3] * dt;
      x_2 += ctrl[2] * cosf(yaw_2) * dt;
      y_2 += ctrl[2] * sinf(yaw_2) * dt;

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->setX(0, x_1);
    resultTyped->setY(0, y_1);
    resultTyped->setYaw(0, yaw_1);

    resultTyped->setX(1, x_2);
    resultTyped->setY(1, y_2);
    resultTyped->setYaw(1, yaw_2);

    // Normalize orientation
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(resultTyped->as<ob::SO2StateSpace::StateType>(1));
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t part) override
  {
    auto stateTyped = state->as<ob::SE2StateSpace::StateType>();

    fcl::Transform3f result;
    if (part == 0) {
      result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(0), stateTyped->getY(0), 0));
      float yaw_1 = stateTyped->getYaw(0);
      result.rotate(Eigen::AngleAxisf(yaw_1, Eigen::Vector3f::UnitZ()));
    } else if (part == 1) {
      result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(1), stateTyped->getY(1), 0));
      float yaw_2 = stateTyped->getYaw(1);
      result.rotate(Eigen::AngleAxisf(yaw_2, Eigen::Vector3f::UnitZ()));
    } else {
      assert(false);
    }
    return result;
  }

  virtual void setPosition(ompl::base::State *state, const fcl::Vector3f position) override
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

    double getX(size_t t) const
    {
      return as<ob::RealVectorStateSpace::StateType>(t+1)->values[0];
    }

    double getY(size_t t) const
    {
      return as<ob::RealVectorStateSpace::StateType>(t+1)->values[1];
    }

    // 0 means theta of pulling car
    double getYaw(size_t t) const
    {
      return as<ob::SO2StateSpace::StateType>(t+1)->value;
    }

    void setX(size_t t, double x)
    {
      auto s = as<ob::RealVectorStateSpace::StateType>(t+1);
      s->values[0] = x;
    }

    void setY(size_t t, double y)
    {
      auto s = as<ob::RealVectorStateSpace::StateType>(t+1);
      s->values[1] = y;
    }
    void setYaw(size_t t, double yaw)
    {
      auto s = as<ob::SO2StateSpace::StateType>(t+1);
      s->value = yaw;

      // Normalize orientation
      ob::SO2StateSpace SO2;
      SO2.enforceBounds(s);
    }

    };
    StateSpace()
    {
      setName("FirstOrderUnicycle" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 1;
      addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);          // robot 1
      addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);          // robot 2
      
      lock();
    }

    ~StateSpace() override = default;
  };

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
  else
  {
    throw std::runtime_error("Unknown robot type!");
  }
  return robot;
}