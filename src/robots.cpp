#include "robots.h"

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class MultiRobotUnicycleFirstOrder : public Robot
{
public:
 MultiRobotUnicycleFirstOrder(
    size_t num_robots,
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, float v_max,
    float w_min, float w_max) : num_robots_(num_robots)
  {
    for (size_t i = 0; i < num_robots_; ++i) {
      geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));
    }
    
    auto space(std::make_shared<StateSpace>(num_robots_));
    for (size_t i = 0; i < num_robots_; ++i) {
      space->setPositionBounds(i,position_bounds);  
    }      
    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2*num_robots_));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2*num_robots_);
    for (size_t i = 0; i < num_robots_; ++i) {
      cbounds.setLow(2*i, v_min);
      cbounds.setHigh(2*i, v_max);
      cbounds.setLow(2*i+1, w_min);
      cbounds.setHigh(2*i+1, w_max);
    }
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

    std::vector<float> x(num_robots_);
    std::vector<float> y(num_robots_);
    std::vector<float> yaw(num_robots_);
    // use simple Euler integration
    for (size_t i = 0; i < num_robots_; ++i) {
        x[i] = startTyped->getX(i);
        y[i] = startTyped->getY(i);
        yaw[i] = startTyped->getYaw(i);
      }
   
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      for (size_t i = 0; i < num_robots_; ++i) {
        yaw[i] += ctrl[2*i+1] * dt;
        x[i] += ctrl[2*i] * cosf(yaw[i]) * dt;
        y[i] += ctrl[2*i] * sinf(yaw[i]) * dt;  
      }
      
      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    for (size_t i = 0; i < num_robots_; ++i) {
      resultTyped->setX(i, x[i]);
      resultTyped->setY(i, y[i]);
      resultTyped->setYaw(i, yaw[i]);
    }
    
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t part) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;
    if (part == 0) {
      result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(0), stateTyped->getY(0), 0));
      float yaw1 = stateTyped->getYaw(0);
      result.rotate(Eigen::AngleAxisf(yaw1, Eigen::Vector3f::UnitZ()));
    } else if (part == 1) {
      result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(1), stateTyped->getY(1), 0));
      float yaw2 = stateTyped->getYaw(1);
      result.rotate(Eigen::AngleAxisf(yaw2, Eigen::Vector3f::UnitZ()));
      
    } else {
      assert(false);
    }
    return result;
  }
protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX(size_t i) const
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getX();

      }
 
      double getY(size_t i) const
      {

        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getY();

      }

      double getYaw(size_t i) const
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getYaw();

      }

      void setX(size_t i, double x)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setX(x);
        
      }

      void setY(size_t i, double y)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setY(y);

      }

      void setYaw(size_t i, double yaw)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setYaw(yaw);

      }
    };

    StateSpace(size_t numRobots)
    {
      setName("MultiRobotUnicycle" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    
      for (size_t i = 0; i < numRobots; ++i) {
        addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);  
      }
      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(size_t i,const ob::RealVectorBounds &bounds)
    {
      as<ob::SE2StateSpace>(i)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getBounds(size_t i) const
    {
      return as<ob::SE2StateSpace>(i)->getBounds();
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
  size_t num_robots_;
};
// ////////////////////////////////////////////////////////////////////////////////////////////////
class MultiRobotCarFirstOrder : public Robot
{
public:
  MultiRobotCarFirstOrder(
      size_t num_robots,
      const ompl::base::RealVectorBounds &position_bounds,
      float v_min,
      float v_max,
      float phi_min,
      float phi_max,
      float L): L_(L), num_robots_(num_robots)
  {
    for (size_t i = 0; i < num_robots_; ++i) {
      geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));
    }
    auto space(std::make_shared<StateSpace>(num_robots_));
    for (size_t i = 0; i < num_robots_; ++i) {
      space->setPositionBounds(i,position_bounds);  
    } 

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2*num_robots_));
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2*num_robots_);
    for (size_t i = 0; i < num_robots_; ++i) {
      cbounds.setLow(2*i, v_min);
      cbounds.setHigh(2*i, v_max);
      cbounds.setLow(2*i+1, phi_min);
      cbounds.setHigh(2*i+1, phi_max);
    }
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

    std::vector<float> x(num_robots_);
    std::vector<float> y(num_robots_);
    std::vector<float> theta(num_robots_);

    for (size_t i = 0; i < num_robots_; ++i) {
      x[i] = startTyped->getX(i);
      y[i] = startTyped->getY(i);
      theta[i] = startTyped->getTheta(i);
    }
    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);
      for (size_t i = 0; i < num_robots_; ++i) {
        theta[i] += ctrl[2*i] / L_ * tanf(ctrl[2*i+1]) * dt;
        x[i] += ctrl[2*i] * cosf(theta[i]) * dt;
        y[i] += ctrl[2*i] * sinf(theta[i]) * dt;  
      }

      remaining_time -= dt;
    } while (remaining_time >= dt_);

    for (size_t i = 0; i < num_robots_; ++i) {
      resultTyped->setX(i, x[i]);
      resultTyped->setY(i, y[i]);
      resultTyped->setTheta(i, theta[i]);
    }
  }

  virtual fcl::Transform3f getTransform(
      const ompl::base::State *state,
      size_t part) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    fcl::Transform3f result;
    result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(part), stateTyped->getY(part), 0));
    float theta = stateTyped->getTheta(part);
    result.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    return result;
  }
protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX(size_t i) const
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getX();

      }
 
      double getY(size_t i) const
      {

        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getY();

      }

      double getTheta(size_t i) const
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        return sub->getYaw();

      }

      void setX(size_t i, double x)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setX(x);
        
      }

      void setY(size_t i, double y)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setY(y);

      }

      void setTheta(size_t i, double yaw)
      {
        auto sub = as<ob::SE2StateSpace::StateType>(i);
        sub->setYaw(yaw);

      }
    };

    StateSpace(size_t numRobots)
    {
      setName("MultiRobotCarFirstOrder" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    
      for (size_t i = 0; i < numRobots; ++i) {
        addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);  
      }
      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(size_t i,const ob::RealVectorBounds &bounds)
    {
      as<ob::SE2StateSpace>(i)->setBounds(bounds);
    }

    const ob::RealVectorBounds &getBounds(size_t i) const
    {
      return as<ob::SE2StateSpace>(i)->getBounds();
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
  size_t num_robots_;
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
      robots_[i]->propagate((*startTyped)[i], (*controlTyped)[i], duration, (*resultTyped)[i]);
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

  virtual size_t numParts() override
  {
    return robots_.size();
  }

  virtual std::shared_ptr<fcl::CollisionGeometryf> getCollisionGeometry(size_t part) override
  {
    return robots_[part]->getCollisionGeometry(0);
  }

protected:
  std::vector<std::shared_ptr<Robot>> robots_;
};

// ////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Robot> create_robot(
  const std::string &robotType,
  const ob::RealVectorBounds &positionBounds)
{
  std::shared_ptr<Robot> robot;
  if (robotType == "unicycle_first_order_0")
  {
    robot.reset(new MultiRobotUnicycleFirstOrder(
        1, 
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.5 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "car_first_order_0")
  {
    robot.reset(new MultiRobotCarFirstOrder(
        1, 
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*phi_min*/ -M_PI/3.0f /*rad*/,
        /*phi_max*/ M_PI/3.0f /*rad*/,
        /*L*/ 0.25 /*m*/
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
