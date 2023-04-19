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
};

////////////////////////////////////////////////////////////////////////////////////////////////

// class RobotDoubleIntegrator2D : public Robot
// {
// public:
//   RobotDoubleIntegrator2D(
//       const ompl::base::RealVectorBounds &position_bounds,
//       float v_limit,      // max velocity in m/s
//       float w_limit,      // max angular velocity in rad/s
//       float a_limit,      // max accelleration in m/s^2
//       float w_dot_limit) // max angular acceleration in rad/s^2
//   {
//     geom_.emplace_back(new fcl::Spheref(0.5));

//     auto space(std::make_shared<StateSpace>());
//     space->setPositionBounds(position_bounds);

//     ob::RealVectorBounds vel_bounds(1);
//     vel_bounds.setLow(-v_limit);
//     vel_bounds.setHigh(v_limit);
//     space->setVelocityBounds(vel_bounds);

//     ob::RealVectorBounds w_bounds(1);
//     w_bounds.setLow(-w_limit);
//     w_bounds.setHigh(w_limit);
//     space->setAngularVelocityBounds(w_bounds);

//     // create a control space
//     // R^1: turning speed
//     auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

//     // set the bounds for the control space
//     ob::RealVectorBounds cbounds(2);
//     cbounds.setLow(0, -a_limit);
//     cbounds.setHigh(0, a_limit);
//     cbounds.setLow(1, -w_dot_limit);
//     cbounds.setHigh(1, w_dot_limit);

//     cspace->setBounds(cbounds);

//     // construct an instance of  space information from this control space
//     si_ = std::make_shared<oc::SpaceInformation>(space, cspace);

//     dt_ = 0.1;
//     is2D_ = true;
//     max_speed_ = v_limit;
//   }

//   void propagate(
//       const ompl::base::State *start,
//       const ompl::control::Control *control,
//       const double duration,
//       ompl::base::State *result) override
//   {
//     auto startTyped = start->as<StateSpace::StateType>();
//     const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

//     auto resultTyped = result->as<StateSpace::StateType>();

//     // use simple Euler integration
//     float x = startTyped->getX();
//     float y = startTyped->getY();
//     float v = startTyped->getVelocity();
//     float w = startTyped->getAngularVelocity();
//     float remaining_time = duration;
//     do
//     {
//       float dt = std::min(remaining_time, dt_);

//       w += ctrl[0] * dt;
//       v += w * dt;
//       x += v * dt;
//       y += v * dt;

//       remaining_time -= dt;
//     } while (remaining_time >= dt_);

//     // update result

//     resultTyped->setX(x);
//     resultTyped->setY(y);
//     resultTyped->setVelocity(v);
//     resultTyped->setAngularVelocity(w);

//   }

//   virtual fcl::Transform3f getTransform(
//       const ompl::base::State *state,
//       size_t /*part*/) override
//   {
//     auto stateTyped = state->as<StateSpace::StateType>();

//     fcl::Transform3f result;
//     result = Eigen::Translation<float, 3>(fcl::Vector3f(stateTyped->getX(), stateTyped->getY(), 0));
//     return result;
//   }

// protected:
//   class StateSpace : public ob::CompoundStateSpace
//   {
//   public:
//     class StateType : public ob::CompoundStateSpace::StateType
//     {
//     public:
//       StateType() = default;

//       double getX() const
//       {
//         auto sub = as<ob::SE2StateSpace::StateType>(0);
//         return sub->getX();
//       }
//       double getY() const
//       {
//         auto sub = as<ob::SE2StateSpace::StateType>(0);
//         return sub->getY();
//       }
//       double getVelocity() const
//       {
//         return as<ob::RealVectorStateSpace::StateType>(1)->values[0];
//       }

//       double getAngularVelocity() const
//       {
//         return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
//       }
//       void setX(double x)
//       {
//         auto sub = as<ob::SE2StateSpace::StateType>(0);
//         sub->setX(x);

//       }

//       void setY(double y)
//       {
//         auto sub = as<ob::SE2StateSpace::StateType>(0);
//         sub->setY(y);

//       }
//       void setVelocity(double velocity)
//       {
//         as<ob::RealVectorStateSpace::StateType>(1)->values[0] = velocity;
//       }

//       void setAngularVelocity(double angularVelocity)
//       {
//         as<ob::RealVectorStateSpace::StateType>(2)->values[0] = angularVelocity;
//       }
//     };

//     StateSpace()
//     {
//       setName("RobotDoubleIntegrator2D" + getName());
//       type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
//       addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);  
//       addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 0.25); // velocity
//       addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 0.25); // angular velocity
//       lock();
//     }

//     ~StateSpace() override = default;

//     void setPositionBounds(const ob::RealVectorBounds &bounds)
//     {
//       as<ob::SE2StateSpace>(0)->setBounds(bounds);
//     }

//     const ob::RealVectorBounds &getPositionBounds() const
//     {
//       return as<ob::SE2StateSpace>(0)->getBounds();

//     }

//     void setVelocityBounds(const ob::RealVectorBounds &bounds)
//     {
//       as<ob::RealVectorStateSpace>(1)->setBounds(bounds);
//     }

//     const ob::RealVectorBounds &getVelocityBounds() const
//     {
//       return as<ob::RealVectorStateSpace>(1)->getBounds();
//     }

//     void setAngularVelocityBounds(const ob::RealVectorBounds &bounds)
//     {
//       as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
//     }

//     const ob::RealVectorBounds &getAngularVelocityBounds() const
//     {
//       return as<ob::RealVectorStateSpace>(2)->getBounds();
//     }

//     ob::State *allocState() const override
//     {
//       auto *state = new StateType();
//       allocStateComponents(state);
//       return state;
//     }

//     void freeState(ob::State *state) const override
//     {
//       CompoundStateSpace::freeState(state);
//     }
//   };
// };

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
};
// ////////////////////////////////////////////////////////////////////////////////////////////////
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
  if (robotType == "unicycle_first_order")
  {
    robot.reset(new RobotUnicycleFirstOrder(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/,
        /*w_min*/ -0.5 /*rad/s*/,
        /*w_max*/ 0.5 /*rad/s*/));
  }
  else if (robotType == "car_first_order")
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
  else if (robotType == "single_integrator")
  {
    robot.reset(new RobotSingleIntegrator2D(
        positionBounds,
        /*v_min*/ -0.5 /* m/s*/,
        /*v_max*/ 0.5 /* m/s*/
        ));
  }
  // else if (robotType == "double_integrator")
  // {
    // robot.reset(new RobotDoubleIntegrator2D(
        // positionBounds,
        // /*v_limit*/ 0.5 /*m/s*/,
        // /*w_limit*/ 0.5 /*rad/s*/,
        // /*a_limit*/ 0.25 /*m/s^2*/,
        // /*w_dot_limit*/ 0.25 /*rad/s^2*/
        // ));
  // }
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
