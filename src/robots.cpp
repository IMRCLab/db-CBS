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
    const ompl::base::RealVectorBounds& position_bounds,
    float v_min, float v_max,
    float w_min, float w_max)
  {
    geom_.emplace_back(new fcl::Boxf(0.5, 0.25, 1.0));
    geom_.emplace_back(new fcl::Boxf(0.3, 0.25, 1.0));


    auto space(std::make_shared<StateSpace>());
    space->setPositionBounds(0,position_bounds);
    space->setPositionBounds(1,position_bounds);
    
    // create a control space
    // R^1: turning speed
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 4));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4);
    cbounds.setLow(0, v_min);
    cbounds.setHigh(0, v_max);
    cbounds.setLow(1, w_min);
    cbounds.setHigh(1, w_max);

    cbounds.setLow(2, v_min);
    cbounds.setHigh(2, v_max);
    cbounds.setLow(3, w_min);
    cbounds.setHigh(3, w_max);
  
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

    // use simple Euler integration
    float x1 = startTyped->getX(0);
    float y1 = startTyped->getY(0);
    float yaw1 = startTyped->getYaw(0);
    float x2 = startTyped->getX(1);  
    float y2 = startTyped->getY(1);
    float yaw2 = startTyped->getYaw(1);

    float remaining_time = duration;
    do
    {
      float dt = std::min(remaining_time, dt_);

      yaw1 += ctrl[1] * dt;
      x1 += ctrl[0] * cosf(yaw1) * dt;
      y1 += ctrl[0] * sinf(yaw1) * dt;

      yaw2 += ctrl[3] * dt;
      x2 += ctrl[2] * cosf(yaw2) * dt;
      y2 += ctrl[2] * sinf(yaw2) * dt;


      remaining_time -= dt;
    } while (remaining_time >= dt_);

    // update result

    resultTyped->setX(0,x1);
    resultTyped->setY(0,y1);
    resultTyped->setYaw(0,yaw1);


    resultTyped->setX(1,x2);
    resultTyped->setY(1,y2);
    resultTyped->setYaw(1,yaw2);

    // Normalize orientation
    // ob::SO2StateSpace SO2;
    // SO2.enforceBounds(resultTyped->as<ob::SO2StateSpace::StateType>(1));
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
  virtual void setPosition(
      ompl::base::State *state,
      const fcl::Vector3f position) override
  {
    auto stateTyped = state->as<StateSpace::StateType>();

    stateTyped->setX(0,position(0));
    stateTyped->setY(2,position(1));
  }

protected:
  class StateSpace : public ob::CompoundStateSpace
  {
  public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
      StateType() = default;

      double getX(int i) const
      {
        return as<ob::RealVectorStateSpace::StateType>(2*i)->values[0];
      }
 
      double getY(int i) const
      {
        return as<ob::RealVectorStateSpace::StateType>(2*i)->values[1];
      }

      double getYaw(int i) const
      {
        return as<ob::SO2StateSpace::StateType>(1+2*i)->value;
      }

      void setX(int i, double x)
      {
        auto s = as<ob::RealVectorStateSpace::StateType>(2*i);
        s->values[0] = x;
      }

      void setY(int i, double y)
      {
        auto s = as<ob::RealVectorStateSpace::StateType>(2*i); 
        s->values[1] = y;
      }

      void setYaw(int i, double yaw)
      {
        auto s = as<ob::SO2StateSpace::StateType>(1+2*i);
        s->value = yaw;
      }
    };

    StateSpace()
    {
      setName("TwoUnicycles" + getName());
      type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);  // position1
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);          // orientation1
      addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);  // position2
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);          // orientation2



      lock();
    }

    ~StateSpace() override = default;

    void setPositionBounds(int i,const ob::RealVectorBounds &bounds)
    {
      as<ob::RealVectorStateSpace>(2*i)->setBounds(bounds);
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
};
////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Robot> create_robot(
  const std::string &robotType,
  const ob::RealVectorBounds &positionBounds)
{
  std::shared_ptr<Robot> robot;
  if (robotType == "unicycle_first_order_0")
  {
    robot.reset(new MultiRobotUnicycleFirstOrder(
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