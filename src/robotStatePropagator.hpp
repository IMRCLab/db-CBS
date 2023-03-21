#pragma once

#include "ompl/control/StatePropagator.h"
#include "robots.h"

class RobotStatePropagator : public ompl::control::StatePropagator
{
public:
    RobotStatePropagator(
        const ompl::control::SpaceInformationPtr& si,
        std::shared_ptr<Robot> robot)
        : ompl::control::StatePropagator(si)
        , robot_(robot)
    {
    }

    ~RobotStatePropagator() override = default;

    void propagate(
        const ompl::base::State *state,
        const ompl::control::Control *control,
        double duration,
        ompl::base::State *result) const override
    {
        // propagate state
        robot_->propagate(state, control, duration, result);
    }

    bool canPropagateBackward() const override
    {
        return false;
    }

    bool canSteer() const override
    {
        return false;
    }

protected:
    std::shared_ptr<Robot> robot_;
};
