/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/planners/rrt/RRT.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <unordered_map>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>

#include "robots.h"
#include "robotStatePropagator.hpp"
#include "fclStateValidityChecker.hpp"


namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;


/* 
When performing Multi-Robot Motion Planning, it is sometimes the case that 
robots are treated as "dynamic obstacles" by other robots (e.g. Prioritized Planning and Kinodynamic Conflict-Based Search). 
Thus, we have extended OMPL to account for dynamic obstacles. To do so, one must implement an additional method called 
``areStatesValid" from the StateValidityChecker class. The method should return true if state1 and state2 are not in collision.
state2 is a pair consisting of the actual state and the SpaceInformation from which the state lives. The SpaceInformation object is included 
so that heterogeneous robots can be properly accounted for (see the example below). Keep in mind that 
time-dependence is handled generically within OMPL. Please see ob::StateValidityChecker::isValid(const State *state, const double time) 
for details.
*/
class myDemoStateValidityChecker: public ob::StateValidityChecker
{
public:
    myDemoStateValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
    {
    }

    // Answers the question: is the robot described by `si_` at `state` valid?
    bool isValid(const ompl::base::State *state) const override
    {
        if (!si_->satisfiesBounds(state)){
            return false;
        }
        return true;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        /* We assume robots are all disks of varying size (see myDemoStateValidityChecker::radii_) */
        const auto *robot1_state = state1->as<ob::RealVectorStateSpace::StateType>();
        const auto *robot2_state = state2.second->as<ob::RealVectorStateSpace::StateType>();

        // one must code required logic to figure out if robot1 at state1 collides with robot2 at state2
        // this example assumes all robots are disks in R^2 of varying sizes.
        double rad1 = radii_[0] + 0.1; // safety criterion
        double rad2 = radii_[1] + 0.1; // safety criterion

        float x1 = robot1_state->values[0];
        float y1 = robot1_state->values[1];

        float x2 = robot2_state->values[0];
        float y2 = robot2_state->values[1];
        double dist = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
        return dist > (rad1 + rad2);
    }
private:
    std::vector<double> radii_{0.5,0.5};
};

/*
Initially, K-CBS is a fully decoupled algorithm. However, in 
certain scenarios, coupling sub-sets of robots are neccesssary to find plans. Thus, 
K-CBS couples certain robots, as needed, while planning. The user must create this behavior based on problem specific elements. 
Thus, the user may implement a SystemMerger, which returns a new omrc::SpaceInformation and omrb::ProblemDefintion pair after 
coupling individuals index1 and index2 together.K-CBS calls the merge function, as needed, to find solutions. In the first case, 
K-CBS is a fully coupled planner. However, experiments show that this is rarely needed and K-CBS usually finds solutions much before 
this point. Nevertheless, the capabilities are possible. 
*/
class myDemoSystemMerger: public omrc::SystemMerger
{
public: 
    myDemoSystemMerger(const omrc::SpaceInformationPtr &si): omrc::SystemMerger(si) {};

    virtual std::pair<const omrc::SpaceInformationPtr, const ompl::multirobot::base::ProblemDefinitionPtr> merge(const int index1, const int index2) const override
    {
        // actually perform the problem set-up here and return the SpaceInformation and ProblemDefinition -- return pair with nullptrs if unable to merge
        if (index1 > 0 && index2 > 0) 
            return std::make_pair(nullptr, nullptr);
        else
            return std::make_pair(nullptr, nullptr);
    }
};

class myDemoGoalCondition: public ob::GoalRegion
{
public:
    myDemoGoalCondition(const ob::SpaceInformationPtr &si, std::vector<double> goal): 
        ob::GoalRegion(si), gx_(goal[0]), gy_(goal[1])
    {
        threshold_ = 0.2;
    }
    
    double distanceGoal(const ob::State *st) const override
    {
        // const auto *robot_state = st->as<ob::SE2StateSpace::StateType>();
        const auto *robot_state = st->as<ob::RealVectorStateSpace::StateType>();
        const double x = robot_state->values[0];
        const double y = robot_state->values[1];
        return sqrt(pow(x - gx_, 2) + pow(y - gy_, 2));

    }
private:
    float gx_;
    float gy_;
};

void myDemoPropagateFunction(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    auto startTyped = start->as<ob::RealVectorStateSpace::StateType>();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    auto resultTyped = result->as<ob::RealVectorStateSpace::StateType>();
    float dt_=0.1;
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

// K-CBS and PP both work for multiple type of low-level planners. 
// Providing this function to the multi-agent space information will let you use any of them
ompl::base::PlannerPtr myDemoPlannerAllocator(const ompl::base::SpaceInformationPtr &si)
{
    const oc::SpaceInformationPtr siC = std::static_pointer_cast<ompl::control::SpaceInformation>(si);
    ompl::base::PlannerPtr planner = std::make_shared<oc::RRT>(siC);
    return planner;
}

void plan(const std::string plannerName)
{
    std::vector<std::vector<double>> start_reals{{1,1}, {5,5}};
    std::vector<std::vector<double>> goal_reals{{8,7}, {2,1}};
    
    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (size_t i = 0; i < start_reals.size(); i++) 
    {
        // construct the state space we are planning in
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.setHigh(10);

        space->setBounds(bounds); 

        // create a control space
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

        // set the bounds for the control space
        ob::RealVectorBounds cbounds(2);
        cbounds.setLow(-0.3);
        cbounds.setHigh(0.3);

        cspace->setBounds(cbounds);

        // construct an instance of  space information from this control space
        auto si = std::make_shared<oc::SpaceInformation>(space, cspace);
        si->setStateValidityChecker(std::make_shared<myDemoStateValidityChecker>(si));
        si->setStatePropagator(myDemoPropagateFunction);
        si->setPropagationStepSize(0.1);
        si->setMinMaxControlDuration(1, 10);
        // create a start state
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        start[0] = start_reals[i][0];
        start[1] = start_reals[i][1];

        // create a problem instance
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<myDemoGoalCondition>(si, goal_reals[i]));
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }
    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();
    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = myDemoPlannerAllocator;
    ma_si->setPlannerAllocator(allocator);
    if (plannerName == "K-CBS")
    {
        // plan using Kinodynamic Conflict Based Search
        auto planner = std::make_shared<omrc::KCBS>(ma_si);
        planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition

        planner->setLowLevelSolveTime(0.5);

        bool solved = planner->as<omrb::Planner>()->solve(30.0);
        if (solved)
        {
            std::cout << "Found solution!" << std::endl;
            omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
            std::ofstream MyFile("plan.txt");
            solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "Robot");
            std::ofstream MyFile2("tree.txt");
            planner->printConstraintTree(MyFile2);
        }
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // std::string inputFile = "/home/akmarak-laptop/IMRC/db-CBS/example/wall.yaml";
    // YAML::Node env_file = YAML::LoadFile(inputFile);
    std::string plannerName = "K-CBS";
    plan(plannerName);

    return 0;
}
