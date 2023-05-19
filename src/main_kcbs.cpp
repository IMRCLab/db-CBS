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
        // cast the abstract state type to the type we expect
        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        // one must code required logic to figure out if state at pos & rot is valid.

        // this returns a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return (const void*)rot != (const void*)pos;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        /* We assume robots are all disks of varying size (see myDemoStateValidityChecker::radii_) */

        // one can get the names of robots via these commands
        std::string robot1 = si_->getStateSpace()->getName();
        std::string robot2 = state2.first->getStateSpace()->getName();

        // one can get the states of robots via these commands
        const auto *robot1_state = state1->as<ob::SE2StateSpace::StateType>();
        const auto *robot2_state = state2.second->as<ob::SE2StateSpace::StateType>();

        // one must code required logic to figure out if robot1 at state1 collides with robot2 at state2
        // this example assumes all robots are disks in R^2 of varying sizes.
        double rad1 = radii_.at(robot1) + 0.1; // safety criterion
        double rad2 = radii_.at(robot2) + 0.1; // safety criterion

        const double* robot1_pos = robot1_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double* robot2_pos = robot2_state->as<ob::RealVectorStateSpace::StateType>(0)->values;

        double dist = sqrt(pow(robot1_pos[0] - robot2_pos[0], 2) + pow(robot1_pos[1] - robot2_pos[1], 2));

        return dist > (rad1 + rad2);
    }
private:
    std::unordered_map<std::string, double> radii_{ {"Robot 0", 0.1}, 
                                                    {"Robot 1", 0.2}, 
                                                    {"Robot 2", 0.3} };
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
    myDemoGoalCondition(const ob::SpaceInformationPtr &si, std::pair<int, int> goal): 
        ob::GoalRegion(si), gx_(goal.first), gy_(goal.second)
    {
        threshold_ = 0.2;
    }
    
    double distanceGoal(const ob::State *st) const override
    {
        const auto *robot_state = st->as<ob::SE2StateSpace::StateType>();
        const double* robot_pos = robot_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
        return sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2));
    }
private:
    int gx_;
    int gy_;
};


void myDemoPropagateFunction(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * duration * cos(rot),
        pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
        rot    + ctrl[1] * duration);
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
    // create start and goals for every robot
    std::unordered_map<std::string, std::pair<int, int>> start_map{ {"Robot 0", {1, 1}}, 
                                                                    {"Robot 1", {5, 5}}, 
                                                                    {"Robot 2", {8, 8}} };
    std::unordered_map<std::string, std::pair<int, int>> goal_map{  {"Robot 0", {8, 7}}, 
                                                                    {"Robot 1", {2, 1}}, 
                                                                    {"Robot 2", {5, 6}} };

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (int i = 0; i < 3; i++) 
    {
        // give robot a name 
        std::string name = "Robot " + std::to_string(i);

        // construct the state space we are planning in
        auto space = std::make_shared<ob::SE2StateSpace>();

        // set the bounds for the R^2 part of SE(2)
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

        // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<myDemoStateValidityChecker>(si));

        // set the state propagation routine
        si->setStatePropagator(myDemoPropagateFunction);

        // it is highly recommended that all robots use the same propogation step size
        si->setPropagationStepSize(0.1);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

        // name the state space parameter (not required but helpful for robot-to-robot collision checking)
        si->getStateSpace()->setName(name);

        // create a start state
        ob::ScopedState<ob::SE2StateSpace> start(space);
        start[0] = start_map.at(name).first;
        start[1] = start_map.at(name).second;
        start[2] = 0.0;

        // create a problem instance
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);

        // set the start and goal states
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<myDemoGoalCondition>(si, goal_map.at(name)));

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }

    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = myDemoPlannerAllocator;
    ma_si->setPlannerAllocator(allocator);

    if (plannerName == "PP")
    {
        // plan for all agents using a prioritized planner (PP)
        auto planner = std::make_shared<omrc::PP>(ma_si);
        planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition
        bool solved = planner->as<omrb::Planner>()->solve(30.0);

        if (solved)
        {
            std::cout << "Found solution!" << std::endl;
            omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
            std::ofstream MyFile("plan.txt");
            solution->as<omrc::PlanControl>()->printAsMatrix(MyFile, "Robot");
        }
    }
    else if (plannerName == "K-CBS")
    {
        // plan using Kinodynamic Conflict Based Search
        auto planner = std::make_shared<omrc::KCBS>(ma_si);
        planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition

        // set the system merger
        // ma_si->setSystemMerger(std::make_shared<myDemoSystemMerger>(ma_si));

        // set the merge bound of K-CBS
        // planner->setMergeBound(0);

        // set the low-level solve time
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
        // planner.reset();
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::string plannerName = "K-CBS";
    // std::string plannerName = "PP";
    plan(plannerName);

    return 0;
}
