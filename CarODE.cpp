#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include "RGRRT.h"
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;


// Definition of the ODE for the car.
void CarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    const double velocity = q[3];
    const double theta = q[2];
    const double angularVelocity= u[0];
    const double acceleration = u[1];   

    qdot.resize(q.size(), 0);
    qdot[0] = velocity * cos(theta);  // x-dot
    qdot[1] = velocity * sin(theta);  // y-dot
    qdot[2] = angularVelocity;    // theta-dot
    qdot[3] = acceleration;    // velocity-dot
}

void postPropagate (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;

    ob::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s[1]);
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    /// cast the abstract state type to the type we expect
    const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = s->as<ob::RealVectorStateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::SO2StateSpace::StateType *rot = s->as<ob::SO2StateSpace::StateType>(1);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}


void planWithSimpleSetup()
{
    // create the state space
    ob::StateSpacePtr space;

    ob::StateSpacePtr r3(new ob::RealVectorStateSpace(3)); // x-y space
    // Set bounds for x and y
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,-10.0); // minimum value of x
    bounds.setHigh(0,10.0); // maximum value of x
    bounds.setLow(1,-10.0); // minimum value of y
    bounds.setHigh(1,10.0); // maximum value of y
    bounds.setLow(2,-0.3); // minimum value of velocity
    bounds.setHigh(2,0.3); // maximum value of velocity

    // Set the bounds
    r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

   ob::StateSpacePtr so2(new ob::SO2StateSpace());  // angle

   space = r3 + so2;


    // create the control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    //cspace->setBounds(cbounds);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });

    // instantiate the derived solver
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarODE));
    //StatePropagator: defines how the system moves given a specific control
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start[0] = -4.0;
    start[1] = -4.0;
    start[2] = 0.0;
    start[3] = 0.0;

    // create a  goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal[0] = 4.0;
    goal[1] = 4.0;
    goal[2] = 0.0;
    goal[3] = 0.0;

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    //ss.setStartAndGoalStates(start, goal, 0.05);

    // set the planner
    ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
    //ob::PlannerPtr planner(new oc::RGRRT(ss.getSpaceInformation())); 
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds
    ob::PlannerStatus solved = ss.solve(10.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        
        // print the path to screen        
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}
