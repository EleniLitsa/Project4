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


// Definition of the ODE for the pendulum.
void PendulumODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
     // Retrieve control values.  
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
 
    // Retrieve the current state of the pendulum.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: theta
    // 1: rotational velocity

    const double theta = q[0]; //angle
    const double velocity = q[1];  //rotational velocity 
    const double torque= u[0];  // control signal

    qdot.resize(q.size(), 0);
    qdot[0] = velocity;   // theta-dot
    qdot[1] = -9.81 * cos(theta) + torque;   // rotational velocity-dot
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
    const double pi = boost::math::constants::pi<double>();

    // create the state space
    ob::StateSpacePtr space;

    
ob::StateSpacePtr r1(new ob::RealVectorStateSpace(1));
 // Set bounds on R^1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10); // minimum value of rotational velocity
    bounds.setHigh(10); // maximum value for rotational velocity

    // Set the bounds
    r1->as<ob::RealVectorStateSpace>()->setBounds(bounds);

   ob::StateSpacePtr so2(new ob::SO2StateSpace());  // angle

   space = r1 + so2;


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
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &PendulumODE));
    //StatePropagator: defines how the system moves given a specific control
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start[0] = -pi/2;
    start[1] = 0.0;

    // create a  goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal[0] = pi/2;
    // fmod(state->as<StateType>()->value, 2.0 * boost::math::constants::pi<double>())
    goal[1] = 0.0;

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
