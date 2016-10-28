#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <cmath>


namespace oc = ompl::control;
namespace ob = ompl::base;


void PendulumODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
 
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

void postPropagate(const base::State* state, const Control* control, const double duration, base::State* result)
{
   ob::SO2StateSpace SO2;

  ob::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
  SO2.enforceBounds(s[1]);
}


bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
 {
     const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();
 
     const ob::RealVectorStateSpace::StateType *pos = s->as<ob::RealVectorStateSpace::StateType>(0);
 
     const ob::SO2StateSpace::StateType *rot = s->as<ob::SO2StateSpace::StateType>(1);
 
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
 }

void planWithSimpsleSetup(void){
ob::StateSpacePtr space;

const double pi = boost::math::constants::pi<double>();

ob::StateSpacePtr r1(new ob::RealVectorStateSpace(1));
 // Set bounds on R^1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10); // minimum value of rotational velocity
    bounds.setHigh(10); // maximum value for rotational velocity

    // Set the bounds
    r1->as<ob::RealVectorStateSpace>()->setBounds(bounds);

   ob::StateSpacePtr so2(new ob::SO2StateSpace());

    space = r1 + so2;

// create the control space
 oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 1));
// set the bounds for the torque
  ob::RealVectorBounds cbounds(1);
cbounds.setLow(0);
cbounds.setHigh(3);

cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);


// define a simple setup class
 oc::SimpleSetup ss(cspace);

// set state validity checking for this space
 ss.setStateValidityChecker(std::bind(&isStateValid, ss.getSpaceInformation().get(), std::placeholders::_1));


// instantiate the derived solver
//oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss, &PendulumODE));
oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));
// StatePropagator: defines how the system moves given a specific control
ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

     // set start and goal configuration
    ob::ScopedState<> start(space);
    start[0] = -pi/2;
    start[1] = 0.0;
   
     ob::ScopedState<> goal(space);
    goal[0] = pi/2;
    // fmod(state->as<StateType>()->value, 2.0 * boost::math::constants::pi<double>())
    goal[1] = 0.0;


ss.setStartAndGoalStates(start, goal);
ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
ss.setPlanner(planner);
ob::PlannerStatus solved = ss.solve(10.0);

if (solved)
{
std::cout << "Found solution:" << std::endl;
}
else
std::cout << "No solution found" << std::endl;
}

 void main(int, char **)
 {
      planWithSimpleSetup();

      return 0;
 
 }

