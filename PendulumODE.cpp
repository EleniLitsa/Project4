#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
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


void planWithSimpsleSetup(void){
ompl::base::StateSpacePtr s;

const double pi = boost::math::constants::pi<double>();

ob::StateSpacePtr r1(new ob::RealVectorStateSpace(1));
 // Set bounds on R^1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10); // minimum value of rotational velocity
    bounds.setHigh(10); // maximum value for rotational velocity

    // Set the bounds
    r1->as<ob::RealVectorStateSpace>()->setBounds(bounds);

   ob::StateSpacePtr so2(new ob::SO2StateSpace());

    s = r1 + so2;

// create the state space 
 ob::StateSpacePtr space(s);
 // set the bounds for the R^2 part of SE(2) 
 ob::RealVectorBounds bounds(2);
 bounds.setLow(-5);
 bounds.setHigh(5);
 space->as<ob::SE2StateSpace>()->setBounds(bounds);

// create the control space
 oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 1));
// set the bounds for the torque
  ob::RealVectorBounds cbounds(1);
cbounds.setLow(0);
cbounds.setHigh(3);

cspace->as<DemoControlSpace>()->setBounds(cbounds);


// define a simple setup class
 oc::SimpleSetup ss(cspace);

// set state validity checking for this space
 ss.setStateValidityChecker(std::bind(&isStateValid, ss.getSpaceInformation().get(), std::placeholders::_1));


// instantiate the derived solver
oc::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ss, &PendulumODE));
// oc::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));
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

ss.setup();

// create a problem instance 
ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(ss));

// set the start and goal states
 pdef.setStartAndGoalStates(start, goal, 0.05);

ob::PlannerPtr planner(new oc::RRT(si));

// set the problem we are trying to solve for the planner
 planner->setProblemDefinition(pdef);

// perform setup steps for the planner
  planner->setup();

// print the settings for this space
 ss.printSettings(std::cout);


// attempt to solve the problem within one second of planning
  ob::PlannerStatus solved = planner.solve(10.0);

 if (solved)
     {
        // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef.getSolutionPath();
         std::cout << "Found solution:" << std::endl;
 
         // print the path to screen
         path.print(std::cout);
     }
     else
        std::cout << "No solution found" << std::endl;


}

 void main(int, char **)
 {
      planWithSimpleSetup();

      return 0;
 
 }

