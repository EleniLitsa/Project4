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


namespace oc = ompl::control;
namespace ob = ompl::base;


void CarODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  
    const double *u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
 
    // Retrieve the current orientation of the car.  The memory for ob::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    // 3: velocity

    const double theta = q[2];
    const double velocity = q[3];
    const double angularVelocity= u[0];
    const double acceleration = u[1];

    qdot.resize(q.size(), 0);
    qdot[0] = velocity * cos(theta);  // x-dot
    qdot[1] = velocity * sin(theta);  // y-dot
    qdot[2] = angularVelocity;    // theta-dot
    qdot[3] = acceleration;    // velocity-dot
}
void postPropagate (const ob::State*, const oc::Control*, const double, ob::State *result)
//void postPropagate(const ob::State* state, const oc::Control* control, const double duration, base::State* result)
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

void planWithSimpleSetup(void){
ob::StateSpacePtr space;

ob::StateSpacePtr r3(new ob::RealVectorStateSpace(3)); // x-y space
 // Set bounds for x and y
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10,1); // minimum value of x
    bounds.setHigh(10,1); // maximum value of x
    bounds.setLow(-10,2); // minimum value of y
    bounds.setHigh(10,2); // maximum value of y
    bounds.setLow(-0.3,3); // minimum value of velocity
    bounds.setHigh(0.3,3); // maximum value of velocity

    // Set the bounds
    r3->as<ob::RealVectorStateSpace>()->setBounds(bounds);

ob::StateSpacePtr so2(new ob::SO2StateSpace());  // angle

space = r3 + so2;

// create the state space 
// ob::StateSpacePtr space(s);

// create the control space
 oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
// set the bounds for the control space 
  ob::RealVectorBounds cbounds(2);
cbounds.setLow(-0.3);
cbounds.setHigh(0.3);

cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);


// define a simple setup class
oc::SimpleSetup ss(cspace);
//oc::SpaceInformationPtr ss(new oc::SpaceInformation(space, cspace));

// set state validity checking for this space
 ss.setStateValidityChecker(std::bind(&isStateValid, ss.getSpaceInformation().get(), std::placeholders::_1));


// instantiate the derived solver
//oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss, &CarODE));
 oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &CarODE));
// StatePropagator: defines how the system moves given a specific control
ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));

     // set start and goal configuration

    ob::ScopedState<> start(space);
    start[0] = -4.0;
    start[1] = -4.0;
    start[2] = 0.0;
    start[3] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 4.0;
    goal[1] = 4.0;
    goal[2] = 0.0;
    goal[3] = 0.0;

//ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(ss));
//pdef->setStartAndGoalStates(start, goal, 0.1);
//ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
//planner->setProblemDefinition(pdef);
//planner->setup();
//ob::PlannerStatus solved = planner->solve(10.0);

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

 int main(int, char **)
 {
      planWithSimpleSetup();
     
      return 0;
 
 }

