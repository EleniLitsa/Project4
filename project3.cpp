#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/control/SimpleSetup.h>
// Except for the state space definitions and any planners
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

//#include <ompl/geometric/planners/prm/PRM.h>
#include "RGRRT.h"

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Use placeholder namesapce for arguments to bound functions.
using namespace std::placeholders;

// This is our state validitiy checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State* state, double sideLength, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::CompoundState* cstate = state->as<ompl::base::CompoundState>();

    // Extract x, y, theta
    const ompl::base::RealVectorStateSpace::StateType* r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    const ompl::base::SO2StateSpace::StateType* so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);
}


// This is our state validity checker.  It says every state is valid.
bool stateAlwaysValid(const ompl::base::State* /*state*/)
{
    return true;
}

void planWithSimpleSetupR2(const std::vector<Rectangle>& obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    // For a robot that can translate in the plane, we can use R^2 directly
    // We also need to set bounds on R^2
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2); // x and y have a minimum of -2
    bounds.setHigh(2); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(r2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker.
    // Note, we are "binding" the obstacles to the state validity checker. The
    // _1 notation is from std::placeholders and indicates "the first argument"
    // to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStatePoint, _1, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    ompl::base::ScopedState<> start(r2);
    start[0] = -0.9;
    start[1] = -0.9;

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 0.9;
    goal[1] = 0.9;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    //ss.setPlanner();

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetupSE2(const std::vector<Rectangle>& obstacles)
{
    // Step 1) Create the state (configuration) space for your system
    // In this instance, we will plan for a square of side length 0.3
    // that both translates and rotates in the plane.
    // The state space can be easily composed of simpler state spaces
    ompl::base::StateSpacePtr se2;

    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2); // x and y have a minimum of -2
    bounds.setHigh(2); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem using OMPL.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker

    // Note, we are "binding" the side length, 0.3, and the obstacles to the
    // state validity checker. The _1 notation is from std::placeholders and
    // indicates "the first argument" to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    // You can index into the components of the state easily with ScopedState
    // The indexes correspond to the order that the StateSpace components were
    // added into the StateSpace
    ompl::base::ScopedState<> start(se2);
    start[0] = -0.9;
    start[1] = -0.9;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 0.9;
    goal[1] = 0.9;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    ompl::base::PlannerPtr planner(new ompl::geometric::RGRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}


/*Rectangle box(int x, int y, double width, double height){
    Rectangle obstacle;
    obstacle.x = x;
    obstacle.y = y;
    obstacle.width = width;
    obstacle.height = height;
    return obstacle;
    
}*/

void addObstacles1(std::vector<Rectangle>* obs){
    
    // obs->push_back(box(-0.5,-0.5,1.0,1.0));
    // obs->push_back(box(-1.5,0,1,2));
    // obs->push_back(box(0,-1.5,2,0.5));
    Rectangle obstacle;
    obstacle.x = -0.5;
    obstacle.y = -0.5;
    obstacle.width = 1.0;
    obstacle.height = 1.0;
    obs->push_back(obstacle);
    Rectangle obstacle1;
    obstacle1.x = -1.5;
    obstacle1.y = 0.0;
    obstacle1.width = 1.0;
    obstacle1.height = 2.0;
    obs->push_back(obstacle1);
    Rectangle obstacle2;
    obstacle2.x = 0.0;
    obstacle2.y = -1.5;
    obstacle2.width = 2.0;
    obstacle2.height = 0.5;
    obs->push_back(obstacle2);
}
void addObstacles2(std::vector<Rectangle>* obs){
    Rectangle obstacle1;
    obstacle1.x = -0.5;
    obstacle1.y = 1.0;
    obstacle1.width = 1.0;
    obstacle1.height = 1.0;
    obs->push_back(obstacle1);
    Rectangle obstacle2;
    obstacle2.x = -2.0;
    obstacle2.y = -0.5;
    obstacle2.width = 1.0;
    obstacle2.height = 1.0;
    obs->push_back(obstacle2);
    Rectangle obstacle3;
    obstacle3.x = -0.5;
    obstacle3.y = -0.5;
    obstacle3.width = 1.0;
    obstacle3.height = 1.0;
    obs->push_back(obstacle3);
    Rectangle obstacle4;
    obstacle4.x = 1.0;
    obstacle4.y = -0.5;
    obstacle4.width = 1.0;
    obstacle4.height = 1.0;
    obs->push_back(obstacle4);
    Rectangle obstacle5;
    obstacle5.x = -0.5;
    obstacle5.y = -2.0;
    obstacle5.width = 1.0;
    obstacle5.height = 1.0;
    obs->push_back(obstacle5);
}
int main(int, char **)
{
    std::vector<Rectangle> obstacles;
    int choice;
    do
    {
        std::cout << "Set Obstacles: "<< std::endl;
        std::cout << " (1) Three Boxes" << std::endl;
        std::cout << " (2) Many Boxes" << std::endl;
        

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    
    switch(choice)
    {
    case 1:
        addObstacles1(&obstacles);
        std::cout << "one"<<std::endl;
        break;
    case 2:
        addObstacles2(&obstacles);
        std::cout << "two"<<std::endl;
        break;
    }
    std::cout <<obstacles.size() << std::endl; 

    choice;
    do
    {
        std::cout << "Plan for: "<< std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch(choice)
    {
    case 1:
        planWithSimpleSetupR2(obstacles);
        break;
    case 2:
        planWithSimpleSetupSE2(obstacles);
        break;
    }
    return 0;
}
