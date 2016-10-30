/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "RGRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include <limits>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    addIntermediateStates_ = false;
    lastGoalMotion_ = nullptr;

    goalBias_ = 0.05;

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates);
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    //number of controls to consider when searching for R(q)
    int numControls=10;

    //find the bounds of the control space
    std::shared_ptr<oc::ControlSpace> cspace = siC_->getControlSpace();
    RealVectorControlSpace* cvspace = cspace->as<RealVectorControlSpace>();
    ob::RealVectorBounds cbounds = cvspace->getBounds();
    double low=cbounds.low[0];
    double high=cbounds.high[0];
    double jump=(high-low)/numControls;
    int controlDuration=1;

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);

        double cValue=0.0;
        for(int i=0;i<numControls;i++){
            RealVectorControlSpace::ControlType* control = 
            dynamic_cast<RealVectorControlSpace::ControlType*>
            (siC_->allocControl());
            
            control->values[0]=cValue;
            motion->controls.push_back(control);

            //testCode
            // RealVectorControlSpace::ControlType* tcontrol = 
            // dynamic_cast<RealVectorControlSpace::ControlType*>
            // (motion->controls[i]);
            // printf("FC:%f; ",tcontrol->values[0]);
            //testCodeEnd

            //base::State *reachState;
            //siC_->propagate(st,control,controlDuration,reachState);
            std::vector<base::State *> pstates;
            siC_->propagateWhileValid(motion->state, control, controlDuration, pstates, true);
            //printf("pss:%d;",pstates.size());
            // for(int j=0;j<pstates.size();j++){
            //     motion->states.push_back(pstates[j]);
            // }
            motion->states.push_back(pstates[controlDuration-1]);
            //motion->states.push_back(reachState);
            cValue+=jump;
        }
        //testCode
        printf("mincd:%d;",siC_->getMinControlDuration());
        //printf("(ss,cs):(%d,%d);",
         //    motion->states.size(),
         //    motion->controls.size());
        //testCodeEnd

        nn_->add(motion);
    }
    //testCode
    // std::vector<Motion*> mvector;
    // nn_->list(mvector);
    // printf("mvs:%d",mvector.size());
    // RealVectorControlSpace::ControlType* tcontrol = 
    // dynamic_cast<RealVectorControlSpace::ControlType*>
    // (mvector[0]->controls[2]);
    // printf("FC:%f; ",tcontrol->values[0]);
    //testCodeEnd

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        

        ob::State *foundState=NULL;
        Control *foundControl=NULL;
        bool hadFoundState=false;

        Motion *nmotion;
        while (!hadFoundState){
            /* sample random state (with goal biasing) */
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
                sampler_->sampleUniform(rstate);

            /* find closest state in the tree */
            nmotion = nn_->nearest(rmotion);
            
            //std::vector<Control*> ncontrols=nmotion->controls;   
            //testCode
            // RealVectorControlSpace::ControlType* tcontrol = 
            // dynamic_cast<RealVectorControlSpace::ControlType*>
            // (ncontrols[2]);
            // printf("FC:%f; ",tcontrol->values[0]);
            // printf("(ss,cs):(%d,%d);",
            //     nmotion->states.size(),
            //     nmotion->controls.size());
            //testCodeEnd


            
            for(int i=0;i<nmotion->states.size();i++){
                
                base::State* rqstate=nmotion->states[i];
                Control* rqcontrol=nmotion->controls[i];

                // double dist1=distanceFunction2(nmotion,rqstate);
                // double dist2=distanceFunction2(rmotion,rqstate);
                double dist1=si_->distance(nmotion->state, rqstate);
                double dist2=si_->distance(rmotion->state, rqstate);

                //testCode
                //printf("i:%d;",i);
                // RealVectorControlSpace::ControlType* tcontrol = 
                // dynamic_cast<RealVectorControlSpace::ControlType*>
                // (rqcontrol);
                // printf("i:%d,d:(%f,%f);",i,dist1,dist2);
                //printf("i:%d,FC:%f; ",i,tcontrol->values[0]);
                
                //testCodeEnd
                if(dist2<dist1){
                    hadFoundState=true;
                    foundState=nmotion->states[i];
                    foundControl=nmotion->controls[i];
                    
                }
            }
        }

        // RealVectorControlSpace::ControlType* fcontrol = 
        //     dynamic_cast<RealVectorControlSpace::ControlType*>
        //     (foundControl);
        // printf("FC:%f; ",fcontrol->values[0]);
        

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        
        if (controlDuration >= siC_->getMinControlDuration())
        //if (controlDuration >= siC_->getMinControlDuration())
        {
            /* create a motion */
            auto *motion = new Motion(siC_);
            //si_->copyState(motion->state, rmotion->state);
            si_->copyState(motion->state, foundState);
            //siC_->copyControl(motion->control, rctrl);
            siC_->copyControl(motion->control, foundControl);
            //motion->steps = cd;
            motion->steps = controlDuration;
            motion->parent = nmotion;

            double cValue=0.0;
            for(int i=0;i<numControls;i++){
                RealVectorControlSpace::ControlType* control = dynamic_cast<RealVectorControlSpace::ControlType*>(siC_->allocControl());
                
                control->values[0]=cValue;
                motion->controls.push_back(control);
                //base::State *reachState;
                //siC_->propagate(st,control,controlDuration,reachState);
                std::vector<base::State *> pstates;
                siC_->propagateWhileValid(motion->state, control, controlDuration, pstates, true);
                // for(int j=0;j<pstates.size();j++){
                //     motion->states.push_back(pstates[j]);
                // }
                motion->states.push_back(pstates[controlDuration-1]);
                //motion->states.push_back(reachState);
                cValue+=jump;
            }
            nn_->add(motion);
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            if (solv)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
        
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
