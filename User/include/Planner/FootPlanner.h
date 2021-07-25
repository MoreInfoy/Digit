//
// Created by nimpng on 7/25/21.
//

#ifndef POPLARDIGIT_FOOTPLANNER_H
#define POPLARDIGIT_FOOTPLANNER_H

#include "Planner.h"

class FootPlanner: public Planner{
public:
    FootPlanner();

    virtual void plan(size_t iter,const RobotState &state, const GaitData &gaitData, Tasks &tasks);

private:
};


#endif //POPLARDIGIT_FOOTPLANNER_H
