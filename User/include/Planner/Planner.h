//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_PLANNER_H
#define POPLARDIGIT_PLANNER_H

#include "StateAndCommand.h"
#include "Poplar.h"

class Planner {
public:
    Planner();

    virtual void plan(size_t iter, const RobotState &state, RobotWrapper &robot, const GaitData &gaitData, Tasks &tasks) = 0;

};

#endif //POPLARDIGIT_PLANNER_H
