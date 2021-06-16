//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_PLANNER_H
#define POPLARDIGIT_PLANNER_H

#include "StateAndCommand.h"

class Planner {
public:
    Planner();

    virtual void plan(const RobotState &state) = 0;

    virtual void trajectory() = 0;
};

#endif //POPLARDIGIT_PLANNER_H
