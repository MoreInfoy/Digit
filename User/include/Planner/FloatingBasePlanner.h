//
// Created by nimpng on 7/25/21.
//

#ifndef POPLARDIGIT_FLOATINGBASEPLANNER_H
#define POPLARDIGIT_FLOATINGBASEPLANNER_H


#include "Planner.h"

class FloatingBasePlanner : public Planner {
public:
    FloatingBasePlanner();

    virtual void plan(size_t iter, const RobotState &state, const GaitData &gaitData, Tasks &tasks);

private:

};


#endif //POPLARDIGIT_FLOATINGBASEPLANNER_H
