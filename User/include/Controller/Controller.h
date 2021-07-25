//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_CONTROLLER_H
#define POPLARDIGIT_CONTROLLER_H

#include "StateAndCommand.h"

class Controller {
public:
    Controller();

    virtual void run(size_t iter, const RobotState &state, const GaitData &gaitData, const Tasks &tasks) = 0;

    virtual const JointsCmd &jointsCmd() = 0;
};

#endif //POPLARDIGIT_CONTROLLER_H
