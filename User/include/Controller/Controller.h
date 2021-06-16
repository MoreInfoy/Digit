//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_CONTROLLER_H
#define POPLARDIGIT_CONTROLLER_H

#include "StateAndCommand.h"

class Controller {
public:
    Controller();

    virtual void run(const Reference &ref, const RobotState &state) = 0;

    virtual const JointsCmd &jointsCmd() = 0;
};

#endif //POPLARDIGIT_CONTROLLER_H
