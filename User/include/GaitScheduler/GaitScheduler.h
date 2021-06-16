//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_GAITSCHEDULER_H
#define POPLARDIGIT_GAITSCHEDULER_H

#include "StateAndCommand.h"

class GaitScheduler {
public:
    GaitScheduler();

    void run(const RobotState &state);
};


#endif //POPLARDIGIT_GAITSCHEDULER_H
