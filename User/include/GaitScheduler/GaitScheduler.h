//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_GAITSCHEDULER_H
#define POPLARDIGIT_GAITSCHEDULER_H

#include "StateAndCommand.h"

class GaitScheduler {
public:
    GaitScheduler();

    void run(size_t iter, const RobotState &state);

    const GaitData &data();

private:
    GaitData _gaitData;
};


#endif //POPLARDIGIT_GAITSCHEDULER_H
