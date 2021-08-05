//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_GAITSCHEDULER_H
#define POPLARDIGIT_GAITSCHEDULER_H

#include "StateAndCommand.h"
#include "GaitScheduler/Gait.h"

class GaitScheduler {
public:
    GaitScheduler(Scalar dt);

    void run(size_t iter, const RobotState &state);

    const GaitData &data();

    ConstMatIntRef contactTable(size_t n_pre, size_t n_per);

private:
    Scalar _dt;
    GaitData _gaitData;
    Gait standing, walking;
    Gait *gait = nullptr;
};


#endif //POPLARDIGIT_GAITSCHEDULER_H
