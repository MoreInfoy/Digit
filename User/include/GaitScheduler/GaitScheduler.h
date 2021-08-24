//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_GAITSCHEDULER_H
#define POPLARDIGIT_GAITSCHEDULER_H

#include "StateAndCommand.h"
#include "GaitScheduler/Gait.h"
#include "GaitScheduler/FSM.h"
#include "Poplar.h"

class GaitScheduler {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GaitScheduler(Scalar dt);

    void run(size_t iter, const RobotState &state, RobotWrapper &robot);

    const GaitData &data();

    void updateContactTable(size_t n_pre, size_t n_per);

    GAIT_TYPE gait_type();

private:
    Scalar _dt;
    GaitData _gaitData;
    Gait standing, walking;
    Gait *gait = nullptr;
    GAIT_TYPE gaitType;

    FSM fsm;
};


#endif //POPLARDIGIT_GAITSCHEDULER_H
