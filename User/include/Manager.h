//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_MANAGER_H
#define POPLARDIGIT_MANAGER_H

#include "Controller/TSC.h"
#include "GaitScheduler/GaitScheduler.h"
#include "Planner/FootPlanner.h"
#include "Planner/FloatingBasePlanner.h"
#include "lcm/RobotMessage.hpp"
#include <lcm/lcm-cpp.hpp>

class Manager {
public:
    explicit Manager(const RobotState &state);

    ~Manager();

    void init();

    void run();

    Poplar::Vec output();

private:
    void update();

    const RobotState &_state;
    RobotWrapper robot;
    GaitScheduler *gaitScheduler;
    FloatingBasePlanner *floatingBasePlanner;
    FootPlanner *footPlanner;
    TSC_IMPL *tsc;
    Tasks tasks;
    size_t _iter;

    lcm::LCM lcm;
    RobotMessage robotMsg;

    Poplar::Index mpc_horizon;
    Scalar mpc_dt, dt;
};


#endif //POPLARDIGIT_MANAGER_H