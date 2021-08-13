//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_MANAGER_H
#define POPLARDIGIT_MANAGER_H

#include "Controller/TSC.h"
#include "GaitScheduler/GaitScheduler.h"
#include "Planner/FootPlanner.h"
#include "Planner/FloatingBasePlanner.h"
#include "RobotMessage.hpp"
#include "Trajectory_LCM.hpp"
#include <lcm/lcm-cpp.hpp>

class Manager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Manager(const RobotState &state);

    ~Manager();

    void init();

    void run();

    Poplar::ConstVecRef output();

private:
    void update();

    void runLCM();

    const RobotState &_state;
    Poplar::Index mpc_horizon;
    Scalar mpc_dt, dt;

    RobotWrapper robot;
    GaitScheduler gaitScheduler;
    FootPlanner footPlanner;
    FloatingBasePlanner floatingBasePlanner;
    TSC_IMPL tsc;
    Tasks tasks;
    size_t _iter;

    lcm::LCM lcm1, lcm2;
    RobotMessage robotMsg;
    Trajectory_LCM trajectoryLcm;

    Poplar::Vec qpos, qdot;

};


#endif //POPLARDIGIT_MANAGER_H