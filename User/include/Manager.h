//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_MANAGER_H
#define POPLARDIGIT_MANAGER_H

#include "Controller/TSC.h"
#include "GaitScheduler/GaitScheduler.h"
#include "Planner/FootPlanner.h"
#include "Planner/FloatingBasePlanner.h"
#include <ostream>

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

    void saveAllData();

    const RobotState &_state;
    Poplar::Index mpc_horizons;
    Scalar mpc_dt, dt;
    std::ofstream out_state, out_gait, out_planning, out_tsc;

    RobotWrapper robot;
    GaitScheduler gaitScheduler;
    FootPlanner footPlanner;
    FloatingBasePlanner floatingBasePlanner;
    TSC_IMPL tsc;
    Tasks tasks;
    size_t _iter;

    Poplar::Vec qpos, qdot;
    Poplar::Vec3 com_pos_des, com_vel_des;
};


#endif //POPLARDIGIT_MANAGER_H