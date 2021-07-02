//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_MANAGER_H
#define POPLARDIGIT_MANAGER_H

#include "Controller/TSC.h"
#include "GaitScheduler/GaitScheduler.h"
#include "Planner/Planner.h"

class Manager {
public:
    explicit Manager(const RobotState &state);

    ~Manager();

    void init();

    void run();

    Poplar::Vec output();

private:
    const RobotState &_state;
    TSC_IMPL* tsc;

};


#endif //POPLARDIGIT_MANAGER_H
