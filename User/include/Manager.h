//
// Created by nimpng on 6/16/21.
//

#ifndef POPLARDIGIT_MANAGER_H
#define POPLARDIGIT_MANAGER_H

#include "Controller/Controller.h"
#include "GaitScheduler/GaitScheduler.h"
#include "Planner/Planner.h"

class Manager {
public:
    explicit Manager(const RobotState &state);

    void init();

    void run();

    Poplar::Vec output();
};


#endif //POPLARDIGIT_MANAGER_H
