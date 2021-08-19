//
// Created by nimpng on 8/19/21.
//

#ifndef POPLARDIGIT_FSM_H
#define POPLARDIGIT_FSM_H

#include "Gait.h"

#include <vector>

using namespace std;

class FSM {
public:
    FSM();

    GAIT_TYPE next();

    GAIT_TYPE last();

    GAIT_TYPE current();

    Poplar::Index iterations();

    vector<GAIT_TYPE> future(Poplar::Index n_iter);

    void run(GAIT_TYPE gaitType);

private:
    GAIT_TYPE _current_gaitType;
    GAIT_TYPE _last_gaitType;
    GAIT_TYPE _next_gaitType;

    Poplar::Index _iter, _period, _start;
};


#endif //POPLARDIGIT_FSM_H
