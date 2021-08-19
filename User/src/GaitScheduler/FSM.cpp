//
// Created by nimpng on 8/19/21.
//

#include "GaitScheduler/FSM.h"
#include "../../include/GaitScheduler/FSM.h"

FSM::FSM() {
    _current_gaitType = GAIT_TYPE::STANCE;
    _last_gaitType = GAIT_TYPE::STANCE;
    _next_gaitType = GAIT_TYPE::STANCE;
    _iter = 0;
    _period = 0;
    _start = _iter;
}

GAIT_TYPE FSM::next() {
    return _next_gaitType;
}

GAIT_TYPE FSM::current() {
    return _current_gaitType;
}

GAIT_TYPE FSM::last() {
    return _last_gaitType;
}

void FSM::run(GAIT_TYPE gaitType) {
    if (_next_gaitType != gaitType) {
        _start += _iter;
        _period = 4000;
        _iter = 0;
        _next_gaitType = gaitType;
    }
    if (_current_gaitType != _next_gaitType && _period < _iter) {
        _last_gaitType = _current_gaitType;
        _current_gaitType = _next_gaitType;
    }

    // TODO:
    /*switch (gaitType) {
        case GAIT_TYPE::STANCE:
            break;
        case GAIT_TYPE::WALK:
            break;
        case GAIT_TYPE::RUN:
            break;
        default:
            break;
    }*/
    _iter++;
}

vector<GAIT_TYPE> FSM::future(Poplar::Index n_iter) {
    vector<GAIT_TYPE> gaitArray(n_iter);
    for (int i = 0; i < n_iter; i++) {
        if (_iter + i < _period) {
            gaitArray[i] = _current_gaitType;
        } else {
            gaitArray[i] = _next_gaitType;
        }
    }
    return gaitArray;
}

Poplar::Index FSM::iterations() {
    return _iter;
}
