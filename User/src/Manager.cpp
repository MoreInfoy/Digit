//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"

Manager::Manager(const RobotState &state) : _state(state), _iter(0) {
    gaitScheduler = new GaitScheduler();
    floatingBasePlanner = new FloatingBasePlanner();
    footPlanner = new FootPlanner();
    tsc = new TSC_IMPL(URDF, SRDF);
    tasks.floatingBaseTask.link_name = "torso";
    tasks.leftFootTask.link_name = "left_toe_roll";
    tasks.rightFootTask.link_name = "right_toe_roll";
}

Manager::~Manager() {
    delete gaitScheduler;
    delete floatingBasePlanner;
    delete footPlanner;
    delete tsc;
}

void Manager::init() {

}

void Manager::run() {
    gaitScheduler->run(_iter, _state);
    footPlanner->plan(_iter, _state, gaitScheduler->data(), tasks);
    floatingBasePlanner->plan(_iter, _state, gaitScheduler->data(), tasks);
#ifdef FIXED_BASE
    VecXi mask(8);
    mask.setZero();
    tsc->setContactMask(mask);
#endif
    tsc->run(_iter, _state, gaitScheduler->data(), tasks);
    _iter++;
}

Poplar::Vec Manager::output() {
    return tsc->jointsCmd().tau_ff;
}