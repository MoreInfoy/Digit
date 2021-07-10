//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"

Manager::Manager(const RobotState &state) : _state(state) {
    tsc = new TSC_IMPL(URDF);

}

Manager::~Manager() {
    delete tsc;
}

void Manager::init() {

}

void Manager::run() {
    Reference ref;
/*#ifdef FIXED_BASE
    VecInt mask(8);
    mask.setZero();
#else
    VecInt mask(8);
    mask.setOnes();
#endif*/
    VecInt mask(8);
    mask.setZero();
    tsc->setContactMask(mask);
    tsc->run(ref, _state);
}

Poplar::Vec Manager::output() {
    return tsc->jointsCmd().tau_ff;
}
