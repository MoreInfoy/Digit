//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"

Manager::Manager(const RobotState &state) : _state(state) {
    tsc = new TSC_IMPL(URDF);
    vector<string> contact_virtual_link;
    contact_virtual_link.emplace_back("contact1");
    contact_virtual_link.emplace_back("contact2");
    contact_virtual_link.emplace_back("contact3");
    contact_virtual_link.emplace_back("contact4");
    contact_virtual_link.emplace_back("contact5");
    contact_virtual_link.emplace_back("contact6");
    contact_virtual_link.emplace_back("contact7");
    contact_virtual_link.emplace_back("contact8");
    tsc->setContactVirtualLink(contact_virtual_link);
}

Manager::~Manager() {
    delete tsc;
}

void Manager::init() {

}

void Manager::run() {
    Reference ref;
    VecInt mask(8);
    mask.setOnes();
    tsc->setContactMask(mask);
    tsc->run(ref, _state);
}

Poplar::Vec Manager::output() {
    return tsc->jointsCmd().tau_ff;
}
