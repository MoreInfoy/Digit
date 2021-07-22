//
// Created by nimapng on 7/2/21.
//

#ifndef POPLARDIGIT_TSC_H
#define POPLARDIGIT_TSC_H

#include "Controller/Controller.h"
#include "PoplarLib.h"

using namespace TSC;

class TSC_IMPL : public Controller {
public:
    TSC_IMPL(string urdf_file, string srdf);

    ~TSC_IMPL();

    void setContactMask(const VecInt &mask);

    void setContactVirtualLink(vector<string> &contact_virtual_link);

    virtual void run(const Reference &ref, const RobotState &state);

    virtual const JointsCmd &jointsCmd();

    RobotWrapper &robot() {
        return ref(_robot);
    }


    ConstVecRef getOptimalContactForce() {
        return tsc->getOptimalContactForce();
    }

    ConstVecRef getOptimalQacc() {
        return tsc->getOptimalQacc();
    }


private:
    void solve(ConstVecRef qpos, ConstVecRef qvel, const VecInt &mask);

    ConstVecRef getOptimalTorque() {
        return tsc->getOptimalTorque();
    }

    RobotWrapper _robot;
    Vec _lb, _ub;
    VecInt _mask;
    SE3MotionTask *mt_waist;
    SE3MotionTask *rf;
    SE3MotionTask *lf;
    CoMMotionTask *com;
    RegularizationTask *rt;
    JointsNominalTask *jointsNominalTask;
    AngularMomentumTask *angularMomentumTask;
    ContactPointsConstraints *cpcstr;
    ContactForceConstraints *cfcstr;
    ClosedChainsConstraints *closedChainsConstraints;
    ActuatorLimit *actuatorLimit;
    QaccBound *qaccBound;
    TaskSpaceControl *tsc;

    vector<string> contact_virtual_link;
    vector<pair<string, string>> link_pairs;
    vector<pair<string, RealNum>> spring_joints;

    JointsCmd _jointsCmd;
    size_t _iter;
};


#endif //POPLARDIGIT_TSC_H
