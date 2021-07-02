//
// Created by nimapng on 7/2/21.
//

#ifndef POPLARDIGIT_TSC_H
#define POPLARDIGIT_TSC_H

#include "Controller/Controller.h"

#include "TaskSpaceControl.h"
#include "Task/SE3MotionTask.h"
#include "Task/RegularizationTask.h"
#include "Constraints/ContactPointsConstraints.h"
#include "Constraints/ContactForceConstraints.h"
#include "Constraints/ActuatorLimit.h"
#include "Task/CoMMotionTask.h"
#include "Constraints/QaccBound.h"
#include "Task/JointsNominalTask.h"
#include "Task/AngularMomentumTask.h"

using namespace TSC;

class TSC_IMPL : public Controller {
public:
    TSC_IMPL(string urdf_file);

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
    CoMMotionTask *com;
    RegularizationTask *rt;
    JointsNominalTask *jointsNominalTask;
    AngularMomentumTask *angularMomentumTask;
    ContactPointsConstraints *cpcstr;
    ContactForceConstraints *cfcstr;
    ActuatorLimit *actuatorLimit;
    QaccBound *qaccBound;
    TaskSpaceControl *tsc;

    JointsCmd _jointsCmd;

};


#endif //POPLARDIGIT_TSC_H
