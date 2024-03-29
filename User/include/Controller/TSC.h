//
// Created by nimapng on 7/2/21.
//

#ifndef POPLARDIGIT_TSC_H
#define POPLARDIGIT_TSC_H

#include "Controller/Controller.h"
#include "Poplar.h"
#include <shared_mutex>

using namespace TSC;

class TSC_IMPL : public Controller
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TSC_IMPL(RobotWrapper& robot);

    ~TSC_IMPL();

    void setContactVirtualLink(vector<string> &contact_virtual_link);

    virtual void run(size_t iter, const RobotState &state, const GaitData &gaitData, const Tasks &tasks);

    virtual const JointsCmd &jointsCmd();

    RobotWrapper &robot()
    {
        return ref(_robot);
    }

    ConstVecRef getOptimalContactForce()
    {
        return tsc->getOptimalContactForce();
    }

    ConstVecRef getOptimalQacc()
    {
        return tsc->getOptimalQacc();
    }

private:
    ConstVecRef getOptimalTorque()
    {
        return tsc->getOptimalTorque();
    }

    RobotWrapper &_robot;
    Vec _lb, _ub;
    shared_ptr<SE3MotionTask> mt_waist;
    shared_ptr<SE3MotionTask> rf;
    shared_ptr<SE3MotionTask> lf;
    shared_ptr<CoMMotionTask> com;
    shared_ptr<RegularizationTask> rt;
    shared_ptr<ForceTask> forceTask;
    shared_ptr<ClosedChains> closedChainsTask;
    shared_ptr<JointsNominalTask> jointsNominalTask;
    shared_ptr<AngularMomentumTask> angularMomentumTask;
    shared_ptr<ContactPointsConstraints> cpcstr;
    shared_ptr<ContactForceConstraints> cfcstr;
    shared_ptr<ClosedChainsConstraints> closedChainsConstraints;
    shared_ptr<ActuatorLimit> actuatorLimit;
    shared_ptr<TaskSpaceControl> tsc;

    vector<string> contact_virtual_link;
    vector<pair<string, string>> link_pairs;
    vector<pair<string, Scalar>> spring_joints;

    JointsCmd _jointsCmd;
    size_t _iter;
};

#endif //POPLARDIGIT_TSC_H
