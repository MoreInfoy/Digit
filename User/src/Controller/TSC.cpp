//
// Created by nimapng on 7/2/21.
//

#include "Controller/TSC.h"
#include "Timer.h"

using namespace TSC;

TSC_IMPL::TSC_IMPL(string urdf_file) : _robot(urdf_file), _iter(0) {
    mt_waist = new SE3MotionTask(_robot, "torso");
    mt_waist->Kp() = 500 * Mat6::Identity();
    mt_waist->Kd() = 2 * mt_waist->Kp().cwiseSqrt();
    mt_waist->weightMatrix() = 5000 * mt_waist->weightMatrix();
    mt_waist->SE3Ref().setIdentity();
    mt_waist->SE3Ref().translation() << 0.0269469, 1.92768e-05, 0.892442;

    com = new CoMMotionTask(_robot, "com");
    com->weightMatrix() = 100 * com->weightMatrix();
    com->Kp() = 500 * Mat3::Identity();
    com->Kd() = 2 * com->Kp().cwiseSqrt();
    com->posRef() << 0.00593849, 7.53746e-05, 0.89112;
    com->velRef().setZero();
    com->accRef().setZero();

    rt = new RegularizationTask(_robot, "RegularizationTask");
    rt->qaccWeight().diagonal().fill(1e-5);
    rt->forceWeight().diagonal().fill(1e-8);

    jointsNominalTask = new JointsNominalTask(_robot, "JointsNominalTask");
    jointsNominalTask->weightMatrix().setIdentity();
    jointsNominalTask->Kp().setIdentity();
    jointsNominalTask->Kp() = 100 * jointsNominalTask->Kp();
    jointsNominalTask->Kd().setIdentity();
    jointsNominalTask->Kd() = 5 * jointsNominalTask->Kd();
    jointsNominalTask->norminalPosition()
            << 0.325, 0, 0, 0, 0, -0.102, -0.07,
            -0.325, 0, 0, 0, 0, 0.102, 0.07,
            0, 0.987, 0, 0,
            0, -0.987, 0, 0;

    angularMomentumTask = new AngularMomentumTask(_robot, "AngularMomentumTask");
    angularMomentumTask->weightMatrix().diagonal().fill(10);
    angularMomentumTask->Kp().diagonal().fill(100);
    angularMomentumTask->ref().setZero();
    angularMomentumTask->ref_dot().setZero();

    cpcstr = new ContactPointsConstraints(_robot, "cpcstr");
    cfcstr = new ContactForceConstraints(_robot, "cfcstr");
    actuatorLimit = new ActuatorLimit(_robot, "ActuatorLimit");
    qaccBound = new QaccBound(_robot, "QaccBound");
    qaccBound->lb().fill(-100);
    qaccBound->ub().fill(100);

    tsc = new TaskSpaceControl(_robot);
    tsc->addTask(mt_waist);
    tsc->addTask(com);
    tsc->addTask(rt);
//        tsc->removeTask(rt->name());
    tsc->addTask(jointsNominalTask);
    tsc->addTask(angularMomentumTask);
    tsc->addLinearConstraint(cpcstr);
    tsc->addLinearConstraint(cfcstr);
//    tsc->addLinearConstraint(actuatorLimit);
    tsc->addLinearConstraint(qaccBound);
    //    tsc.removeLinearConstraint(ccstr.name());
}

TSC_IMPL::~TSC_IMPL() {
    delete mt_waist;
    delete com;
    delete rt;
    delete jointsNominalTask;
    delete angularMomentumTask;
    delete cpcstr;
    delete cfcstr;
    delete actuatorLimit;
    delete qaccBound;
    delete tsc;
}

void TSC_IMPL::setContactMask(const VecInt &mask) {
    _robot.setContactMask(mask);
    _mask = mask;
}

void TSC_IMPL::setContactVirtualLink(vector<string> &contact_virtual_link) {
    _robot.setContactVirtualLink(contact_virtual_link);
}

void TSC_IMPL::solve(ConstVecRef qpos, ConstVecRef qvel, const VecInt &mask) {
    _robot.computeAllData(qpos, qvel, mask);
    mt_waist->SE3Ref().translation()(2) = 0.892442 + 0.05 * sin(0.004 * _iter);
    tsc->solve();
//    cout << robot().frame_6dVel_local("torso") << endl;
//        tsc->saveAllData("data.txt");
}

void TSC_IMPL::run(const Reference &ref, const RobotState &state) {
    Vec qpos;
    qpos.resize(state.jointsState.qpos.size() + 7);
    Vec quat(4);
    quat << state.floatingBaseState.quat.x(),
            state.floatingBaseState.quat.y(),
            state.floatingBaseState.quat.z(),
            state.floatingBaseState.quat.w();
    qpos << state.floatingBaseState.pos,
            quat,
            state.jointsState.qpos;
    Vec qdot;
    qdot.resize(state.jointsState.qvel.size() + 6);

    qdot << state.floatingBaseState.quat.toRotationMatrix().transpose() * state.floatingBaseState.vel,
            state.floatingBaseState.omega, state.jointsState.qvel;

//    for (int i = 0; i < _dataSets.robotModelData.model.nv + 1; i++) {
//        printf("%f, ", qpos(i));
//    }
//    printf("\n qdot: ");
//    for (int i = 0; i < qdot.size(); i++) {
//        printf("%f, ", qdot(i));
//    }
    Timer timer;
    solve(qpos, qdot, _mask);
    cout << "time cost: " << timer.getMs() << endl;

    _jointsCmd.tau_ff = getOptimalTorque();
//    cout << "optimal qacc: " << getOptimalQacc().transpose() << endl;
//    cout << "optimal force: " << getOptimalContactForce().transpose() << endl;
//    cout << "optimal torque: " << _jointsCmd.tau_ff.transpose() << endl;
    _iter++;
}

const JointsCmd &TSC_IMPL::jointsCmd() {
    return _jointsCmd;
}
