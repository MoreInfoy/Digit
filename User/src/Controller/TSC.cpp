//
// Created by nimapng on 7/2/21.
//

#include "Controller/TSC.h"
#include "Timer.h"

#ifdef FIXED_BASE
bool fixedBase = true;
#else
bool fixedBase = false;
#endif
using namespace TSC;

TSC_IMPL::TSC_IMPL(string urdf_file) : _robot(urdf_file, fixedBase), _iter(0) {

    mt_waist = new SE3MotionTask(_robot, "torso");
    mt_waist->Kp() = 500 * Mat6::Identity();
    mt_waist->Kd() = 2 * mt_waist->Kp().cwiseSqrt();
    mt_waist->weightMatrix() = 5000 * mt_waist->weightMatrix();
    mt_waist->SE3Ref().setIdentity();
    if (_robot.isFixedBase()) {
        mt_waist->SE3Ref().translation() << 0.0269469, 1.92768e-05, 0.892442 - 0.9;
    } else {
        mt_waist->SE3Ref().translation() << 0.0269469, 1.92768e-05, 0.892442;
    }

    com = new CoMMotionTask(_robot, "com");
    com->weightMatrix() = 100 * com->weightMatrix();
    com->Kp() = 500 * Mat3::Identity();
    com->Kd() = 2 * com->Kp().cwiseSqrt();
    if (_robot.isFixedBase()) {
        com->posRef() << 0.00593849, 7.53746e-05, -0.15;
    } else {
        com->posRef() << 0.00593849, 7.53746e-05, 0.89112;
    }
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

    if (_robot.isFixedBase()) {
        Vec qpos(_robot.nq()), qvel(_robot.nv());
        qpos << 0.325, 0, 0, 0, 0, -0.102, -0.07,
                -0.325, 0, 0, 0, 0, 0.102, 0.07,
                0, 0.987, 0, 0,
                0, -0.987, 0, 0;
        qvel.setZero();
        _robot.computeAllData(qpos, qvel);
        rf = new SE3MotionTask(_robot, "right_toe_roll");
        rf->Kp() = 500 * Mat6::Identity();
        rf->Kd() = 2 * rf->Kp().cwiseSqrt();
        rf->weightMatrix() = 1000 * mt_waist->weightMatrix();
        rf->SE3Ref() = _robot.frame_pose("right_toe_roll");
        lf = new SE3MotionTask(_robot, "left_toe_roll");
        lf->Kp() = 500 * Mat6::Identity();
        lf->Kd() = 2 * rf->Kp().cwiseSqrt();
        lf->weightMatrix() = 1000 * mt_waist->weightMatrix();
        lf->SE3Ref() = _robot.frame_pose("left_toe_roll");
        com->posRef() = _robot.CoM_pos();
    }

    cpcstr = new ContactPointsConstraints(_robot, "cpcstr");
    cfcstr = new ContactForceConstraints(_robot, "cfcstr");
    closedChainsConstraints = new ClosedChainsConstraints(_robot, "ClosedChainsConstraints");
    actuatorLimit = new ActuatorLimit(_robot, "ActuatorLimit");
    qaccBound = new QaccBound(_robot, "QaccBound");
    qaccBound->lb().fill(-100);
    qaccBound->ub().fill(100);

    tsc = new TaskSpaceControl(_robot);
    if (!_robot.isFixedBase()) {
        tsc->addTask(mt_waist);
        tsc->addLinearConstraint(cpcstr);
        tsc->addLinearConstraint(cfcstr);
    } else {
        tsc->addTask(lf);
        tsc->addTask(rf);
    }
    tsc->addLinearConstraint(closedChainsConstraints);
    tsc->addTask(com);
    tsc->addTask(rt);
//        tsc->removeTask(rt->name());
    tsc->addTask(jointsNominalTask);
    tsc->addTask(angularMomentumTask);
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
    delete closedChainsConstraints;
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
    if (_robot.isFixedBase()) {
        rf->SE3Ref().translation()(2) = -0.839273 + 0.05 * sin(0.004 * _iter);
        lf->SE3Ref().translation()(2) = -0.839273 - 0.05 * sin(0.004 * _iter);
    } else {
        mt_waist->SE3Ref().translation()(2) = 0.892442 + 0.05 * sin(0.004 * _iter);
    }
    tsc->solve();
    cout << robot().frame_pose("right_toe_roll") << endl;
//    cout << robot().frame_6dVel_local("torso") << endl;
//        tsc->saveAllData("data.txt");
}

void TSC_IMPL::run(const Reference &ref, const RobotState &state) {
    Vec qpos, qdot;
    if (_robot.isFixedBase()) {
        qpos = state.jointsState.qpos;
        qdot = state.jointsState.qvel;
    } else {
        qpos.resize(state.jointsState.qpos.size() + 7);
        Vec quat(4);
        quat << state.floatingBaseState.quat.x(),
                state.floatingBaseState.quat.y(),
                state.floatingBaseState.quat.z(),
                state.floatingBaseState.quat.w();
        qpos << state.floatingBaseState.pos,
                quat,
                state.jointsState.qpos;
        qdot.resize(state.jointsState.qvel.size() + 6);

        qdot << state.floatingBaseState.quat.toRotationMatrix().transpose() * state.floatingBaseState.vel,
                state.floatingBaseState.omega, state.jointsState.qvel;
    }

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
