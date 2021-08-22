//
// Created by nimapng on 7/2/21.
//

#include "Controller/TSC.h"
#include "Timer.h"

using namespace TSC;

TSC_IMPL::TSC_IMPL(RobotWrapper &robot) : _robot(robot), _iter(0) {
    if (_robot.isFixedBase()) {
        Vec qpos(_robot.nq()), qvel(_robot.nv());
        qpos = _robot.homeConfigurations().tail(_robot.na());
        qvel.setZero();
        _robot.update(qpos, qvel);
    } else {
        Vec qpos(_robot.nq()), qvel(_robot.nv());
        qpos = _robot.homeConfigurations();
        qvel.setZero();
        _robot.update(qpos, qvel);
    }

    mt_waist = make_shared<SE3MotionTask>(_robot, "torso");
    mt_waist->Kp().diagonal() << 0, 0, 0, 200, 200, 500;
    mt_waist->Kd() = 2 * mt_waist->Kp().cwiseSqrt();
    mt_waist->weightMatrix() = 500 * mt_waist->weightMatrix();
    mt_waist->weightMatrix().diagonal().head(3).setZero();
    mt_waist->SE3Ref() = _robot.frame_pose("torso");

    com = make_shared<CoMMotionTask>(_robot, "com");
    com->weightMatrix() = 1000 * com->weightMatrix();
    com->Kp().diagonal() << 800, 800, 400;
    com->Kd() = 2 * com->Kp().cwiseSqrt();
    com->posRef() = _robot.CoM_pos();
    com->velRef().setZero();
    com->accRef().setZero();

    forceTask = make_shared<ForceTask>(_robot, "ForceTask");
    forceTask->weightMatrix().diagonal().fill(1e2);

    rt = make_shared<RegularizationTask>(_robot, "RegularizationTask");
    rt->qaccWeight().diagonal().fill(1e-5);
    rt->forceWeight().diagonal().fill(1e-8);

    jointsNominalTask = make_shared<JointsNominalTask>(_robot, "JointsNominalTask");
    jointsNominalTask->weightMatrix().setIdentity();
    jointsNominalTask->weightMatrix().diagonal().segment(9, 4).fill(100);
    jointsNominalTask->weightMatrix().diagonal().segment(22, 4).fill(100);
    jointsNominalTask->Kp().setIdentity();
    jointsNominalTask->Kp() = 50 * jointsNominalTask->Kp();
    jointsNominalTask->Kd() = 2 * jointsNominalTask->Kp().cwiseSqrt();
    jointsNominalTask->norminalPosition() = _robot.homeConfigurations().tail(_robot.na());

    angularMomentumTask = make_shared<AngularMomentumTask>(_robot, "AngularMomentumTask");
    angularMomentumTask->weightMatrix().diagonal().fill(1);
    angularMomentumTask->Kp().diagonal().fill(50);
    angularMomentumTask->ref().setZero();
    angularMomentumTask->ref_dot().setZero();

    rf = make_shared<SE3MotionTask>(_robot, "right_toe_roll");
    rf->Kp().diagonal() << 100, 100, 100, 200, 200, 200;
    rf->Kd() = 2 * rf->Kp().cwiseSqrt();
    rf->weightMatrix().diagonal() << 500, 500, 1000, 1000, 1000, 500;
    rf->SE3Ref() = _robot.frame_pose("right_toe_roll");
    lf = make_shared<SE3MotionTask>(_robot, "left_toe_roll");
    lf->Kp().diagonal() << 100, 100, 100, 200, 200, 200;
    lf->Kd() = 2 * lf->Kp().cwiseSqrt();
    lf->weightMatrix().diagonal() << 500, 500, 1000, 1000, 1000, 500;
    lf->SE3Ref() = _robot.frame_pose("left_toe_roll");

    cpcstr = make_shared<ContactPointsConstraints>(_robot, "cpcstr");
    cfcstr = make_shared<ContactForceConstraints>(_robot, "cfcstr");
    closedChainsConstraints = make_shared<ClosedChainsConstraints>(_robot, "ClosedChainsConstraints");
    actuatorLimit = make_shared<ActuatorLimit>(_robot, "ActuatorLimit");

    tsc = make_shared<TaskSpaceControl>(_robot);
    if (_robot.isFixedBase()) {
        tsc->addTask(lf);
        tsc->addTask(rf);
    } else {
        tsc->addTask(mt_waist);
        tsc->addTask(com);
        tsc->addTask(angularMomentumTask);
        tsc->addTask(forceTask);
//        tsc->addLinearConstraint(cpcstr);
        tsc->addLinearConstraint(cfcstr);
        tsc->addTask(lf);
        tsc->addTask(rf);
    }
    tsc->addLinearConstraint(closedChainsConstraints);
    tsc->addTask(rt);
    tsc->addTask(jointsNominalTask);
    tsc->addLinearConstraint(actuatorLimit);

    contact_virtual_link.emplace_back("contact1");
    contact_virtual_link.emplace_back("contact2");
    contact_virtual_link.emplace_back("contact3");
    contact_virtual_link.emplace_back("contact4");
    contact_virtual_link.emplace_back("contact5");
    contact_virtual_link.emplace_back("contact6");
    contact_virtual_link.emplace_back("contact7");
    contact_virtual_link.emplace_back("contact8");
    _robot.setContactVirtualLink(contact_virtual_link);

    link_pairs.push_back(pair<string, string>("cp_left_achillies_rod", "cp_left_heel_spring"));
    link_pairs.push_back(pair<string, string>("cp_right_achillies_rod", "cp_right_heel_spring"));
    link_pairs.push_back(pair<string, string>("cp_left_toe_A_rod", "cp_left_toe_roll_A"));
    link_pairs.push_back(pair<string, string>("cp_left_toe_B_rod", "cp_left_toe_roll_B"));
    link_pairs.push_back(pair<string, string>("cp_right_toe_A_rod", "cp_right_toe_roll_A"));
    link_pairs.push_back(pair<string, string>("cp_right_toe_B_rod", "cp_right_toe_roll_B"));
    _robot.setConnectVirtualLink(link_pairs);

    /*spring_joints.push_back(pair<string, Scalar>("left_shin_joint", 600.0));
    spring_joints.push_back(pair<string, Scalar>("left_heel_spring_joint", 437.5));
    spring_joints.push_back(pair<string, Scalar>("right_shin_joint", 600.0));
    spring_joints.push_back(pair<string, Scalar>("right_heel_spring_joint", 437.5));
    _robot.setSpringJoints(spring_joints);*/

    cout << "tau limit: " << _robot.actuatorsEffortLimit().transpose() << endl;
}

TSC_IMPL::~TSC_IMPL() {
}

void TSC_IMPL::setContactVirtualLink(vector<string> &contact_virtual_link) {
    _robot.setContactVirtualLink(contact_virtual_link);
}

void TSC_IMPL::run(size_t iter, const RobotState &state, const GaitData &gaitData, const Tasks &tasks) {

    VecXi mask = VecXi::Ones(8);

    if (gaitData.swingTimeRemain(0) > 0) {
//        lf->SE3Ref().rotation() = tasks.leftFootTask.R_wb;
        lf->SE3Ref().translation() = tasks.leftFootTask.pos;
        lf->spatialVelRef() << tasks.leftFootTask.vel, tasks.leftFootTask.omega;
        lf->spatialAccRef()
                << tasks.leftFootTask.acc,
                tasks.leftFootTask.omega_dot; // TODO: analytical acc to spatial acc
        if (!tsc->existTask(lf->name())) {
            tsc->addTask(lf);
        }
        lf->Kp().diagonal() << 100, 100, 100, 200, 200, 200;
        lf->Kd() = 2 * lf->Kp().cwiseSqrt();
        lf->weightMatrix().diagonal() << 500, 500, 1000, 5000, 5000, 500;
        mask.head(4).setZero();
    } else {
        lf->SE3Ref().translation() = tasks.leftFootTask.pos;
        lf->spatialVelRef() << tasks.leftFootTask.vel, tasks.leftFootTask.omega;
        lf->spatialAccRef()
                << tasks.leftFootTask.acc,
                tasks.leftFootTask.omega_dot; // TODO: analytical acc to spatial acc
        if (!tsc->existTask(lf->name())) {
            tsc->addTask(lf);
        }
        lf->Kp().diagonal() << 10, 10, 10, 20, 20, 20;
        lf->Kd() = 2 * lf->Kp().cwiseSqrt();
        lf->weightMatrix().diagonal() << 500, 500, 1000, 5000, 5000, 500;
//        tsc->removeTask(tasks.leftFootTask.link_name);
    }

    if (gaitData.swingTimeRemain(1) > 0) {
//        rf->SE3Ref().rotation() = tasks.rightFootTask.R_wb;
        rf->SE3Ref().translation() = tasks.rightFootTask.pos;
        rf->spatialVelRef() << tasks.rightFootTask.vel, tasks.rightFootTask.omega;
        rf->spatialAccRef()
                << tasks.rightFootTask.acc,
                tasks.rightFootTask.omega_dot; // TODO: analytical acc to spatial acc
        if (!tsc->existTask(rf->name())) {
            tsc->addTask(rf);
        }
        rf->Kp().diagonal() << 100, 100, 100, 10, 10, 200;
        rf->Kd() = 2 * rf->Kp().cwiseSqrt();
        rf->weightMatrix().diagonal() << 500, 500, 1000, 1000, 1000, 500;
        mask.tail(4).setZero();
    } else {
        rf->SE3Ref().translation() = tasks.rightFootTask.pos;
        rf->spatialVelRef() << tasks.rightFootTask.vel, tasks.rightFootTask.omega;
        rf->spatialAccRef()
                << tasks.rightFootTask.acc,
                tasks.rightFootTask.omega_dot; // TODO: analytical acc to spatial acc
        if (!tsc->existTask(rf->name())) {
            tsc->addTask(rf);
        }
        rf->Kp().diagonal() << 10, 10, 10, 20, 20, 20;
        rf->Kd() = 2 * rf->Kp().cwiseSqrt();
        rf->weightMatrix().diagonal() << 500, 500, 1000, 1000, 1000, 500;
//        tsc->removeTask(tasks.rightFootTask.link_name);
    }

    if (_robot.isFixedBase()) {
        mask.setZero();
        _robot.setContactMask(mask);
        /*rf->SE3Ref().translation()(2) = -0.839273 + 0.10 * sin(0.004 * _iter);
        lf->SE3Ref().translation()(2) = -0.839273 - 0.10 * sin(0.004 * _iter);

        Vec3 rf_v, lf_v, rf_acc, lf_acc;
        rf_v << 0, 0, 0.4 * cos(0.004 * _iter);
        lf_v << 0, 0, -0.4 * cos(0.004 * _iter);
        rf_acc << 0, 0, -1.6 * sin(0.004 * _iter);
        lf_acc << 0, 0, 1.6 * sin(0.004 * _iter);

        rf->spatialVelRef().head(3) = robot().frame_pose(rf->name()).rotation().transpose() * rf_v;
        lf->spatialVelRef().head(3) = robot().frame_pose(lf->name()).rotation().transpose() * lf_v;
        rf->spatialAccRef().head(3) = robot().frame_pose(rf->name()).rotation().transpose() * rf_acc;
        lf->spatialAccRef().head(3) = robot().frame_pose(lf->name()).rotation().transpose() * lf_acc;*/
    } else {
        auto base_frame = robot().frame_pose("torso");
        //        com->posRef().y() = 0.05 * sin(0.004 * _iter);

        com->posRef() = tasks.floatingBaseTask.pos;
        com->velRef() = tasks.floatingBaseTask.vel;
        com->accRef() = tasks.floatingBaseTask.acc;
        /*mt_waist->SE3Ref().translation() = tasks.floatingBaseTask.pos;
        mt_waist->spatialVelRef().head(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.vel;
        mt_waist->spatialAccRef().head(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.acc;*/
        mt_waist->SE3Ref().rotation() = tasks.floatingBaseTask.R_wb;
        mt_waist->spatialVelRef().tail(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.omega;
        mt_waist->spatialAccRef().tail(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.omega_dot;
        forceTask->setForceRef(tasks.forceTask);
//        std::cout << "force ref: " << tasks.forceTask.transpose() << std::endl;
    }
//    mask.setOnes();
    _robot.compute(mask);

    Timer timer;
    Mat T, T_dot;
    Vec par;
    T.resize(link_pairs.size() * 6, link_pairs.size());
    T.setZero();
    T_dot = T;
    for (int i = 0; i < link_pairs.size(); i++) {
        auto p = _robot.frame_pose(link_pairs[i].first).translation();
        auto s = _robot.frame_pose(link_pairs[i].second).translation();
        T.block<3, 1>(i * 6, i) = p - s;
        par = _robot.connectPointRelativeJacobia().middleRows(i * 6, 6) * _robot.qvel();
        T_dot.block<3, 1>(i * 6, i) = par.head(3);
    }
    _robot.setConstraintForceSubspace(T, T_dot);
    _robot.computeClosedChainTerm();
    tsc->solve();
    cout << "time cost: " << timer.getMs() << " ms" << endl;
    _jointsCmd.tau_ff = getOptimalTorque();

    /*cout << "contact force: " << tsc->getOptimalContactForce().transpose() << endl;*/
    //    tsc->saveAllData("data.txt");
    /*auto tau = getOptimalTorque();
    _jointsCmd.tau_ff << tau.head(4), tau.segment(5, 2), tau.segment(9, 8), tau.segment(18, 2), tau.tail(4);
    cout << "optimal qacc: " << getOptimalQacc().transpose() << endl;
    cout << "optimal force: " << getOptimalContactForce().transpose() << endl;
    cout << "qpos: " << qpos.transpose() << endl;
    cout << "qvel: " << qdot.transpose() << endl;
    cout << "optimal torque: " << _jointsCmd.tau_ff.transpose() << endl;
    getchar();
    cout << "jointSpringForce:" << _robot.jointsSpringForce().transpose() << endl;*/
    _iter++;
}

const JointsCmd &TSC_IMPL::jointsCmd() {
    return _jointsCmd;
}
