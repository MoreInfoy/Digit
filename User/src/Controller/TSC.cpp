//
// Created by nimapng on 7/2/21.
//

#include "Controller/TSC.h"
#include "Timer.h"

using namespace TSC;

TSC_IMPL::TSC_IMPL(RobotWrapper &robot) : _robot(robot), _iter(0)
{
    if (_robot.isFixedBase())
    {
        Vec qpos(_robot.nq()), qvel(_robot.nv());
        qpos = _robot.homeConfigurations().tail(_robot.na());
        qvel.setZero();
        _robot.update(qpos, qvel);
    }
    else
    {
        Vec qpos(_robot.nq()), qvel(_robot.nv());
        qpos = _robot.homeConfigurations();
        qvel.setZero();
        _robot.update(qpos, qvel);
    }

    /*mt_waist = make_shared<SE3MotionTask>(_robot, "torso");
    mt_waist->Kp().diagonal() << 0, 0, 0, 400, 400, 500;
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
    com->accRef().setZero();*/

    mt_waist = make_shared<SE3MotionTask>(_robot, "torso");
    mt_waist->Kp().diagonal() << 200, 200, 400, 400, 400, 500;
    mt_waist->Kd() = 2 * mt_waist->Kp().cwiseSqrt();
    mt_waist->weightMatrix().diagonal() << 1000, 1000, 1000, 500, 500, 500;
    mt_waist->SE3Ref() = _robot.frame_pose("torso");

    forceTask = make_shared<ForceTask>(_robot, "ForceTask");
    forceTask->weightMatrix().diagonal().fill(5);

    closedChainsTask = make_shared<ClosedChains>(_robot, "ClosedChains");

    rt = make_shared<RegularizationTask>(_robot, "RegularizationTask");
    rt->qaccWeight().diagonal().fill(1e-5);
    rt->forceWeight().diagonal().fill(1e-8);

    jointsNominalTask = make_shared<JointsNominalTask>(_robot, "JointsNominalTask");
    jointsNominalTask->weightMatrix().setIdentity();
    jointsNominalTask->weightMatrix().diagonal().segment(9, 4).fill(40);
    jointsNominalTask->weightMatrix().diagonal().segment(22, 4).fill(40);
    jointsNominalTask->Kp().setIdentity();
    jointsNominalTask->Kp() = 50 * jointsNominalTask->Kp();
    jointsNominalTask->Kd() = 2 * jointsNominalTask->Kp().cwiseSqrt();
    jointsNominalTask->norminalPosition() = _robot.homeConfigurations().tail(_robot.na());

    angularMomentumTask = make_shared<AngularMomentumTask>(_robot, "AngularMomentumTask");
    angularMomentumTask->weightMatrix().diagonal().fill(1);
    angularMomentumTask->Kp().diagonal().fill(20);
    angularMomentumTask->ref().setZero();
    angularMomentumTask->ref_dot().setZero();

    rf = make_shared<SE3MotionTask>(_robot, "right_toe_roll");
    rf->Kp().diagonal() << 100, 100, 100, 500, 500, 500;
    rf->Kd() = 2 * rf->Kp().cwiseSqrt();
    rf->weightMatrix().diagonal() << 500, 500, 1000, 2000, 2000, 2500;
    rf->SE3Ref() = _robot.frame_pose("right_toe_roll");
    lf = make_shared<SE3MotionTask>(_robot, "left_toe_roll");
    lf->Kp().diagonal() << 100, 100, 100, 500, 500, 500;
    lf->Kd() = 2 * lf->Kp().cwiseSqrt();
    lf->weightMatrix().diagonal() << 500, 500, 1000, 2000, 2000, 2500;
    lf->SE3Ref() = _robot.frame_pose("left_toe_roll");

    cpcstr = make_shared<ContactPointsConstraints>(_robot, "cpcstr");
    cfcstr = make_shared<ContactForceConstraints>(_robot, "cfcstr");
    cfcstr->mu() = 0.6;
    cfcstr->max() = 500;
    closedChainsConstraints = make_shared<ClosedChainsConstraints>(_robot, "ClosedChainsConstraints");
    actuatorLimit = make_shared<ActuatorLimit>(_robot, "ActuatorLimit");

    tsc = make_shared<TaskSpaceControl>(_robot);
    if (_robot.isFixedBase())
    {
        tsc->addTask(lf);
        tsc->addTask(rf);
    }
    else
    {
        tsc->addTask(mt_waist);
        //        tsc->addTask(com);
        tsc->addTask(angularMomentumTask);
        tsc->addTask(forceTask);
        // tsc->addLinearConstraint(cpcstr);
        tsc->addLinearConstraint(cfcstr);
        tsc->addTask(lf);
        tsc->addTask(rf);
    }
    // tsc->addLinearConstraint(closedChainsConstraints);
    tsc->addTask(rt);
    tsc->addTask(closedChainsTask);
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

TSC_IMPL::~TSC_IMPL()
{
}

void TSC_IMPL::setContactVirtualLink(vector<string> &contact_virtual_link)
{
    _robot.setContactVirtualLink(contact_virtual_link);
}

void TSC_IMPL::run(size_t iter, const RobotState &state, const GaitData &gaitData, const Tasks &tasks)
{

    /*Vec qpos(robot().nq());
    Vec qvel(robot().nv());

    qpos
            << 10.4164, 0.0613067, 0.895519, 0.00133029, -0.00591213, -0.00462753, 0.999971, 0.375514, 0.0330204, 0.0124171, -0.0508007, 0.0641627, 0.117571, -0.0644452, -0.105817, 0.0254804, -0.306858, 0.96581, -0.0379381, 0.0209151, -0.356968, -0.0253625, -0.263522, 0.049312, -0.0587092, 0.094606, -0.107737, -0.115112, 0.00996371, 0.309939, -0.983794, -0.0289673, 0.0219756;
    qvel
            << 0.503494, 0.162463, -0.0340758, 0.0124457, -0.00742559, -0.150331, 0.12542, 0.649725, 1.99746, 2.25021, -0.99527, 6.75671, 0.452258, -4.0853, 4.97922, 0.0100622, 0.105891, -0.00233009, 0.000319363, 0.0592553, -0.245543, 0.851314, 1.69952, -0.144687, 2.18067, -1.13674, -0.440538, -1.13337, 0.00593312, 0.0761187, -0.00218185, 0.00044268;
    robot().update(qpos, qvel);*/

    VecXi mask = VecXi::Ones(8);

    /* if (gaitData.stanceTimeRemain[0] > 0)
    {
        lf->Kp().diagonal() << 10, 10, 10, 500, 500, 500;
        lf->Kd() = 0.5 * lf->Kp().cwiseSqrt();
        lf->weightMatrix().diagonal() << 500, 500, 1000, 20000, 20000, 25000;
    }
    else
    {
        lf->Kp().diagonal() << 100, 100, 100, 500, 500, 500;
        lf->Kd() = 2 * lf->Kp().cwiseSqrt();
        lf->weightMatrix().diagonal() << 500, 500, 1000, 2000, 2000, 2500;
    }
    if (gaitData.stanceTimeRemain[1] > 0)
    {
        rf->Kp().diagonal() << 10, 10, 10, 500, 500, 500;
        rf->Kd() = 0.5 * rf->Kp().cwiseSqrt();
        rf->weightMatrix().diagonal() << 500, 500, 1000, 20000, 20000, 25000;
    }
    else
    {
        rf->Kp().diagonal() << 100, 100, 100, 500, 500, 500;
        rf->Kd() = 2 * rf->Kp().cwiseSqrt();
        rf->weightMatrix().diagonal() << 500, 500, 1000, 2000, 2000, 2500;
    } */

    auto lf_vel = _robot.frame_6dVel_local(lf->name());
    lf->SE3Ref().translation() = tasks.leftFootTask.pos;
    lf->spatialVelRef() << tasks.leftFootTask.vel, tasks.leftFootTask.omega;
    lf->spatialAccRef()
        << tasks.leftFootTask.acc - lf_vel.angular().cross(lf_vel.linear()),
        tasks.leftFootTask.omega_dot; // TODO: analytical acc to spatial acc
    if (!tsc->existTask(lf->name()))
    {
        tsc->addTask(lf);
    }

    auto rf_vel = _robot.frame_6dVel_local(rf->name());
    rf->SE3Ref().translation() = tasks.rightFootTask.pos;
    rf->spatialVelRef() << tasks.rightFootTask.vel, tasks.rightFootTask.omega;
    rf->spatialAccRef()
        << tasks.rightFootTask.acc - rf_vel.angular().cross(rf_vel.linear()),
        tasks.rightFootTask.omega_dot; // TODO: analytical acc to spatial acc
    if (!tsc->existTask(rf->name()))
    {
        tsc->addTask(rf);
    }

    /*lf->SE3Ref().translation() << 10.4637, 0.150714, 0.0611726;
    lf->spatialVelRef().setZero();
    lf->spatialAccRef().setZero();
    if (!tsc->existTask(lf->name())) {
        tsc->addTask(lf);
    }

    rf->SE3Ref().translation() << 10.2821, -0.0444303, 0.0617226;
    rf->spatialVelRef() << -0.0492134, -0.0373673, 0.254207, 0, 0, 0;
    rf->spatialAccRef() << -3.39086, -2.19233, 17.2868, 0, 0, 0;
    if (!tsc->existTask(rf->name())) {
        tsc->addTask(rf);
    }*/

    if (gaitData.swingTimeRemain(0) > 0)
    {
        mask.head(4).setZero();
    }

    if (gaitData.swingTimeRemain(1) > 0)
    {
        mask.tail(4).setZero();
    }

    if (_robot.isFixedBase())
    {
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
    }
    else
    {
        auto base_frame = robot().frame_pose("torso");
        auto base_vel = robot().frame_6dVel_local("torso");

        //        com->posRef().y() = 0.05 * sin(0.004 * _iter);

        /*com->posRef() = tasks.floatingBaseTask.pos;
        com->velRef() = tasks.floatingBaseTask.vel;
        com->accRef() = tasks.floatingBaseTask.acc;*/
        mt_waist->SE3Ref().translation() = tasks.floatingBaseTask.pos;
        mt_waist->spatialVelRef().head(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.vel;
        mt_waist->spatialAccRef().head(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.acc - base_vel.angular().cross(base_vel.linear());
        mt_waist->SE3Ref().rotation() = tasks.floatingBaseTask.R_wb;
        mt_waist->spatialVelRef().tail(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.omega;
        mt_waist->spatialAccRef().tail(3) = base_frame.rotation().transpose() * tasks.floatingBaseTask.omega_dot;
        forceTask->setForceRef(tasks.forceTask);

        /*mt_waist->SE3Ref().translation() << 10.4403, 0.0697616, 0.892442;
        mt_waist->spatialVelRef() << 0.490221, 0.17602, -0.0062687, 0, 0, 0;
        mt_waist->spatialAccRef() << -0.662789, -0.423326, 0.00897869, 0, 0, 0;
        mt_waist->SE3Ref().rotation().setIdentity();
        Vec forceRef(24);
        forceRef
                << -8.81236, -4.10457, 269.759, -1.12811e-14, 1.83768e-14, -2.61583e-16, -8.15895, -7.38375, 223.965, -1.45141e-14, -4.69346e-14, -4.14198e-15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        forceTask->setForceRef(tasks.forceTask);*/
    }
    //    mask.setZero();
    //    mask.head(4).setOnes();
    _robot.compute(mask);

    Timer timer;
    Mat T, T_dot;
    Vec par;
    T.resize(link_pairs.size() * 6, link_pairs.size());
    T.setZero();
    T_dot = T;
    for (int i = 0; i < link_pairs.size(); i++)
    {
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

    cout << "optimal force: " << getOptimalContactForce().transpose() << endl;

    /*cout << "-------------------- TSC Reference ------------------------" << endl
         << "contact mask: " << mask.transpose() << endl
         << "base pose: \n" << mt_waist->SE3Ref() << endl
         << "base vel: \n" << mt_waist->spatialVelRef() << endl
         << "base acc: \n" << mt_waist->spatialAccRef() << endl
         << "lf pose: \n" << lf->SE3Ref() << endl
         << "rf pose: \n" << rf->SE3Ref() << endl
         << "lf vel: \n" << lf->spatialVelRef().transpose() << endl
         << "rf vel: \n" << rf->spatialVelRef().transpose() << endl
         << "lf acc: \n" << lf->spatialAccRef().transpose() << endl
         << "rf acc: \n" << rf->spatialAccRef().transpose() << endl
         << "force: \n" << tasks.forceTask.transpose() << endl

         << "-------------------- TSC State ------------------------" << endl
         << "qpos: " << _robot.qpos().transpose() << endl
         << "qvel: \n" << _robot.qvel().transpose() << endl;*/
    /*<< "-------------------- TSC Closed Chains ------------------------" << endl
    << "T: \n" << T << endl
    << "T_dot: \n" << T_dot << endl;*/

    ConstMatRef N = _robot.constraintForceJacobia();
    Mat NMinvNT, NMinvNT_inv, N_Proj;
    NMinvNT.noalias() = N * N.transpose();
    NMinvNT_inv.noalias() = NMinvNT.inverse();
    N_Proj = N.transpose() * NMinvNT_inv * N;

    string foot_links[2] = {lf->name(), rf->name()};
    Mat6x J(6, _robot.nv());
    Mat Js(12, _robot.nv());

    Vec p_err(12), v_err(12);
    for (int i = 0; i < 2; i++)
    {
        LinkTask foot_task = tasks.leftFootTask;
        if (i == 1)
        {
            foot_task = tasks.rightFootTask;
        }
        _robot.Jacobia_local(foot_links[i], J);
        J.leftCols(6).setZero();
        Js.middleRows(6 * i, 6) = J;

        pin::SE3 foot, foot_des;
        foot_des.translation() = foot_task.pos;
        foot_des.rotation() = foot_task.R_wb;
        foot = _robot.frame_pose(foot_links[i]);
        p_err.segment(6 * i, 6) = log6(foot.actInv(foot_des)).toVector();

        Vec6 vel_ref;
        vel_ref << foot_task.vel, foot_task.omega;
        v_err.segment(6 * i, 6) = vel_ref - _robot.frame_6dVel_local(foot_links[i]).toVector();
    }

    Mat JJT, JJT_inv, J_pinv, par1;
    JJT.noalias() = Js * Js.transpose();
    JJT_inv.noalias() = JJT.inverse();
    J_pinv = Js.transpose() * JJT_inv;

    par1.noalias() = Mat::Identity(_robot.nv(), _robot.nv()) - N_Proj;
    Vec delta_qpos = par1 * J_pinv * p_err;
    Vec delta_qvel = par1 * J_pinv * v_err;

    Vec force_fb = 200 * delta_qpos + 28.5 * delta_qvel;
    _jointsCmd.tau_ff += force_fb.tail(_robot.na());
    _iter++;
}

const JointsCmd &TSC_IMPL::jointsCmd()
{
    return _jointsCmd;
}
