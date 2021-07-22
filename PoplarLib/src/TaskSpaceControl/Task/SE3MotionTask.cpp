//
// Created by nimapng on 6/9/21.
//

#include "TaskSpaceControl/Task/SE3MotionTask.h"

using namespace TSC;

SE3MotionTask::SE3MotionTask(RobotWrapper &robot, string name) : Task(robot, name)
{
    _SE3Ref.setIdentity();
    _spatialVelRef.setZero();
    _spatialAccRef.setZero();
    _Kp.setZero();
    _Kd.setZero();
    _Q.setIdentity();
}

void SE3MotionTask::update()
{
    int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    Mat S;
    S.resize(robot().nv(), input_dims);
    S.setZero();
    S.leftCols(robot().nv()).setIdentity();

    robot().Jacobia_local(Task::name(), J);

    _SE3 = robot().frame_pose(Task::name());
    acc_fb = _Kp * log6(_SE3.actInv(_SE3Ref)).toVector() +
             _Kd * (_spatialVelRef - robot().frame_6dVel_local(Task::name()).toVector()) + _spatialAccRef;

    A = J * S;
    a = acc_fb - robot().frame_6dAcc_local(Task::name()).toVector();
    _H = A.transpose() * _Q * A;
    _g = -A.transpose() * _Q * a;
/*     std::cout << "acc_fb:\n"
              << acc_fb.transpose() << std::endl;
    std::cout << "J*qdot\n"
              << J * robot().qdot() << std::endl;
    std::cout << "Jdot\n"
              << Jdot << std::endl;
    std::cout << "SE3\n"
              << _SE3 << std::endl;
    std::cout << "SE3Ref\n"
              << _SE3Ref << std::endl; */
}

ConstMatRef SE3MotionTask::H()
{
    return ConstMatRef(_H);
}

ConstVecRef SE3MotionTask::g()
{
    return ConstVecRef(_g);
}

Mat6Ref SE3MotionTask::Kp()
{
    return TSC::Mat6Ref(_Kp);
}

Mat6Ref SE3MotionTask::Kd()
{
    return TSC::Mat6Ref(_Kd);
}

pin::SE3 &SE3MotionTask::SE3Ref()
{
    return ref(_SE3Ref);
}

Vec6Ref SE3MotionTask::spatialVelRef()
{
    return TSC::Vec6Ref(_spatialVelRef);
}

Vec6Ref SE3MotionTask::spatialAccRef()
{
    return TSC::Vec6Ref(_spatialAccRef);
}

RealNum SE3MotionTask::cost(ConstVecRef optimal_u)
{
    return 0.5 * (optimal_u.transpose() * _H * optimal_u + _g.transpose() * optimal_u)[0];
}

Vec6 SE3MotionTask::error(ConstVecRef optimal_u)
{
    return (A * optimal_u - a);
}

Mat6Ref SE3MotionTask::weightMatrix()
{
    return TSC::Mat6Ref(_Q);
}
