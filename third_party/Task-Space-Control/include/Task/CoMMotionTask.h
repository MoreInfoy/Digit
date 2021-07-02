//
// Created by nimapng on 6/11/21.
//

#ifndef TASKSPACECONTROL_COMMOTIONTASK_H
#define TASKSPACECONTROL_COMMOTIONTASK_H

#include "Task.h"

namespace TSC {
    class CoMMotionTask : public Task {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CoMMotionTask(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        Mat3Ref Kp();

        Mat3Ref Kd();

        Vec3Ref posRef();

        Vec3Ref velRef();

        Vec3Ref accRef();

        Mat3Ref weightMatrix();

    private:
        Mat _H;
        Mat3 _Kp, _Kd, _Q;
        Vec _g;
        Vec3 _posRef, _velRef, _accRef, acc_fb; // acc_fb , feedback term
    };
}


#endif //TASKSPACECONTROL_COMMOTIONTASK_H
