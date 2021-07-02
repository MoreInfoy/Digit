//
// Created by nimapng on 6/9/21.
//

#ifndef TASKSPACECONTROL_SE3MOTIONTASK_H
#define TASKSPACECONTROL_SE3MOTIONTASK_H

#include "Task.h"

namespace TSC {
    class SE3MotionTask : public Task {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SE3MotionTask(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        Mat6Ref Kp();

        Mat6Ref Kd();

        pin::SE3 &SE3Ref();

        Vec6Ref spatialVelRef();

        Vec6Ref spatialAccRef();

        RealNum cost(ConstVecRef optimal_u);

        Vec6 error(ConstVecRef optimal_u);

        Mat6Ref weightMatrix();

    private:
        Mat _H, A;
        Mat6x J;
        Mat6 _Kp, _Kd, _Q;
        Vec _g, a;
        pin::SE3 _SE3Ref, _SE3;
        Vec6 _spatialVel, _spatialVelRef, _spatialAccRef, acc_fb; // acc_fb , feedback term
        pin::FrameIndex frame_index;
    };
}

#endif //TASKSPACECONTROL_SE3MOTIONTASK_H
