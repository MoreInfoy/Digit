//
// Created by nimapng on 7/1/21.
//

#ifndef POPLAR_ANGULARMOMENTUMTASK_H
#define POPLAR_ANGULARMOMENTUMTASK_H

#include "TaskSpaceControl/Task/Task.h"

namespace TSC {
    class AngularMomentumTask : public Task {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        AngularMomentumTask(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        Mat3Ref Kp();

        Vec3Ref ref();

        Vec3Ref ref_dot();

        Mat3Ref weightMatrix();

    private:
        Mat _H;
        Mat3 _Kp,  _Q;
        Vec _g;
        Vec3 _ref, _ref_dot;
    };
}


#endif //POPLAR_ANGULARMOMENTUMTASK_H
