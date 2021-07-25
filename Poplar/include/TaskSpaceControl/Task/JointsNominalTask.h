//
// Created by nimapng on 7/1/21.
//

#ifndef POPLAR_JOINTSNOMINALTASK_H
#define POPLAR_JOINTSNOMINALTASK_H

#include "TaskSpaceControl/Task/Task.h"

namespace TSC {
    class JointsNominalTask : public Task {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        JointsNominalTask(RobotWrapper &robot, string name);

        void update() override;

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        MatRef Kp();

        MatRef Kd();

        MatRef weightMatrix();

        VecRef norminalPosition();

    private:
        Mat _H, _Q, _Kp, _Kd;
        Vec _g, _q_n;
    };
}


#endif //POPLAR_JOINTNORMINALTASK_H
