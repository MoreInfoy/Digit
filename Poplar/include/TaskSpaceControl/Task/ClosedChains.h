//
// Created by nimapng on 8/23/21.
//

#ifndef TASKSPACECONTROL_CLOSEDCHAINS_H
#define TASKSPACECONTROL_CLOSEDCHAINS_H

#include "TaskSpaceControl/Task/Task.h"

namespace TSC {
    class ClosedChains : public Task {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ClosedChains(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef H();

        virtual ConstVecRef g();

    private:
        Mat _H;
        Vec _g;
        Vec _acc_fb; // acc_fb , feedback term
    };
}
#endif //POPLARDIGIT_CLOSEDCHAINS_H
