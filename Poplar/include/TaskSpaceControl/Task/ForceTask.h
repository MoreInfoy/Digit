//
// Created by nimpng on 8/7/21.
//

#ifndef POPLARDIGIT_FORCETASK_H
#define POPLARDIGIT_FORCETASK_H

#include "TaskSpaceControl/Task/Task.h"

namespace TSC {
    class ForceTask : public Task {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ForceTask(RobotWrapper &robot, string name);

        void update() override;

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        Mat3Ref weightMatrix();

        void setForceRef(ConstVecRef forceRef);

    private:
        Mat _H, _Q;
        Mat3 _Qf;
        Vec _g, _forceRef;
    };
}


#endif //POPLARDIGIT_FORCETASK_H
