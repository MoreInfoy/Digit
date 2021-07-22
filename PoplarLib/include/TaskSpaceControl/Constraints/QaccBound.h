//
// Created by nimapng on 6/29/21.
//

#ifndef TASKSPACECONTROL_QACCBOUND_H
#define TASKSPACECONTROL_QACCBOUND_H

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

namespace TSC
{
    class QaccBound: public LinearConstraints{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit QaccBound(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef C();

        virtual ConstVecRef c_lb();

        virtual ConstVecRef c_ub();

        VecRef lb();

        VecRef ub();

    private:
        Mat _C;
        Vec _c_lb, _c_ub;
        Vec _lb, _ub;
    };
}



#endif //TASKSPACECONTROL_QACCBOUND_H
