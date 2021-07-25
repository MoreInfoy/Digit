//
// Created by nimapng on 6/11/21.
//

#ifndef TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H
#define TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

namespace TSC {
    class ContactPointsConstraints : public LinearConstraints {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit ContactPointsConstraints(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef C();

        virtual ConstVecRef c_lb();

        virtual ConstVecRef c_ub();

    private:
        Mat _C;
        Vec _c_lb, _c_ub;
    };
}


#endif //TASKSPACECONTROL_CONTACTPOINTSCONSTRAINTS_H
