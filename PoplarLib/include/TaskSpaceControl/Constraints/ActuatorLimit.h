//
// Created by nimapng on 6/29/21.
//

#ifndef TASKSPACECONTROL_ACTUATORLIMIT_H
#define TASKSPACECONTROL_ACTUATORLIMIT_H

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

namespace TSC {
    class ActuatorLimit : public LinearConstraints {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit ActuatorLimit(RobotWrapper &robot, string name);

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


#endif //TASKSPACECONTROL_ACTUATORLIMIT_H
