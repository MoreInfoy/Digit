//
// Created by nimpng on 7/10/21.
//

#ifndef POPLARDIGIT_CLOSEDCHAINSCONSTRAINTS_H
#define POPLARDIGIT_CLOSEDCHAINSCONSTRAINTS_H

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

namespace TSC {
    class ClosedChainsConstraints : public LinearConstraints {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit ClosedChainsConstraints(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef C();

        virtual ConstVecRef c_lb();

        virtual ConstVecRef c_ub();

    private:
        Mat _C;
        Vec _c_lb, _c_ub;
    };
}


#endif //POPLARDIGIT_CLOSEDCHAINSCONSTRAINTS_H
