//
// Created by nimpng on 6/10/21.
//

#ifndef TASKSPACECONTROL_REGULARIZATIONTASK_H
#define TASKSPACECONTROL_REGULARIZATIONTASK_H

#include "Task.h"

namespace TSC {
    class RegularizationTask : public Task {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RegularizationTask(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef H();

        virtual ConstVecRef g();

        Mat3 &forceWeight();

        MatRef qaccWeight();

        RealNum& constraintForceWeight();

    private:
        Mat _H, _Q, _Q_qacc;
        Mat3 _Q_f;
        Vec _g;
        double _Q_cstrf;
        bool _Q_isUpdated;
    };
}


#endif //TASKSPACECONTROL_REGULARIZATIONTASK_H
