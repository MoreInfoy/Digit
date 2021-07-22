//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_CONSTRAINTS_H
#define TASK_SPACE_CONTROL_CONSTRAINTS_H

#include "TaskSpaceControl/ConfigurationTSC.h"
#include "TaskSpaceControl/TypeDefinition.h"
#include "RobotWrapper/RobotWrapper.h"

namespace TSC {
    class LinearConstraints {
    public:
        explicit LinearConstraints(RobotWrapper &robot, string name, bool isEqual = false);

        virtual void update() = 0;

        virtual ConstMatRef C() = 0;

        virtual ConstVecRef c_lb() = 0;

        virtual ConstVecRef c_ub() = 0;

        virtual const string &name();

        virtual bool isEqual();

        virtual void errPrint(ConstVecRef u);

        RobotWrapper &robot();

    protected:
        RobotWrapper &_robot;
        bool _isEqual;

    private:
        string _name;
    };
}


#endif //TASK_SPACE_CONTROL_CONSTRAINTS_H
