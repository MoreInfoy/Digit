//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_TASK_H
#define TASK_SPACE_CONTROL_TASK_H

#include "TaskSpaceControl/ConfigurationTSC.h"
#include "TaskSpaceControl/TypeDefinition.h"
#include "RobotWrapper/RobotWrapper.h"

namespace TSC {
    class Task {
    public:
        explicit Task(RobotWrapper &robot, string name);

        virtual void update() = 0;

        virtual ConstMatRef H() = 0;

        virtual ConstVecRef g() = 0;

        virtual const string &name();

        RobotWrapper &robot();

    protected:
        RobotWrapper &_robot;

    private:
        string _name;
    };
}

#endif //TASK_SPACE_CONTROL_TASK_H
