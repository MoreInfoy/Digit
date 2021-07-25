//
// Created by nimapng on 6/9/21.
//

#include "TaskSpaceControl/Task/Task.h"

TSC::Task::Task(RobotWrapper &robot, string name) : _robot(robot), _name(name) {

}

const string &TSC::Task::name() {
    return _name;
}

RobotWrapper &TSC::Task::robot() {
    return _robot;
}
