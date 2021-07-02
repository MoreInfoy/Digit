//
// Created by nimapng on 6/9/21.
//

#include "Constraints/LinearConstraints.h"

TSC::LinearConstraints::LinearConstraints(RobotWrapper &robot, string name, bool isEqual) : _robot(robot), _name(name), _isEqual(isEqual)
{
}

const string &TSC::LinearConstraints::name()
{
    return _name;
}

RobotWrapper &TSC::LinearConstraints::robot()
{
    return ref(_robot);
}

bool TSC::LinearConstraints::isEqual() {
    return _isEqual;
}
