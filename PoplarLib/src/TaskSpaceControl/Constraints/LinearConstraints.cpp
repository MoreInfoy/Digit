//
// Created by nimapng on 6/9/21.
//

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

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

void TSC::LinearConstraints::errPrint(ConstVecRef u) {
    if(isEqual())
    {
        cout << name() << " equality err: " << (C() * u - c_ub()).norm() << endl;
    }
    else
    {
        cout << name() << " inequality ub err: " << (c_ub() - C() * u).transpose() << endl;
        cout << name() << " inequality lb err: " << (c_lb() - C() * u).transpose() << endl;
    }
}
