//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_CONFIGURATION_H
#define TASK_SPACE_CONTROL_CONFIGURATION_H

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace std;
namespace pin = pinocchio;

#define RealNum double

#define DAMPING_TERM

#endif //TASK_SPACE_CONTROL_CONFIGURATION_H
