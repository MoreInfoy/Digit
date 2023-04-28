#ifndef _PROJECT_NAME_CONFIG_H
#define _PROJECT_NAME_CONFIG_H

#define THIS_COM "@THIS_COM@"
//#define FIXED_BASE

#ifdef FIXED_BASE
#define ROBOT_NV 30
#else
#define ROBOT_NV 36
#endif

#define ROBOT_NJ 26 // Joints number, not include passive joints
#define ROBOT_NU 26 // Actuator number

#define URDF "@THIS_COM@/Simulation/model/digit.urdf"
#define SRDF "@THIS_COM@/Simulation/model/digit.srdf"

#endif
