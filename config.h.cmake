#ifndef _${PROJECT_NAME}_CONFIG_H
#define _${PROJECT_NAME}_CONFIG_H

#define THIS_COM "@THIS_COM@"
//#define FIXED_BASE

#ifdef FIXED_BASE
#define ROBOT_NV 30
#else
#define ROBOT_NV 36
#endif

#define ROBOT_NJ 26 // Joints number, not include passive joints
#define ROBOT_NU 26 // Actuator number
#define RealNum double
#define MJKEY "@THIS_COM@/Simulation/lib/mjkey.txt"

#define URDF "@THIS_COM@/Simulation/model/digit_ysp_o.urdf"
#define SRDF "@THIS_COM@/Simulation/model/digit.srdf"

#endif
