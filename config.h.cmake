#ifndef _${PROJECT_NAME}_CONFIG_H
#define _${PROJECT_NAME}_CONFIG_H

#define THIS_COM "@THIS_COM@"
//#define FIXED_BASE

#ifdef FIXED_BASE
#define ROBOT_NV 30
#else
#define ROBOT_NV 36
#endif

#define ROBOT_NJ 30 // Joints number, include passive joints
#define ROBOT_NU 30 // Actuator number
#define RealNum double
#define MJKEY "@THIS_COM@/Simulation/lib/mjkey.txt"

#define URDF "@THIS_COM@/Simulation/model/digit_ysp_o.urdf"

#endif
