#ifndef _${PROJECT_NAME}_CONFIG_H
#define _${PROJECT_NAME}_CONFIG_H

#define THIS_COM "@THIS_COM@"
// #define FIXED_BASE

#ifdef FIXED_BASE
#define ROBOT_NV 22
#else
#define ROBOT_NV 28
#endif

#define ROBOT_NU 22
#define RealNum double
#define MJKEY "@THIS_COM@/Simulation/lib/mjkey.txt"

#define URDF "@THIS_COM@/Simulation/model/digit_model.urdf"



#endif
