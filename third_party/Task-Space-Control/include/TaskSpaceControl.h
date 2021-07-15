//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_TASKSPACECONTROL_H
#define TASK_SPACE_CONTROL_TASKSPACECONTROL_H


#include "ConfigurationTSC.h"
#include "TypeDefinition.h"
#include "Task/Task.h"
#include "Constraints/LinearConstraints.h"
#include "RobotWrapper/RobotWrapper.h"
#include "qpOASES.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

//#define USE_QPOASES

//#define PRINT_ERR

namespace TSC {
    class TaskSpaceControl {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit TaskSpaceControl(RobotWrapper &robot);

        void addTask(Task *task);

        void removeTask(string name);

        void addLinearConstraint(LinearConstraints *constraints);

        void removeLinearConstraint(string name);

        size_t getInputDims();

        void solve();

        ConstVecRef getOptimalQacc(); // [qacc;f]

        ConstVecRef getOptimalTorque(); // tau

        ConstVecRef getOptimalContactForce(); // f

        void saveAllData(string file_name);

        void printCstrsErr();

    private:
        RobotWrapper &_robot;
        vector<Task *> _tasks;
        vector<LinearConstraints *> _linearConstraints;
#ifdef USE_QPOASES
        Eigen::Matrix<RealNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H, C, Ce;
#else
        Mat H, C, Ce;
#endif
        Vec g, c_lb, c_ub, ce, optimal_u, optimal_tau;
        int _u_dims;

#ifndef USE_QPOASES
        eiquadprog::solvers::EiquadprogFast eiquadprog_solver;
#endif
    };
}


#endif //TASK_SPACE_CONTROL_TASKSPACECONTROL_H
