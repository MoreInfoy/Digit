//
// Created by nimapng on 6/9/21.
//

#include "TaskSpaceControl/TaskSpaceControl.h"
#include "TaskSpaceControl/Task/SE3MotionTask.h"
#include "TaskSpaceControl/Task/RegularizationTask.h"
#include "TaskSpaceControl/Constraints/ContactPointsConstraints.h"
#include "TaskSpaceControl/Constraints/ContactForceConstraints.h"
#include "TaskSpaceControl/Constraints/ActuatorLimit.h"
#include "TaskSpaceControl/Constraints/ClosedChainsConstraints.h"
#include "TaskSpaceControl/Task/CoMMotionTask.h"
#include "TaskSpaceControl/Constraints/QaccBound.h"
#include "TaskSpaceControl/Task/JointsNominalTask.h"
#include "TaskSpaceControl/Task/AngularMomentumTask.h"
#include "path_PoplarLib.h"

using namespace TSC;

class TSC_IMPL_TEST
{
public:
    TSC_IMPL_TEST(string urdf_file) : _robot(urdf_file)
    {
        mt = make_shared<SE3MotionTask>(_robot, "pelvis");
        mt->Kp() = 100 * Mat6::Identity();
        mt->Kd() = 2 * mt->Kp().cwiseSqrt();
        mt->weightMatrix() = 500 * mt->weightMatrix();
        mt->SE3Ref().setIdentity();
        mt->SE3Ref().translation() << -0.002087, -1.10555e-05, 0.857918;

        mt_waist = make_shared<SE3MotionTask>(_robot, "utorso");
        mt_waist->Kp() = 500 * Mat6::Identity();
        mt_waist->Kd() = 2 * mt_waist->Kp().cwiseSqrt();
        mt_waist->weightMatrix() = 1000 * mt_waist->weightMatrix();
        mt_waist->SE3Ref().setIdentity();
        mt_waist->SE3Ref().translation() << -0.0102634, 0.000143472, 1.06973;

        com = make_shared<CoMMotionTask>(_robot, "com");
        com->weightMatrix() = 5000 * com->weightMatrix();
        com->Kp() = 10 * Mat3::Identity();
        com->Kd() = 2 * com->Kp().cwiseSqrt();
        com->posRef() << 0.0547294, 0.00103252, 1.12591;
        com->velRef().setZero();
        com->accRef().setZero();

        rt = make_shared<RegularizationTask>(_robot, "RegularizationTask");
        rt->qaccWeight().diagonal().fill(1e-2);
        rt->forceWeight().diagonal().fill(1e-8);

        jointsNominalTask = make_shared<JointsNominalTask>(_robot, "JointsNominalTask");
        jointsNominalTask->weightMatrix().setIdentity();
        jointsNominalTask->Kp().setIdentity();
        jointsNominalTask->Kp() = 100 * jointsNominalTask->Kp();
        jointsNominalTask->Kd().setIdentity();
        jointsNominalTask->Kd() = 5 * jointsNominalTask->Kd();
        jointsNominalTask->norminalPosition() << 0, 0.127, 0,
            0.785, -1.15, 2.33, 2.09, 0.452, 1.02, -1.13,
            -0.785, 1.15, 2.33, -2.09, 0.452, -1.02, -1.13,
            0, 0.0785, -0.545, 1.05, -0.54, -0.096,
            0, -0.0785, -0.545, 1.05, -0.54, 0.096;

        angularMomentumTask = make_shared<AngularMomentumTask>(_robot, "AngularMomentumTask");
        angularMomentumTask->weightMatrix().diagonal().fill(10);
        angularMomentumTask->Kp().diagonal().fill(100);
        angularMomentumTask->ref().setZero();
        angularMomentumTask->ref_dot().setZero();

        cpcstr = make_shared<ContactPointsConstraints>(_robot, "cpcstr");
        cfcstr = make_shared<ContactForceConstraints>(_robot, "cfcstr");
        actuatorLimit = make_shared<ActuatorLimit>(_robot, "ActuatorLimit");
        qaccBound = make_shared<QaccBound>(_robot, "QaccBound");
        qaccBound->lb().fill(-100);
        qaccBound->ub().fill(100);

        tsc = make_shared<TaskSpaceControl>(_robot);
        tsc->addTask(mt);
        tsc->addTask(mt_waist);
        tsc->addTask(com);
        tsc->addTask(rt);
        //        tsc->removeTask(rt->name());
        tsc->addTask(jointsNominalTask);
        tsc->addTask(angularMomentumTask);
        tsc->addLinearConstraint(cpcstr);
        tsc->addLinearConstraint(cfcstr);
        tsc->addLinearConstraint(actuatorLimit);
        tsc->addLinearConstraint(qaccBound);
        //    tsc.removeLinearConstraint(ccstr.name());
    }

    ~TSC_IMPL_TEST()
    {
    }

    void setContactMask(const VecXi &mask)
    {
        _robot.setContactMask(mask);
    }

    void setContactVirtualLink(vector<string> &contact_virtual_link)
    {
        _robot.setContactVirtualLink(contact_virtual_link);
    }

    void solve(ConstVecRef qpos, ConstVecRef qvel, const VecXi &mask)
    {
        _robot.computeAllData(qpos, qvel, mask);
        tsc->solve();
        cout << robot().frame_pose("utorso") << endl;
        //        tsc->saveAllData("data.txt");
    }

    RobotWrapper &robot()
    {
        return ref(_robot);
    }

    ConstVecRef getOptimalTorque()
    {
        return tsc->getOptimalTorque();
    }

    ConstVecRef getOptimalContactForce()
    {
        return tsc->getOptimalContactForce();
    }

    ConstVecRef getOptimalQacc()
    {
        return tsc->getOptimalQacc();
    }

private:
    RobotWrapper _robot;
    Vec _lb, _ub;

    shared_ptr<SE3MotionTask> mt;
    shared_ptr<SE3MotionTask> mt_waist;
    shared_ptr<SE3MotionTask> rf;
    shared_ptr<SE3MotionTask> lf;
    shared_ptr<CoMMotionTask> com;
    shared_ptr<RegularizationTask> rt;
    shared_ptr<JointsNominalTask> jointsNominalTask;
    shared_ptr<AngularMomentumTask> angularMomentumTask;
    shared_ptr<ContactPointsConstraints> cpcstr;
    shared_ptr<ContactForceConstraints> cfcstr;
    shared_ptr<QaccBound> qaccBound;
    shared_ptr<ActuatorLimit> actuatorLimit;
    shared_ptr<TaskSpaceControl> tsc;
};

int main()
{
    // build robot model
    string urdf = string(PoplarLib_PATH) + "/test/TaskSpaceControl/atlas.urdf";
    vector<string> contact_virtual_link;
    contact_virtual_link.emplace_back("contact1");
    contact_virtual_link.emplace_back("contact2");
    contact_virtual_link.emplace_back("contact3");
    contact_virtual_link.emplace_back("contact4");
    contact_virtual_link.emplace_back("contact5");
    contact_virtual_link.emplace_back("contact6");
    contact_virtual_link.emplace_back("contact7");
    contact_virtual_link.emplace_back("contact8");
    TSC_IMPL_TEST tsc_impl(urdf);
    tsc_impl.setContactVirtualLink(contact_virtual_link);

    // contact mask
    VecXi mask(8);
    mask.setOnes();
    tsc_impl.setContactMask(mask);

    // robot update
    auto &robot = tsc_impl.robot();
    Vec q, qdot;
    q.resize(robot.nv() + 1);
    //    q << 0.0, 0.0, 0.859652, -0.000007, -0.000244, -0.000001, 1.000000,
    //            0, 0.127, 0,
    //            0.785, -1.15, 2.33, 2.09, 0.452, 1.02, -1.13,
    //            -0.785, 1.15, 2.33, -2.09, 0.452, -1.02, -1.13,
    //            0, 0.0785, -0.545, 1.05, -0.54, -0.096,
    //            0, -0.0785, -0.545, 1.05, -0.54, 0.096;

    q << 0.058076, 0.000003, 0.833132, -0.000039, 0.012975, 0.000006, 0.999916,
        0.000020, 0.191004, -0.000259,
        0.783068, -1.150820, 2.333116, 2.082421, 0.451311, 1.020247, -1.130018,
        -0.783097, 1.150791, 2.333089, -2.082408, 0.451315, -1.020245, -1.130017,
        -0.000404, 0.091643, -0.549629, 1.122151, -0.596475, -0.092708,
        0.000471, -0.091532, -0.549393, 1.121983, -0.596545, 0.092730;
    qdot.resize(robot.nv());
    qdot.setZero();

    // solve
    tsc_impl.solve(q, qdot, mask);
    Vec torque = tsc_impl.getOptimalTorque();
    Vec cforce = tsc_impl.getOptimalContactForce();
    Mat S;
    S.resize(robot.nv(), robot.na());
    S.setZero();
    S.bottomRows(robot.na()).setIdentity();
    Vec qdd = robot.Minv() * (S * torque + robot.contactJacobia().transpose() * cforce - robot.nonLinearEffects());
    cout << torque.transpose() << endl;
    cout << cforce.transpose() << endl;
    cout << qdd.transpose() << endl;
    return 0;
}