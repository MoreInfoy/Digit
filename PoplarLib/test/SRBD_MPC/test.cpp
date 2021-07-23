//
// Created by nimapng on 7/19/21.
//

#include "SRGB_MPC/SRGB_MPC.h"

using namespace SRGB_MPC;

int main() {
    SRGB_MPC::SRGB_MPC_IMPL srgbMpc(10, 0.03);
    Vec Qx(12);
    Qx << 2.5, 2.5, 10, 10, 10, 50, 0, 0, 0.3, 0.2, 0.2, 0.1;
    Vec Qf(12);
    Qf.fill(1e-5);
    srgbMpc.setWeight(Qx, Qf);

    Mat3 Ibody;
    Ibody << 0.353682, -0.000624863, -0.0313391,
            -0.000624863, 0.813405, -0.000940105,
            -0.0313391, -0.000940105, 0.863894;
    RealNum mass = 15.8198;
    srgbMpc.setMassAndInertia(mass, Ibody);

    Vec x0(12);
    x0 << 0, 0, 0, 0, 0, 0.41, 0, 0, 0, 0, 0, 0;
    srgbMpc.setCurrentState(x0);

    MatInt contactTable(4, 10);
    contactTable.setOnes();
    srgbMpc.setContactTable(contactTable);

    vector<Vec3> cp;
    cp.push_back(Vec3(0.322132, 0.172679, 0.0357475));
    cp.push_back(Vec3(0.33049, -0.172644, 0.0357495));
    cp.push_back(Vec3(-0.255729, 0.172508, 0.0356581));
    cp.push_back(Vec3(-0.248833, -0.172669, 0.0356571));
    srgbMpc.setContactPointPos(cp);

    srgbMpc.solve(0.0);
    std::cout << "cf: " << srgbMpc.getOptimalContactForce().head(12).transpose() << std::endl;
    std::cout << "x: " << srgbMpc.getDiscreteOptimizedTrajectory().transpose() << std::endl;
}