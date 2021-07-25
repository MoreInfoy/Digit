//
// Created by nimapng on 6/7/21.
//

#include "SRGB_MPC/LinearMPC.h"
#include "Timer.h"

int main() {
    MPC::Timer timer;
    LinearMPC linearMpc;
    double st = timer.getMs();
    MPC::Mat_Ak Ak;
    MPC::Mat_Bk Bk;
    Ak << 1.0, 0.0494, 0, 0.9753;
    Bk << 0.0012, 0.0494;
    linearMpc.setAkBk(Ak, Bk);

//    MPC::Mat_Ccx Ccx;
//    Ccx << 1.0, 0;
//    MPC::Vec_bcx cxlb, cxub;
//    cxlb << -3.0;
//    cxub << 3.0;
//    linearMpc.setStateConstraints(Ccx, cxlb, cxub);

//    MPC::Mat_Ccu Ccu;
//    Ccu << 0.5;
//    MPC::Vec_bcu culb, cuub;
//    culb << -3.0;
//    cuub << 3.0;
//    linearMpc.setInputConstraints(Ccu, culb, cuub);

//    MPC::Vec_U_Bounds lb, ub;
//    lb << -5.5;
//    ub << 4.6;
//    linearMpc.setInputBounds(lb, ub);

    MPC::Mat_Qx Q;
    Q << 1000.0, 0, 0, 10.0;
    MPC::Mat_Ru R;
    R << 0.1;
    linearMpc.setWeightMatrix(Q, R);

    MPC::Vec x0, x_ref;
    x0.resize(2);
    x0 << 2.0, 0.0;
    x_ref.resize(HORIZON * Ak_ROWS);
    x_ref.setZero();
    for (int i = 0; i < 2 * HORIZON; i++) {
        if(i%2==0)
        {
            x_ref(i) = -2.0;
        }
        else
        {
            x_ref(i) = 0.0;
        }
    }

    linearMpc.setInitialStateAndRef(x0, x_ref);

    double time_formulation = timer.getMs() - st;

    for (int i = 0; i < 10; i++) {
        st = timer.getMs();

//        std::cout << "sol: " << linearMpc.getSolution().transpose() << std::endl;

        linearMpc.getSolution();
        double et = timer.getMs();

        printf("solve time: %f, formulation time: %f\n", et - st, time_formulation);
    }

    linearMpc.outputAllDataToFile("data.txt");

    return 0;

}
