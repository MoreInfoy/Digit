#include "simulate.h"
#include <iostream>
#include <csignal>
#include <lcm/lcm-cpp.hpp>
#include "Trajectory_LCM.hpp"

static void default_handler(int sig) {
    (void) sig;
    settings.exitrequest = true;
}

class Handler {
public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const Trajectory_LCM *msg) {
        traj_mutex.lock();
        trajectory.resize(msg->n_point);
        for (int i = 0; i < msg->n_point; i++) {
            trajectory[i] << msg->data[i * 3], msg->data[i * 3 + 1], msg->data[i * 3 + 2];
            cout << "trajectory[" << i << "]" << trajectory[i].transpose() << endl;
        }
        traj_mutex.unlock();
    }
};

lcm::LCM sub;

void subscribe() {
    while (sub.handle() == 0 && !settings.exitrequest);
}

// run event loop
int main(int argc, const char **argv) {
    signal(SIGINT, default_handler);
    signal(SIGTSTP, default_handler);
    signal(SIGQUIT, default_handler);

    // initialize everything
    init();

    // request loadmodel if file given (otherwise drag-and-drop)
    if (argc > 1) {
        mju_strncpy(filename, argv[1], 1000);
        settings.loadrequest = 2;
    } else {
        mju_strncpy(filename, (std::string(THIS_COM) + std::string("/Simulation/model/digit_ysp_o.xml")).c_str(), 2000);
        std::cout << "MODEL: " << filename << std::endl;
        settings.loadrequest = 1;
    }

    // start simulation thread
    std::thread simthread(simulate);

    if (!sub.good())
        return 1;
    std::thread sub_thread(subscribe);
    Handler handlerObject;
    sub.subscribe("TRAJECTORY_LCM", &Handler::handleMessage, &handlerObject);

    // event loop
    while (!glfwWindowShouldClose(window) && !settings.exitrequest) {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
            loadmodel();
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    mj_deactivate();

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
