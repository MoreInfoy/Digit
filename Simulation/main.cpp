#include "simulate.h"
#include <iostream>
#include <csignal>

static void default_handler(int sig)
{
    (void)sig;
    settings.exitrequest = true;
}
// run event loop
int main(int argc, const char **argv)
{
    signal(SIGINT, default_handler);
    signal(SIGTSTP, default_handler);
    signal(SIGQUIT, default_handler);

    // initialize everything
    init();

    // request loadmodel if file given (otherwise drag-and-drop)
    if (argc > 1)
    {
        mju_strncpy(filename, argv[1], 1000);
        settings.loadrequest = 2;
    }
    else
    {
        mju_strncpy(filename, (std::string(THIS_COM) + std::string("/Simulation/model/digit_ysp.xml")).c_str(), 1000);
        std::cout << "MODEL: " << filename << std::endl;
        settings.loadrequest = 1;
    }

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while (!glfwWindowShouldClose(window) && !settings.exitrequest)
    {
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

        glBegin(GL_LINE);
        glVertex2f(.25, 0.25);
        glVertex2f(.75, .75);
        glEnd();

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
