//
// Created by nimpng on 6/13/21.
//

#include "SharedMessages.h"
#include "EigenTypes.h"
#include <csignal>

bool exitrequest = false;

static void default_handler(int sig) {
    (void) sig;
    exitrequest = true;
}

int main() {
    signal(SIGINT, default_handler);
    signal(SIGTSTP, default_handler);
    signal(SIGQUIT, default_handler);

    SharedMemoryObject<SyncronizedSharedMessage<UserCmd, RobotState>> shared_memory;
    if (shared_memory.is_created()) {
        shared_memory.attach(SHAREDMEMORY_NAME);
    } else {
        shared_memory.createNew(SHAREDMEMORY_NAME, true);
    }
    shared_memory().init();

    Poplar::Vec qdes(ROBOT_NU);
    qdes << 0.325, 0, 0, 0, 0, -0.102, -0.07,
            0, 0.987, 0, 0,
            -0.325, 0, 0, 0, 0, 0.102, 0.07,
            0, -0.987, 0, 0;

    while (!exitrequest) {
        if (shared_memory().waitForRobotWithTimeout(1, 0)) {
            Poplar::Vec qj, qjdot;
            qj = shared_memory().robotToUser.q.tail(ROBOT_NU);
            qjdot = shared_memory().robotToUser.qdot.tail(ROBOT_NU);
            shared_memory().userToRobot.tau = 1000 * (qdes - qj) - 10.5 * qjdot;
            shared_memory().userDone();
        }

    }
    if (shared_memory.is_created())
        shared_memory.detach();

    return 0;
}
