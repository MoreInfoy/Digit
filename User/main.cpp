//
// Created by nimpng on 6/13/21.
//

#include "StateAndCommand.h"
#include "SharedMessages.h"
#include "Manager.h"
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

    Manager manager(shared_memory().robotToUser);
    manager.init();

    Poplar::Vec qdes(ROBOT_NJ);
    qdes
            << 0.325685, 0.00398248, 0.0609534, 0.109731, -0.109664, 0.0277016, -0.0611248, -0.0469275, -0.0573293,
            -0.308679, 0.937404, -0.0370688, 0.0204076,
            -0.325474, -0.000611338, -0.0622421, -0.109314, 0.109222, -0.0266944, 0.0600983, 0.0458314, 0.0572924,
            0.309157, -1.01209, 0.00156306, 0.0198919;
    /*qdes << 0.325, 0, 0, 0, 0, -0.102, -0.07,
            -0.325, 0, 0, 0, 0, 0.102, 0.07,
            -0.288, 0.987, 0, 0,
            0.288, -0.987, 0, 0;*/


    while (!exitrequest) {
        if (shared_memory().waitForRobotWithTimeout(1, 0)) {
            manager.run();
            auto &qpos = shared_memory().robotToUser.jointsState.qpos;
            auto &qvel = shared_memory().robotToUser.jointsState.qvel;
            shared_memory().userToRobot.tau = manager.output();
//            shared_memory().userToRobot.tau = 1000 * (qdes - qpos) - 5 * qvel;
            shared_memory().userDone();
        }
    }
    if (shared_memory.is_created())
        shared_memory.detach();

    return 0;
}
