/*! @file SimulatorMessage.h
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory.   This is basically everything except
 * for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "SharedMemory.h"
#include "Configuration.h"
#include <vector>


/*!
 * All the data shared between the robot and the simulator
 */
template<typename UserToRobotMessage, typename RobotToUserMessage>
class SharedMessage {
public:
    SharedMessage() : userToRobot(), robotToUser() {
    }

    UserToRobotMessage userToRobot;
    RobotToUserMessage robotToUser;
};

/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by
 * both the simulator and the robot The simulator and robot take turns have
 * exclusive access to the entire message. The intended sequence is:
 *  - robot: waitForSimulator()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do
 * anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do
 * anything)
 *  - robot: robotDone();
 *  - robot: waitForSimulator()
 *  ...
 */
template<typename UserToRobotMessage, typename RobotToUserMessage>
class SyncronizedSharedMessage : public SharedMessage<UserToRobotMessage, RobotToUserMessage> {
public:
    SyncronizedSharedMessage() : SharedMessage<UserToRobotMessage, RobotToUserMessage>() {
    }

    /*!
     * The init() method should only be called *after* shared memory is connected!
     */
    void init() {
        robotToUserSemaphore.init(0);
        userToRobotSemaphore.init(0);
    }

    void robotToUserInit() {
        robotToUserSemaphore.init(0);
    }

    void userToRobotInit() {
        userToRobotSemaphore.init(0);
    }

    void reset() {
        robotToUserSemaphore.reset();
        userToRobotSemaphore.reset();
    }

    void waitForRobot() { robotToUserSemaphore.decrement(); }

    bool waitForRobotWithTimeout(unsigned int seconds, unsigned int nanoseconds) {
        return robotToUserSemaphore.decrementTimeout(seconds, nanoseconds);
    }

    void robotDone() { robotToUserSemaphore.increment(); }

    void waitForUser() { userToRobotSemaphore.decrement(); }

    bool tryWaitForUser() { return userToRobotSemaphore.tryDecrement(); }

    bool waitForUserWithTimeout(unsigned int seconds, unsigned int nanoseconds) {
        return userToRobotSemaphore.decrementTimeout(seconds, nanoseconds);
    }

    void userDone() { userToRobotSemaphore.increment(); }

private:
    SharedMemorySemaphore robotToUserSemaphore, userToRobotSemaphore;
};

#endif // PROJECT_SIMULATORTOROBOTMESSAGE_H
