
add_executable(RobotWrapperTest RobotWrapper/test.cpp)
target_link_libraries(RobotWrapperTest PoplarLib)

add_executable(SRBD_MPCTest SRBD_MPC/test.cpp)
target_link_libraries(SRBD_MPCTest PoplarLib)

add_executable(TaskSpaceControlTest TaskSpaceControl/test.cpp)
target_link_libraries(TaskSpaceControlTest PoplarLib)

add_executable(TrajectoryTest Trajectory/CubicBSplieTest.cpp)
target_link_libraries(TrajectoryTest PoplarLib)