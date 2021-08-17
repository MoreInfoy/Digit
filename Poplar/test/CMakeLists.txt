
add_executable(RobotWrapperTest RobotWrapper/test.cpp)
target_link_libraries(RobotWrapperTest Poplar)

add_executable(SRBD_MPCTest SRBD_MPC/test.cpp)
target_link_libraries(SRBD_MPCTest Poplar)

add_executable(TaskSpaceControlTest TaskSpaceControl/test.cpp)
target_link_libraries(TaskSpaceControlTest Poplar)

add_executable(TrajectoryTest Trajectory/CubicBSplieTest.cpp)
target_link_libraries(TrajectoryTest Poplar)

add_executable(LIPM_MPC_Test LIPM/test.cpp)
target_link_libraries(LIPM_MPC_Test Poplar)