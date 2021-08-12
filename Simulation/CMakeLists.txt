

FILE(GLOB_RECURSE sim_source "*.cpp")

#message(${sim_source})

add_executable(MuJoCoSim ${sim_source})

include_directories(include)

link_directories(${THIS_COM}/Simulation/lib/)

target_link_libraries(MuJoCoSim  mujoco200 GL glew glfw rt pthread lcm)