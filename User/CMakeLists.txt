

FILE(GLOB_RECURSE user_source "*.cpp")

#message(${sim_source})

add_executable(User ${user_source})

include_directories(include)

target_link_libraries(User rt pthread PoplarLib)