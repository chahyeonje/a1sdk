cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

set(CMAKE_AUTOMOC ON) #if qt does not work.

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )
ADD_COMPILE_DEFINITIONS(URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rsc")

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS
        pthread
        unitree_legged_sdk_amd64
        lcm
        Qt5::Widgets)

set(CMAKE_CXX_FLAGS "-O3")

set(GUI_SOURCES
        examples/qt/mainwindow.cpp
        examples/qt/qcustomplot.cpp
        )

add_executable(example_position examples/example_position.cpp examples/colRBDL.cpp ${GUI_SOURCES})
target_link_libraries(example_position ${EXTRA_LIBS} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

#rbdl test
add_executable(test_rbdl examples/test_rbdl.cpp examples/colRBDL.cpp)
target_link_libraries(test_rbdl ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

# collision detection
add_executable(example_collision_detection examples/example_collisiondetection.cpp examples/colRBDL.cpp)
target_link_libraries(example_collision_detection ${EXTRA_LIBS} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

add_executable(example_getLowState examples/example_getLowState.cpp examples/colRBDL.cpp)
target_link_libraries(example_getLowState ${EXTRA_LIBS} ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})

# one pc one process

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_dance examples/example_dance.cpp)
target_link_libraries(example_dance ${EXTRA_LIBS})

add_executable(example_wirelessHandle examples/example_wirelessHandle.cpp)
target_link_libraries(example_wirelessHandle ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

#add_subdirectory(pybind11)
#pybind11_add_module(robot_interface_high_level src/python_interface_high_cmd.cpp)
#target_link_libraries(robot_interface_high_level ${EXTRA_LIBS})