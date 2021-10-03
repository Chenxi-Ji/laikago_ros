cmake_minimum_required(VERSION 3.5)
project(laikago_gazebo)
set(CMAKE_CXX_STANDARD 14)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    laikago_msgs  
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    laikago_msgs 
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    #${GAZEBO_LIBRARY_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


add_subdirectory(qpOASES)
add_subdirectory(JCQP)

# Declare a C++ library
add_library(${PROJECT_NAME}
    include/body.h
    include/StateEstimatorContainer.h
    src/DesiredCommand.cpp
    BalanceController/BalanceController.cpp
    BalanceController/BalanceControllerWrapper.cpp
    BalanceController/QPLocomotion.cpp
    ConvexMPC/RobotState.cpp
    ConvexMPC/SolverMPC.cpp
    ConvexMPC/convexMPC_interface.cpp
    ConvexMPC/ConvexMPCLocomotion.cpp
    #ConvexMPC/ConvexMPCBounding.cpp
    LeastSquareSolution/LeastSquareSolution.cpp
    src/body.cpp 
    src/CurrentState.cpp
    src/LegController.cpp
    src/teleCmd.cpp
    src/Gait.cpp
    src/FSM.cpp
    include/TransitionData.h
    include/ControlFSMData.h
    src/FootSwingTrajectory.cpp
    src/PositionVelocityEstimator.cpp
    src/OrientationEstimator.cpp
)
file(GLOB_RECURSE sources "src/*.cpp")

add_dependencies(${PROJECT_NAME} laikago_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

add_library(LaikagoFootContactPlugin SHARED plugin/foot_contact_plugin.cc)

target_link_libraries(LaikagoFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-O3  -ggdb -no-pie")

add_library(LaikagoDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(LaikagoDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} qpOASES JCQP)

add_executable(laikago_external_force src/exe/external_force.cpp)
target_link_libraries(laikago_external_force ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(drcl_ctrl src/exe/DRCL.cpp)
target_link_libraries(drcl_ctrl ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(laikago_servo src/exe/servo.cpp)
target_link_libraries(laikago_servo ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(leg_test src/exe/leg_test.cpp)
target_link_libraries(leg_test ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(ori_test src/exe/ori_test.cpp)
target_link_libraries(ori_test ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(a1_jump src/exe/jump.cpp)
target_link_libraries(a1_jump ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(reset src/exe/reset.cpp)
target_link_libraries(reset ${PROJECT_NAME} ${catkin_LIBRARIES} qpOASES JCQP)

add_executable(move_block src/exe/addBlock.cpp)
target_link_libraries(move_block ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} qpOASES JCQP)
