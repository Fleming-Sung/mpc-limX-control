cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mpc_limx_control)

set(CMAKE_BUILD_TYPE "Release")

    
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    # ocs2_legged_robot
    # realtime_tools
    # tf2_ros
    geometry_msgs
    gazebo_msgs
    tf
    std_msgs
    pointfoot_sdk_lowlevel
)

find_package(Eigen3 REQUIRED)

# find_package(pinocchio REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS    
    roscpp    
    geometry_msgs  
    gazebo_msgs  
    tf  
    std_msgs  
    # ocs2_legged_robot
    # realtime_tools
    # tf2_ros
    pointfoot_sdk_lowlevel
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${PINOCCHIO_INCLUDE_DIRS}
    )

# link_directories(
#     ${pinocchio_LIBRARY_DIRS}
#     "../pointfoot-sdk-lowlevel/lib/amd64"
#     )

# set(COMMON_SRCS
#   src/pf_controller_base.cpp
# )

# if (WIN32)
#   set(LINK_LIBS pointfoot_sdk_lowlevel shlwapi)
# else()
#   set(LINK_LIBS pointfoot_sdk_lowlevel pthread)
# endif()


# # state_estimator_fake.cpp的可执行测试
# add_executable(state_estimator_fake src/state_estimator_fake.cpp)
# target_link_libraries(state_estimator_fake ${catkin_LIBRARIES})
# # install(TARGETS state_estimator_fake DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})
# # 安装可执行文件
# install(TARGETS state_estimator_fake
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# state_estimator_fake.h的可执行测试（面向对象封装）
add_executable(state_estimator_test src/state_estimator_test.cpp)
target_link_libraries(state_estimator_test ${catkin_LIBRARIES})
# 安装可执行文件
install(TARGETS state_estimator_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# mpc control主程序的开发过程测试
add_executable(mpc_limx_control_fake_state src/mpc_control_fake_state.cpp)
target_link_libraries(mpc_limx_control_fake_state ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})
# 安装可执行文件
install(TARGETS mpc_limx_control_fake_state
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # 真实状态估计器的测试
# add_executable(pf_groupJoints_move_test src/pf_groupJoints_move_test.cpp ${COMMON_SRCS}) 
# # target_link_libraries(pf_groupJoints_move_test stateEstimator ${catkin_LIBRARIES} Eigen3::Eigen)
# target_link_libraries(pf_groupJoints_move_test ${catkin_LIBRARIES} ${LINK_LIBS})
# install(TARGETS pf_groupJoints_move_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
