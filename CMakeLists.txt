cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mpc_limx_control)

set(CMAKE_BUILD_TYPE "Release")
    
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    ocs2_legged_robot
    realtime_tools
    tf2_ros
    pointfoot_sdk_lowlevel
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS    
    roscpp    
    ocs2_legged_robot
    realtime_tools
    tf2_ros
    pointfoot_sdk_lowlevel
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)
    
link_directories(
    ${pinocchio_LIBRARY_DIRS}
    "../pointfoot-sdk-lowlevel/lib/amd64"
)

## Declare a cpp library
# add_library(state_estimator
#   src/stateEstimator.cpp
# )


# executable file -- mpc_control
# add_executable(mpc_control src/mpc_control.cpp)
# target_link_libraries(mpc_control ${catkin_LIBRARIES} Eigen3::Eigen)
# install(TARGETS mpc_control DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})
set(COMMON_SRCS
  src/pf_controller_base.cpp
)

if (WIN32)
  set(LINK_LIBS pointfoot_sdk_lowlevel shlwapi)
else()
  set(LINK_LIBS pointfoot_sdk_lowlevel pthread)
endif()

add_executable(pf_groupJoints_move_test src/pf_groupJoints_move_test.cpp ${COMMON_SRCS}) 
# target_link_libraries(pf_groupJoints_move_test stateEstimator ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(pf_groupJoints_move_test ${catkin_LIBRARIES} ${LINK_LIBS})
install(TARGETS pf_groupJoints_move_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#     include_directories(${catkin_INCLUDE_DIRS})

#     add_custom_target(pointfoot_sdk_lowlevel_include_target ALL
#       COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include/limxsdk ${CATKIN_DEVEL_PREFIX}/include
#     )

#     if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
#       add_custom_target(pointfoot_sdk_lowlevel_target ALL
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_sdk_lowlevel.so ${CATKIN_DEVEL_PREFIX}/lib
#       )
#     elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
#       add_custom_target(pointfoot_sdk_lowlevel_target ALL
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_sdk_lowlevel.so ${CATKIN_DEVEL_PREFIX}/lib
#       )
#     elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
#       add_custom_target(pointfoot_sdk_lowlevel_target ALL
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/libpointfoot_sdk_lowlevel.so ${CATKIN_DEVEL_PREFIX}/lib
#       )
#     elseif (WIN32)
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
#       add_custom_target(pointfoot_sdk_lowlevel_target ALL
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/pointfoot_sdk_lowlevel.dll ${CATKIN_DEVEL_PREFIX}/lib
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_PATH}/pointfoot_sdk_lowlevel.lib ${CATKIN_DEVEL_PREFIX}/lib
#       )
#     else()
#       message(FATAL_ERROR "Unsupported architecture!")
#     endif()

#     link_directories( ${LIB_PATH} )

#     install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    
#     # Install library files to build output
#     install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
  
#   elseif($ENV{ROS_VERSION} EQUAL 2)
#     # In ROS2 compilation environment
#     message("Currently in ROS2 compilation environment")

#     find_package(ament_cmake REQUIRED)

#     if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
#     elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
#     elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
#     elseif (WIN32)
#       set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
#     else()
#       message(FATAL_ERROR "Unsupported architecture!")
#     endif()

#     link_directories( ${LIB_PATH} )

#     install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
    
#     # Install library files to build output
#     install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

#     # Export the library
#     ament_export_libraries(pointfoot_sdk_lowlevel)

#     # Export the include
#     ament_export_include_directories(include)

#     ament_package()
#   endif()
# else()
#   if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
#     set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
#   elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
#     set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
#   elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
#     set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
#   elseif (WIN32)
#     set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
#     install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
#   else()
#     message(FATAL_ERROR "Unsupported architecture!")
#   endif()

#   link_directories(${LIB_PATH})

#   install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})

#   # Install library files to build output
#   install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
# endif()

# option(EXAMPLE "Build examples ON/OFF" ON)
# if(EXAMPLE)
#   add_subdirectory(examples)
# endif()

