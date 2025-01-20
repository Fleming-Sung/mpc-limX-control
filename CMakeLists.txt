cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mpc_limx_control)

set(CMAKE_BUILD_TYPE "Release")

    
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    gazebo_msgs
    std_msgs
    pointfoot_sdk_lowlevel
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS    roscpp    geometry_msgs  gazebo_msgs  std_msgs  pointfoot_sdk_lowlevel
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

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
# install(TARGETS state_estimator_fake DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})
# 安装可执行文件
install(TARGETS state_estimator_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



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

