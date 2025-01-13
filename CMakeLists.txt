cmake_minimum_required(VERSION 3.0.2)
project(curl_slam)
## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-16")
#set(CMAKE_C_COMPILER "/usr/bin/clang-16")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# Eigen
set(CMAKE_PREFIX_PATH "/home/user/Application/eigen-3.4.0/installation" ${CMAKE_PREFIX_PATH})
set(Eigen3_DIR "/home/user/Application/eigen-3.4.0/installation/share/eigen3/cmake")
find_package(Eigen3 REQUIRED 3.4.0)
include_directories(${EIGEN3_INCLUDE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        #  pcl_ros
        roscpp
        rospy
        std_msgs
        visualization_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
        cfg/map_config_release.cfg
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# pcl
set(PCL_DIR "/home/user/Application/pcl/installation/share/pcl-1.13")
find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Boost
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
# yaml-cpp
find_package(yaml-cpp REQUIRED)
# Ceres
set(Ceres_DIR "/home/user/Application/ceres-solver/installation/lib/cmake/Ceres")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(libigl)
igl_include(glfw)
# CGAL

set(CGAL_DIR "/home/user/Application/cgal/installation/lib/cmake/CGAL")
find_package(CGAL REQUIRED COMPONENTS Core)
include_directories("/home/user/Application/cgal/installation/include")
add_definitions(-DCGAL_KERNEL_NO_ASSERTIONS)
# Sophus
find_package(Sophus REQUIRED)
# opencv
set(OpenCV_DIR /home/user/Application/opencv-4.5.5/installation/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
# nanoflann
find_package(nanoflann)
# open3d
find_package(Open3D REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg

# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES curl_slam
        #  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs visualization_msgs
        #  DEPENDS system_lib
        #        CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include/curl_slam
        ${catkin_INCLUDE_DIRS}
        # /home/user/Application/boost_1_82_0/
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/curl_slam.cpp
# )
add_library(ROS_Handler SHARED lib/RosHandler.cpp)
target_link_libraries(ROS_Handler ${catkin_LIBRARIES} Eigen3::Eigen)

#add_library(direct_method SHARED lib/DirectMethod.cpp)
#target_link_libraries(direct_method Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(Patch_Procession SHARED lib/PatchProcession.cpp)
target_link_libraries(Patch_Procession Eigen3::Eigen ${PCL_LIBRARIES} CGAL::CGAL CGAL::CGAL_Core)

# add_library(CURL_local_BA SHARED lib/CURL_local_BA.cpp)
# target_link_libraries(CURL_local_BA ${PCL_LIBRARIES} ${CERES_LIBRARIES} Eigen3::Eigen gflags yaml-cpp Sophus::Sophus CGAL::CGAL CGAL::CGAL_Core ROS_Handler direct_method Patch_Procession ${OpenCV_LIBS})

add_library(range_v3 INTERFACE IMPORTED)
set_target_properties(range_v3 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES /home/user/Application/range-v3/include)

add_library(CURL_Geometry SHARED lib/CurlGeometry.cpp)
target_link_libraries(CURL_Geometry PUBLIC Eigen3::Eigen range_v3)

add_library(ikd_tree SHARED include/curl_slam/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree ${PCL_LIBRARIES})

add_library(AABB_tree SHARED include/curl_slam/AABB_tree/AABB.cc)

# three main library for odometry
add_library(CurlConformalMapping SHARED lib/ConformalMappingSPH.cpp)
target_link_libraries(CurlConformalMapping CURL_Geometry Patch_Procession AABB_tree ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp CGAL::CGAL CGAL::CGAL_Core nanoflann::nanoflann)

add_library(CurlMapUpdate SHARED lib/CurlMapUpdate.cpp)
target_link_libraries(CurlMapUpdate CurlConformalMapping Patch_Procession AABB_tree ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp CGAL::CGAL CGAL::CGAL_Core nanoflann::nanoflann)

add_library(CurlTracking SHARED lib/CurlTracking.cpp)
target_link_libraries(CurlTracking CurlConformalMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} Eigen3::Eigen gflags yaml-cpp Sophus::Sophus CGAL::CGAL CGAL::CGAL_Core ROS_Handler ${OpenCV_LIBS})

add_library(CurlLocalBA SHARED lib/CurlLocalBA.cpp)
target_link_libraries(CurlLocalBA CurlConformalMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} Eigen3::Eigen gflags yaml-cpp Sophus::Sophus CGAL::CGAL CGAL::CGAL_Core ROS_Handler ${OpenCV_LIBS})

add_library(CurlMapGeneration SHARED lib/CurlMapGeneration.cpp)
add_dependencies(CurlMapGeneration ${PROJECT_NAME}_gencfg)
target_link_libraries(CurlMapGeneration ${catkin_LIBRARIES} CurlConformalMapping Open3D::Open3D)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/curl_slam_node.cpp)
add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})
add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})
add_executable(save_segmented_pc src/save_segmented_pc.cpp)
target_link_libraries(save_segmented_pc ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp)
# add_executable(main_local_BA src/main_local_BA.cpp)
# target_link_libraries(main_local_BA ${PCL_LIBRARIES} Eigen3::Eigen CURL_local_BA Boost::program_options yaml-cpp -lstdc++fs ${catkin_LIBRARIES})

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node CurlConformalMapping CurlTracking CurlLocalBA ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp)

add_executable(map_generation_node src/map_generation_node.cpp)
target_link_libraries(map_generation_node CurlMapGeneration ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp)

add_executable(down_sampling src/down_sampling.cpp)
target_link_libraries(down_sampling ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp)

#target_link_libraries(save_segmented_pc Eigen3::Eigen yaml-cpp ${catkin_LIBRARIES} Boost::program_options)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_curl_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
