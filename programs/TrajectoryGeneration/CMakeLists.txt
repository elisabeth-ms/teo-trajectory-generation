# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_trajectoryGeneration)

set(KEYWORD "trajectoryGeneration")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set (CMAKE_CXX_STANDARD 11)

find_package(YARP 3.3 REQUIRED COMPONENTS os dev)


find_package(orocos_kdl 1.4)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(TEO_CHECK_COLLISIONS REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl 0.6.0 REQUIRED)
find_library(trac-ik NAMES trac_ik libtrac_ik)


# Retrieve a list of targets defined elsewhere.
# get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)


yarp_prepare_plugin(TrajectoryGeneration
                    CATEGORY device
                    TYPE TrajectoryGeneration
                    INCLUDE TrajectoryGeneration.hpp
		    DEFAULT ON)

yarp_add_plugin(TrajectoryGeneration TrajectoryGeneration.cpp TrajectoryGeneration.hpp)


include_directories( ${track_ik_INCLUDEDIR}   ${pkg_nlopt_INCLUDE_DIRS})

#add_executable(clientTrajectoryGenerationTargetPoseGazebo clientTrajectoryGenerationTargetPoseGazebo.cpp)

# Link external libraries.

target_link_libraries(TrajectoryGeneration PRIVATE
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ${orocos_kdl_LIBRARIES}
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicRepresentationLib
                                 ROBOTICSLAB::TeoCheckCollisionsLibrary
				                 ${OMPL_LIBRARIES} 
                                 fcl
                                 ${nlopt_LIBRARIES}
                                 nlopt
                                 ${PCL_LIBRARIES}
                                trac_ik ${trac_ik_LIBRARIES})

target_include_directories(TrajectoryGeneration PRIVATE 
                                ${orocos_kdl_INCLUDE_DIRS}  
                                ${OMPL_INCLUDE_DIRS}     
                                ${FCL_INCLUDE_DIRS}
                                ${track_ik_INCLUDE_DIRS}
                                )

# list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
#                                                 YARP_sig
#                                                 YARP_dev
#                                                 ROBOTICSLAB::KdlVectorConverterLib
#                                                 ROBOTICSLAB::KinematicRepresentationLib
#                                                 ${OMPL_LIBRARIES} 
#                                                 fcl
#                                                 CheckSelfCollisionLibrary)

yarp_install(
                TARGETS TrajectoryGeneration
                LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
                YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
                )



endif(ENABLE_trajectoryGeneration)
