# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: 
# CopyPolicy: Released under the terms of the 

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project. Set the LANGUAGES option according to your
# needs, e.g. 'C', 'CXX', 'NONE'.
project(TEO_TRAJECTORY_GENERATION VERSION 0.1.0 LANGUAGES CXX)

# Set module path.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/modules
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.2 REQUIRED COMPONENTS OS)
find_package(PCL REQUIRED)

# Soft dependencies.
find_package(Doxygen QUIET)
find_package(GTestSources 1.6.0 QUIET)

# Add WITH_PLUGINS if this projects exports YARP devices.
yarp_configure_external_installation(teo-trajectory-generation)

# Retrieve current TEO_TRAJECTORY_GENERATION version.
include(TEO_TRAJECTORY_GENERATIONVersion)

# Define standard paths, check system configuration.
include(TEO_TRAJECTORY_GENERATIONSystem)

# Setup build options.
include(TEO_TRAJECTORY_GENERATIONOptions)

# Define and enter subdirectories.
add_subdirectory(cmake)
add_subdirectory(programs)
add_subdirectory(share)


# Output configuration files.
include(TEO_TRAJECTORY_GENERATIONDescribe)

# Invoke CPack from here.
include(TEO_TRAJECTORY_GENERATIONPackage)

# Generate a report of enabled/disabled features.
feature_summary(WHAT ALL)
