# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: 
# CopyPolicy: Released under the terms of the 

if(ENABLE_exampleProgram)

set(KEYWORD "exampleProgram")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_TRAJECTORY_GENERATION_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} main.cpp
                          ExampleProgram.cpp
                          ExampleProgram.hpp)

# Link external libraries.
target_link_libraries(${KEYWORD} YARP::YARP_OS
                                 YARP::YARP_init
                                 ROBOTICSLAB::ColorDebug
                                 ${_exported_targets})

# Create installation rule.
install(TARGETS ${KEYWORD}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT example_program)

endif()
