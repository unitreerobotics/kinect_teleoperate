# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/src/CMakeLists.txt")
    find_package(Git REQUIRED QUIET)
    if (Git_FOUND)
        message(STATUS "Fetching git modules...")
        execute_process(
            COMMAND
                ${GIT_EXECUTABLE} submodule update --init --recursive
            WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE
                GIT_SUBMODULE_UPDATE_RESULT
            OUTPUT_VARIABLE
                GIT_SUBMODULE_UPDATE_OUTPUT
        )
        
        if (NOT (${GIT_SUBMODULE_UPDATE_RESULT} EQUAL 0))
            message(FATAL_ERROR "git submodule update failed: ${GIT_SUBMODULE_UDPATE_OUTPUT}")
        endif()
    endif()
endif()

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

add_subdirectory(extern)
add_subdirectory(kinect_teleoperate_robot)
add_subdirectory(sample_helper_libs)
