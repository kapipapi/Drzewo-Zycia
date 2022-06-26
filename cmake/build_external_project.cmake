include(ProcessorCount)

# This function is used to force a build on a dependant project at cmake configuration phase.
function(build_external_project TARGET URL VERSION) #FOLLOWING ARGUMENTS are the CMAKE_ARGS of ExternalProject_Add
    set(TARGET_DIR ${CMAKE_BINARY_DIR}/external/${TARGET})
    set(BUILD_DIR ${TARGET_DIR}/build)
    set(INSTALL_DIR ${TARGET_DIR}/install)

    file(MAKE_DIRECTORY ${TARGET_DIR} ${BUILD_DIR})

    set(${TARGET}_URL ${URL})
    set(${TARGET}_VERSION ${VERSION})
    set(${TARGET}_CMAKE_ARGS ${ARGN})

    configure_file("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/external_project.cmake.in" ${TARGET_DIR}/CMakeLists.txt)

    execute_process(
            COMMAND ${CMAKE_COMMAND} ..
            WORKING_DIRECTORY ${BUILD_DIR}
    )

    ProcessorCount(N)
    execute_process(
            COMMAND ${CMAKE_COMMAND} --build . --parallel ${N}
            WORKING_DIRECTORY ${BUILD_DIR}
    )

    set(${TARGET}_DIR ${INSTALL_DIR}/lib/cmake/${TARGET} PARENT_SCOPE)
endfunction()