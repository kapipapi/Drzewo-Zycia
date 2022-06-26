include(CMakeParseArguments)

function(parse_function_args)
    cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
    cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
    if (OUT_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
    endif ()
    foreach (arg ${IN_REQUIRED})
        if (NOT OUT_${arg})
            if (NOT "${OUT_${arg}}" STREQUAL "0")
                message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
            endif ()
        endif ()
    endforeach ()
    foreach (arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
        set(${arg} ${OUT_${arg}} PARENT_SCOPE)
    endforeach ()
endfunction()

function(prepend)
    parse_function_args(
            NAME prepend
            ONE_VALUE PREFIX LIST
            REQUIRED PREFIX LIST
            ARGN ${ARGN})

    if (${CMAKE_VERSION} VERSION_LESS "3.12.0")
        FOREACH (element ${${LIST}})
            LIST(APPEND PREPENDED "${PREFIX}/${element}")
        ENDFOREACH ()
        SET(${LIST} ${PREPENDED})
    else ()
        list(TRANSFORM ${LIST} PREPEND ${PREFIX})
    endif ()

    SET(${LIST} ${${LIST}} PARENT_SCOPE)

endfunction()


function(library)
    parse_function_args(
            NAME library
            ONE_VALUE ID
            MULTI_VALUE SOURCES DEPENDENCIES TEST_DEPENDENCIES
            REQUIRED NAME SOURCES
            ARGN ${ARGN})

    prepend(LIST SOURCES PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/")

    add_library(${ID} ${SOURCES})

    target_include_directories(${ID} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            PRIVATE src)

    if (DEPENDENCIES)
        target_link_libraries(${ID} ${DEPENDENCIES})
    endif ()

    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
        add_executable(test_${ID} ${TEST_SOURCES})
        file(GLOB TEST_RESOURCES EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/tests/*)
        list(FILTER TEST_RESOURCES EXCLUDE REGEX "^.*\\.[cpp|h]")
        foreach (file_path ${TEST_RESOURCES})
            get_filename_component(file_name "${file_path}" NAME)
            configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/${file_name} ${CMAKE_CURRENT_BINARY_DIR}/${file_name} COPYONLY)
        endforeach ()
        target_link_libraries(test_${ID} ${ID} ${DEPENDENCIES} ${TEST_DEPENDENCIES})
        add_test(${ID} test_${ID})
    endif ()

endfunction()

function(executable)
    parse_function_args(
            NAME executable
            ONE_VALUE ID
            MULTI_VALUE SOURCES DEPENDENCIES TEST_DEPENDENCIES
            REQUIRED NAME SOURCES
            ARGN ${ARGN})

    prepend(LIST SOURCES PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/")

    add_executable(${ID} ${SOURCES})
    install(TARGETS ${ID}
            RUNTIME DESTINATION bin)
    #    set_target_properties(${ID} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
    target_include_directories(${ID} PRIVATE src)

    if (DEPENDENCIES)
        target_link_libraries(${ID} ${DEPENDENCIES})
    endif ()

    file(GLOB RESOURCES EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/res/*)

    if(RESOURCES)
        add_custom_target(${ID}-resources)
        foreach(resource ${RESOURCES})
            add_custom_command(TARGET ${ID}-resources PRE_BUILD
                    COMMAND ${CMAKE_COMMAND} -E
                    copy_if_different ${resource} ${CMAKE_CURRENT_BINARY_DIR}/)
            get_filename_component(filename ${resource} NAME)
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${filename} DESTINATION bin)
        endforeach()
        add_dependencies(${ID} ${ID}-resources)
    endif()

endfunction()