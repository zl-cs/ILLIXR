# generate the profile yaml files from the master plugins.yaml
function(generate_yaml)
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/plugins/plugins.yaml")
        message(FATAL_ERROR "plugins/plugins.yaml file is missing")
    endif()

    file(TIMESTAMP "${PROJECT_SOURCE_DIR}/plugins/plugins.yaml" PROFILE_FILE_TIMESTAMP "%s" UTC)

    if(DEFINED CACHE{LAST_YAML_BUILD} AND DEFINED CACHE{PLUGIN_LIST} AND DEFINED CACHE{VISUALIZER_LIST_LIST})
        #message("Profile files are up to date")
    else()
        set(LAST_YAML_BUILD "0" CACHE INTERNAL "")
    endif()
    if($CACHE{LAST_YAML_BUILD} LESS ${PROFILE_FILE_TIMESTAMP})
        message("Rebuilding profile yaml files")
        set(PROFILE_NAMES "all")
        file(STRINGS "${PROJECT_SOURCE_DIR}/plugins/plugins.yaml" YAML_LINES)
        foreach(LINE_ITEM IN LISTS YAML_LINES)
            set(HAVE_VIS False)
            if(LINE_ITEM STREQUAL "")
                continue()
            elseif(LINE_ITEM MATCHES "^#")
                continue()
            endif()
            string(REPLACE " " "" ITEM ${LINE_ITEM})
            string(REPLACE ":" ";" ITEM_LIST "${ITEM}")
            list(GET ITEM_LIST 0 ENTRY)
            set(ALL_FILE "${PROJECT_SOURCE_DIR}/profiles/all.yaml")
            if(ENTRY STREQUAL "all_plugins")
                list(GET ITEM_LIST 1 PLUGIN_NAMES)
                string(REPLACE "," ";" TEMP_PLUGIN_LIST "${PLUGIN_NAMES}")
                set(PLUGIN_LIST ${TEMP_PLUGIN_LIST} CACHE INTERNAL "" FORCE)
            elseif(ENTRY STREQUAL "all_visualizers")
                list(GET ITEM_LIST 1 VIS_NAMES)
                string(REPLACE "," ";" TEMP_VIS_LIST "${VIS_NAMES}")
                set(VISUALIZER_LIST ${TEMP_VIS_LIST} CACHE INTERNAL "" FORCE)

                file(WRITE ${ALL_FILE} "# This file was auto generated, take caution if editing manually.\n")
                file(APPEND ${ALL_FILE} "plugins: ${PLUGIN_NAMES}\n")
                file(APPEND ${ALL_FILE} "visualizers: ${VIS_NAMES}\n")
                file(APPEND ${ALL_FILE} "enable_offload: false\n")
                file(APPEND ${ALL_FILE} "enable_alignment: false\n")
                file(APPEND ${ALL_FILE} "enable_verbose_errors: false\n")
                file(APPEND ${ALL_FILE} "enable_pre_sleep: false\n")
                set(VIS_NAMES "${VIS_NAMES}" CACHE INTERNAL "" FORCE)
            elseif(ENTRY STREQUAL "visualizers")
                set(USE_VIS "")
                if(NOT PROFILE_WRITING)
                    message(FATAL_ERROR "plugins/plugins.yaml has incorrect format")
                endif()
                list(GET ITEM_LIST 1 VISUALIZER)
                string(REPLACE "," ";" TEMP_VIS "${VISUALIZER}")
                foreach(V_ITEM IN LISTS TEMP_VIS)
                    set(VIS_FOUND No)
                    foreach(ITEM IN LISTS VISUALIZER_LIST)
                        if(V_ITEM STREQUAL ${ITEM})
                            list(APPEND USE_VIS ${V_ITEM})
                            set(VIS_FOUND Yes)
                        endif()
                    endforeach()
                    if(NOT VIS_FOUND)
                        message(FATAL_ERROR "Improper visualizer given, must be one of ${VISUALIZER_LIST}. ")
                    endif()
                endforeach()
            elseif(ENTRY STREQUAL "profile")
                if(PROFILE_WRITING)
                    file(APPEND ${OUTFILE} "visualizers: ${USE_VIS}\n")
                    file(APPEND ${OUTFILE} "enable_offload: ${ENABLE_OFFLOAD}\n")
                    file(APPEND ${OUTFILE} "enable_alignment: ${ENABLE_ALIGNMENT}\n")
                    file(APPEND ${OUTFILE} "enable_verbose_errors: ${ENABLE_VERBOSE_ERRORS}\n")
                    file(APPEND ${OUTFILE} "enable_pre_sleep: ${ENABLE_PRE_SLEEP}\n")
                endif()
                set(ENABLE_OFFLOAD "false")
                set(ENABLE_ALIGNMENT "false")
                set(ENABLE_VERBOSE_ERRORS "false")
                set(ENABLE_PRE_SLEEP "false")
                set(PROFILE_WRITING TRUE)
                set(USE_VIS "${VIS_NAMES}")
            elseif(ENTRY STREQUAL "name")
                if(NOT PROFILE_WRITING)
                    message(FATAL_ERROR "plugins/plugins.yaml has incorrect format")
                endif()
                list(GET ITEM_LIST 1 PROFILE_NAME)
                list(APPEND PROFILE_NAMES ${PROFILE_NAME})
                set(OUTFILE "${PROJECT_SOURCE_DIR}/profiles/${PROFILE_NAME}.yaml")
                file(WRITE ${OUTFILE} "# This file was auto generated, take caution if manually editing.\n")
            elseif(ENTRY STREQUAL "plugins")
                list(GET ITEM_LIST 1 PLUGIN_NAMES)
                file(APPEND ${OUTFILE} "plugins: ${PLUGIN_NAMES}\n")
            elseif(ENTRY STREQUAL "run")
                list(GET ITEM_LIST 1 PLUGIN_NAMES)
                file(APPEND ${OUTFILE} "run: ${PLUGIN_NAMES}\n")
            elseif(ENTRY STREQUAL "data")
                list(SUBLIST ITEM_LIST 1 -1 TDATA_LIST)
                list(JOIN TDATA_LIST ":" TDATA_FILE)
                file(APPEND ${OUTFILE} "data: ${TDATA_FILE}\n")
            elseif (ENTRY STREQUAL "build_type")
                list(GET ITEM_LIST 1 BLD_TYPE)
                file(APPEND ${OUTFILE} "build_type: ${BLD_TYPE}\n")
            elseif(ENTRY STREQUAL "enable_offload")
                list(GET ITEM_LIST 1 PREFIX)
                set(ENABLE_OFFLOAD ${PREFIX})
            elseif(ENTRY STREQUAL "enable_alignment")
                list(GET ITEM_LIST 1 EN_ALI)
                set(ENABLE_ALIGNMENT ${EN_ALI})
            elseif(ENTRY STREQUAL "enable_verbose_errors")
                list(GET ITEM_LIST 1 EN_VERB)
                set(ENABLE_VERBOSE_ERRORS ${EN_VERB})
            elseif(ENTRY STREQUAL "enable_pre_sleep")
                list(GET ITEM_LIST 1 EN_PRES)
                set(ENABLE_PRE_SLEEP ${EN_PRES})
            elseif(ENTRY STREQUAL "duration")
                list(GET ITEM_LIST 1 DURATION)
                file(APPEND ${OUTFILE} "duration: ${DURATION}\n")
            endif()
        endforeach ()
        file(APPEND ${OUTFILE} "visualizers: ${USE_VIS}\n")
        file(APPEND ${OUTFILE} "enable_offload: ${ENABLE_OFFLOAD}\n")
        file(APPEND ${OUTFILE} "enable_alignment: ${ENABLE_ALIGNMENT}\n")
        file(APPEND ${OUTFILE} "enable_verbose_errors: ${ENABLE_VERBOSE_ERRORS}\n")
        file(APPEND ${OUTFILE} "enable_pre_sleep: ${ENABLE_PRE_SLEEP}\n")

        string(TIMESTAMP CURRENT_TIME "%s" UTC)
        set(LAST_YAML_BUILD ${CURRENT_TIME} CACHE INTERNAL "")
        set(ILLIXR_PROFILE_NAMES ${PROFILE_NAMES} CACHE INTERNAL "")
    endif()
endfunction()

# read the provided yaml file for arguments/options
function(read_yaml FILENAME)
    # check that the file exists
    if(NOT EXISTS "${FILENAME}")
        FIND_FILE(FILEPATH ${FILENAME}
                PATHS profiles ${CMAKE_SOURCE_DIR} $ENV{HOME} $ENV{HOME}/.illixr
                PATH_SUFFIXES profiles
                )
        if(${FILEPATH} STREQUAL "FILEPATH-NOTFOUND")
            message(FATAL_ERROR "Could not find ${FILENAME} in any of the following paths:\n    profiles\n    ${CMAKE_SOURCE_DIR}\n    ${CMAKE_SOURCE_DIR}/profiles\n    $ENV{HOME}\n    $ENV{HOME}/profiles\n    $ENV{HOME}/.illixr\n    $ENV{HOME}/.illixr/profiles")
        endif()
        set(FILENAME ${FILEPATH})
        message(STATUS "Reading profile file ${FILENAME}")
    endif()

    set(ENABLE_OFFLOAD "false" PARENT_SCOPE)
    set(ENABLE_ALIGNMENT "false" PARENT_SCOPE)
    set(ENABLE_VERBOSE_ERRORS "false" PARENT_SCOPE)
    set(ENABLE_PRE_SLEEP "false" PARENT_SCOPE)
    file(STRINGS ${FILENAME} YAML_LINES)
    set(VIS_FOUND No)
    foreach(LINE IN LISTS YAML_LINES)
        string(REPLACE " " "" ITEM ${LINE})
        string(REPLACE ":" ";" ITEM_LIST "${ITEM}")
        list(GET ITEM_LIST 0 KEY)
        if(KEY STREQUAL "plugins")
            list(GET ITEM_LIST 1 PLUGIN_NAMES)
            string(REPLACE "," ";" TEMP_PLUGIN_LIST "${PLUGIN_NAMES}")
            foreach(PL IN LISTS TEMP_PLUGIN_LIST)
                string(TOUPPER "USE_${PL}" PL_UPPER)
                set(${PL_UPPER} ON PARENT_SCOPE)
            endforeach()
        elseif(KEY STREQUAL "visualizers")
            list(GET ITEM_LIST 1 VIS_NAMES)
            string(REPLACE "," ";" TEMP_VIS_LIST "${VIS_NAMES}")
            list(LENGTH TEMP_VIS_LIST VIS_LEN)
            if(VIS_LEN GREATER 0 AND (NOT VISUALIZER OR VISUALIZER_TO_USE STREQUAL "ALL"))
                foreach(VI IN LISTS TEMP_VIS_LIST)
                    string(TOUPPER "USE_${VI}" VI_UPPER)
                    set(${VI_UPPER} ON PARENT_SCOPE)
                endforeach ()
                set(VISUALIZER_TO_USE ${TEMP_VIS_LIST} CACHE INTERNAL "")
            elseif(NOT VISUALIZER_TO_USE STREQUAL "ALL")
                foreach(VI IN LISTS VISUALIZER_TO_USE)
                    string(TOUPPER "USE_${VI}" VI_UPPER)
                    set(${VI_UPPER} ON PARENT_SCOPE)
                endforeach()
            endif()
            set(VIS_FOUND)
        elseif(KEY STREQUAL "run")
            list(GET ITEM_LIST 1 RUN_NAMES)
            set(ILLIXR_RUN_NAMES  ${RUN_NAMES} PARENT_SCOPE)
        elseif(KEY STREQUAL "data")
            list(SUBLIST ITEM_LIST 1 -1 TDATA_LIST)
            list(JOIN TDATA_LIST ":" DATA_FILE_ITEM)
            set(DATA_FILE ${DATA_FILE_ITEM} PARENT_SCOPE)
        elseif (KEY STREQUAL "build_type")
            if(CMAKE_BUILD_TYPE STREQUAL "")
                list(GET ITEM_LIST 1 BLD_TYPE)
                set(CMAKE_BUILD_TYPE ${BLD_TYPE} PARENT_SCOPE)
            endif()
        elseif(KEY STREQUAL "install_prefix")
            if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
                list(GET ITEM_LIST 1 PREFIX)
                set(CMAKE_INSTALL_PREFIX ${PREFIX} PARENT_SCOPE)
            endif()
        elseif(KEY STREQUAL "enable_offload")
            list(GET ITEM_LIST 1 PREFIX)
            set(ENABLE_OFFLOAD ${PREFIX} PARENT_SCOPE)
        elseif(KEY STREQUAL "enable_alignment")
            list(GET ITEM_LIST 1 EN_ALI)
            set(ENABLE_ALIGNMENT ${EN_ALI} PARENT_SCOPE)
        elseif(KEY STREQUAL "enable_verbose_errors")
            list(GET ITEM_LIST 1 EN_VERB)
            set(ENABLE_VERBOSE_ERRORS ${EN_VERB} PARENT_SCOPE)
        elseif(KEY STREQUAL "enable_pre_sleep")
            list(GET ITEM_LIST 1 EN_PRES)
            set(ENABLE_PRE_SLEEP ${EN_PRES} PARENT_SCOPE)
        elseif(KEY STREQUAL "demo_data")
            list(GET ITEM_LIST 1 D_DATA)
            set(DEMO_DATA ${D_DATA} PARENT_SCOPE)
        endif()
    endforeach()
    if(NOT VIS_FOUND)
        if(VISUALIZER_TO_USE STREQUAL "ALL")
            set(VISUALIZER_TO_USE ${VISUALIZER_LIST} CACHE INTERNAL "")
        endif()
        foreach(VI IN LISTS VISUALIZER_TO_USE)
            string(TOUPPER "USE_${VI}" VI_UPPER)
            set(${VI_UPPER} ON PARENT_SCOPE)
        endforeach()
    endif()
endfunction()

# function to make sure all of the plugins actually exist
function(check_plugins)
    foreach(ITEM IN LISTS PLUGIN_LIST)
        string(TOLOWER ${ITEM} PLUGIN)
        if(NOT EXISTS "${CMAKE_SOURCE_DIR}/plugins/${PLUGIN}")
            message(FATAL_ERROR "Plugin directory for ${PLUGIN} does not exist in ${CMAKE_SOURCE_DIR}/plugins")
        endif()
    endforeach()

    foreach(ITEM IN LISTS VISUALIZER_LIST)
        string(TOLOWER ${ITEM} PLUGIN)
        if(NOT EXISTS "${CMAKE_SOURCE_DIR}/plugins/${PLUGIN}")
            message(FATAL_ERROR "Visualizer directory for ${PLUGIN} does not exist in ${CMAKE_SOURCE_DIR}/plugins")
        endif()
    endforeach()
endfunction()

# check for 3rd party dependencies that are not parts of OS repos
macro(get_external proj)
    if(NOT ${proj}_EXTERNAL)
        list(APPEND EXTERNAL_LIBRARIES "${proj}")
        set(${proj}_EXTERNAL No)
        set(${proj}_DEP_STR "")
        include(${CMAKE_SOURCE_DIR}/cmake/Get${proj}.cmake)
    endif()
endmacro()

# check for 3rd party dependencies that are not parts of OS repos, but from included plugins
macro(get_external_for_plugin proj)
    if(NOT ${proj}_EXTERNAL)
        list(APPEND EXTERNAL_LIBRARIES "${proj}")
        set(${proj}_EXTERNAL No PARENT_SCOPE)
        set(${proj}_DEP_STR "" PARENT_SCOPE)
        include(${CMAKE_SOURCE_DIR}/cmake/Get${proj}.cmake)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} PARENT_SCOPE)
        set(${proj}_EXTERNAL ${${proj}_EXTERNAL} PARENT_SCOPE)
        set(${proj}_DEP_STR ${${proj}_DEP_STR} PARENT_SCOPE)
    endif()
endmacro()

