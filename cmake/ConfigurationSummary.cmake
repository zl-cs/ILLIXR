# formatting definitions
if(COLOR_OUTPUT)
    string(ASCII 27 ESC)
    set(RESET_FORMAT "${ESC}[m")
    set(BOLD "${ESC}[1m")
    set(COLOR_WHITE "${ESC}[37m")
    set(COLOR_WHITE_BOLD "${ESC}[1;37m")
    set(COLOR_RED "${ESC}[31m")
    set(COLOR_RED_BOLD "${ESC}[1;31m")
    set(COLOR_YELLOW "${ESC}[33m")
    set(COLOR_YELLOW_BOLD "${ESC}[1;33m")
    set(COLOR_GREEN "${ESC}[32m")
    set(COLOR_GREEN_BOLD "${ESC}[1;32m")
    set(COLOR_CYAN "${ESC}[36m")
    set(COLOR_CYAN_BOLD "${ESC}[1;36m")
    set(COLOR_BLUE "${ESC}[34m")
    set(COLOR_BLUE_BOLD "${ESC}[1;34m")
    set(COLOR_MAGENTA "${ESC}[35m")
    set(COLOR_MAGENTA_BOLD "${ESC}[1;35m")
else()
    set(RESET_FORMAT "")
    set(BOLD "")
    set(COLOR_WHITE "")
    set(COLOR_WHITE_BOLD "")
    set(COLOR_RED "")
    set(COLOR_RED_BOLD "")
    set(COLOR_YELLOW "")
    set(COLOR_YELLOW_BOLD "")
    set(COLOR_GREEN "")
    set(COLOR_GREEN_BOLD "")
    set(COLOR_CYAN "")
    set(COLOR_CYAN_BOLD "")
    set(COLOR_BLUE "")
    set(COLOR_BLUE_BOLD "")
    set(COLOR_MAGENTA "")
    set(COLOR_MAGENTA_BOLD "")
endif()
# helper functions

# based on https://github.com/jimbraun/XCDF/blob/master/cmake/CMakePadString.cmake
function(pad_string RESULT REQUESTED_LENGTH VALUE)
    string(LENGTH "${VALUE}" VALUE_LENGTH)
    math(EXPR REQUIRED_PAD "${REQUESTED_LENGTH} - ${VALUE_LENGTH}")
    set(PAD ${VALUE})
    if(REQUIRED_PAD GREATER 0)
        math(EXPR REQUIRED_MINUS_ONE "${REQUIRED_PAD} - 1")
        foreach(BLAH RANGE ${REQUIRED_PAD})
            set(PAD "${PAD} ")
        endforeach()
    endif()
    set(${RESULT} "${PAD}" PARENT_SCOPE)
endfunction()

set(ILLIXR_SUMMARY_PADDING 35 CACHE STRING "Padding of each report summary line")
mark_as_advanced(ILLIXR_SUMMARY_PADDING)

function(report_value value msg)
    pad_string(padded_value ${ILLIXR_SUMMARY_PADDING} "  ${value}")
    message(STATUS "${padded_value}: ${msg}")
endfunction()

function(print_enabled_disabled value msg)
    pad_string(padded_value ${ILLIXR_SUMMARY_PADDING} "  ${msg}")
    if(value)
        message(STATUS "${padded_value}: ${COLOR_GREEN}Enabled${RESET_FORMAT}")
    else()
        message(STATUS "${padded_value}: ${COLOR_YELLOW}Disabled${RESET_FORMAT}")
    endif()
endfunction()

function(report_padded varname)
    pad_string(padded_value ${ILLIXR_SUMMARY_PADDING} " ${varname}")
    message(STATUS "${padded_value}: ${${varname}}")
endfunction()

function(shell_set var val)
    if(CUR_SHELL STREQUAL "CSH")
        message("  setenv ${var} ${val}")
    else ()
        message("  export ${var}=${val}")
    endif()
endfunction()

macro(giveExecString)
    message("")
    message("After building and installing ILLIXR, use one of the following command(s) to run it.")
    message("${COLOR_YELLOW_BOLD}Note${RESET_FORMAT}: please ensure that ${CMAKE_INSTALL_PREFIX}/bin is in your ${BOLD}PATH${RESET_FORMAT} and ${CMAKE_INSTALL_PREFIX}/lib is in your ${BOLD}LD_LIBRARY_PATH${RESET_FORMAT}")
    message("")
    if(USE_OPENVINS)
        message("To use OpenVINS")
        message("  main${ILLIXR_BUILD_SUFFIX}.exe --yaml=${CMAKE_SOURCE_DIR}/illixr.yaml --vis=openvins")
    endif()
    if(OPENXR_RUNTIME)
        string(REGEX MATCH "csh$" CSHELL $ENV{SHELL})
        if(CSHELL)
            set(CUR_SHELL "CSH")
        else()
            set(CUR_SHELL "BASH")
        endif()
        message("")
        message("To use monado use the following:")
        message("${COLOR_YELLOW_BOLD}Note${RESET_FORMAT}: It is up to you to make sure the plugins listed are compatible with the given monado library.")
        set(PLUGINS_TO_RUN_LIST "")
        foreach(ITEM IN LISTS OPENXR_PLUGINS)
            list(APPEND PLUGINS_TO_RUN_LIST "\${IXIR_PTH}/libplugin.${ITEM}${ILLIXR_BUILD_SUFFIX}.so")
        endforeach()
        string(REPLACE ";" ":" PLUGINS_TO_RUN "${PLUGINS_TO_RUN_LIST}")
        message("")
        shell_set("IXIR_PTH" "${CMAKE_INSTALL_PREFIX}")
        shell_set("ILLIXR_DATA" "${CMAKE_SOURCE_DIR}/data/mav0")
        if(DEMO_DATA)
            shell_set("ILLIXR_DEMO_DATA" "${DEMO_DATA}")
        else()
            shell_set("ILLIXR_DEMO_DATA" "${CMAKE_BINARY_DIR}/demo_data")
        endif()
        shell_set("ILLIXR_PATH" "\${ILXR_PTH}/lib/libplugin.main${ILLIXR_BUILD_SUFFIX}.so")
        shell_set("ILLIXR_COMP" "${PLUGINS_TO_RUN}")
        shell_set("XRT_TRACING" "true")
        shell_set("ILLIXR_OFFLOAD_ENABLE" "${ENABLE_OFFLOAD}")
        shell_set("ILLIXR_ALIGNMENT_ENABLE" "${ENABLE_ALIGNMENT}")
        shell_set("ILLIXR_ENABLE_VERBOSE_ERRORS" "${ENABLE_VERBOSE_ERRORS}")
        shell_set("ILLIXR_ENABLE_PRE_SLEEP" "${ENABLE_PRE_SLEEP}")
        message("")
        message("If using a realsense camera give the camera type as follows")
        shell_set("REALSENSE_CAM" "<TYPE>")
        set(BOTH_MONADO false)
        if(MONADO_RUNTIME_VK AND MONADO_RUNTIME_GL)
            set(BOTH_MONADO true)
        endif()
        message("")
        if(MONADO_RUNTIME_VK)
            if(BOTH_MONADO)
                message("To use monado_vk")
            endif()
            shell_set("XR_RUNTIME_JSON" "${MONADO_RUNTIME_VK}")
            message("")
            message("  ${CMAKE_INSTALL_PREFIX}/bin/monado-service-vk")
        elseif(MONADO_RUNTIME_GL)
            if(BOTH_MONADO)
                message("")
                message("To use monado_gl")
            endif()
            shell_set("XR_RUNTIME_JSON" "${MONADO_RUNTIME_GL}")
            message("")
            message("  ${CMAKE_INSTALL_PREFIX}/bin/monado-service-gl")
        endif()
        message("  ${OPENXR_RUNTIME}")
    endif()
    message("")
endmacro()

# Report configuration

string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPER)
message("")
message("${COLOR_BLUE_BOLD}-------------------------------------------------------------")
message("------------------- Configuration Options -------------------${RESET_FORMAT}")
if(YAML_FILE)
    report_value("Arguments read from" "Command line and ${YAML_FILE}")
else()
    report_value("Arguments read from" "Command line")
endif()
report_value("Linux vendor" "${OS_FLAVOR}")
report_value("CMAKE_CXX_COMPILER_ID type" "${CMAKE_CXX_COMPILER_ID}")
report_value("CMAKE_CXX_COMPILER_VERSION" "${CMAKE_CXX_COMPILER_VERSION}")
report_value("CMake version"    "${CMAKE_VERSION}")
report_value("CMake generator"  "${CMAKE_GENERATOR}")
report_value("CMake build tool" "${CMAKE_BUILD_TOOL}")
report_value("Build type" "${CMAKE_BUILD_TYPE}")
report_value("Library suffix" "${ILLIXR_BUILD_SUFFIX}${CMAKE_SHARED_LIBRARY_SUFFIX}")
report_value("C compilation flags" "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
report_value("C++ compilation flags" "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
report_value("Install location" "${CMAKE_INSTALL_PREFIX}")

# ------------------------- External Libraries ---------------------------------
message("${COLOR_BLUE_BOLD}External Libraries${RESET_FORMAT}")
foreach(ITEM IN LISTS EXTERNAL_LIBRARIES)
    if(${ITEM}_EXTERNAL)
        report_value("${ITEM}" "${COLOR_CYAN}Download and Install${RESET_FORMAT}")
    else()
        report_value("${ITEM}" "${COLOR_GREEN}Found version ${${ITEM}_VERSION}${RESET_FORMAT}")
    endif()
endforeach()
if(BUILDING_OPENCV)
    report_value("OpenCV" "${COLOR_CYAN}Download and Install${RESET_FORMAT}")
endif()

# ------------------------- Plugins --------------------------------------------
message("${COLOR_BLUE_BOLD}Build Plugins${RESET_FORMAT}")
set(PLUGIN_LINE "plugins: ")
set(VIS_LINE "")
set(FIRST True)
set(HAVE_PLUGIN False)
foreach(ITEM IN LISTS PLUGIN_LIST)
    string(TOUPPER "USE_${ITEM}" ITEM_UPPER)
    if(${${ITEM_UPPER}})
        if(NOT FIRST)
            set(PLUGIN_LINE "${PLUGIN_LINE},")
        else()
            set(FIRST False)
            set(HAVE_PLUGIN True)
        endif()
        set(PLUGIN_LINE "${PLUGIN_LINE}${ITEM}")
    endif()
    print_enabled_disabled("${${ITEM_UPPER}}" "${ITEM}")
    unset(ITEM_UPPER)
endforeach()

foreach(ITEM IN LISTS VISUALIZER_LIST)
    string(TOUPPER "USE_${ITEM}" ITEM_UPPER)
    if(${${ITEM_UPPER}})
        if(VIS_LINE)
            set(VIS_LINE "${VIS_LINE},")
        endif()
        set(VIS_LINE "${VIS_LINE}${ITEM}")
    endif()
    print_enabled_disabled("${${ITEM_UPPER}}" "${ITEM}")
endforeach()
message("")
report_value("Data file" "${DATA_FILE}")
report_value("Generating yaml file" "illixr.yaml")
message("${COLOR_BLUE_BOLD}-------------------------------------------------------------${RESET_FORMAT}")

set(OUTFILE "illixr.yaml")

file(WRITE ${OUTFILE} "")
if(HAVE_PLUGIN)
    file(APPEND ${OUTFILE} "${PLUGIN_LINE}\n")
    if(ILLIXR_RUN_NAMES)
        file(APPEND ${OUTFILE} "run: ${ILLIXR_RUN_NAMES}\n")
    endif()
endif()
if(VIS_LINE)
    file(APPEND ${OUTFILE} "visualizers: ${VIS_LINE}\n")
endif()
if(LOCAL_DATA)
    file(APPEND ${OUTFILE} "data: ${LOCAL_DATA}\n")
elseif(DATA_FILE)
    file(APPEND ${OUTFILE} "data: ${CMAKE_SOURCE_DIR}/data/mav0\n")
endif()
if(DEMO_DATA)
    file(APPEND ${OUTFILE} "demo_data: ${DEMO_DATA}\n")
else()
    file(APPEND ${OUTFILE} "demo_data: ${CMAKE_BINARY_DIR}/demo_data\n")
endif()
if(CMAKE_INSTALL_PREFIX)
    file(APPEND ${OUTFILE} "install_prefix: ${CMAKE_INSTALL_PREFIX}\n")
endif()
file(APPEND ${OUTFILE} "enable_offload: ${ENABLE_OFFLOAD}\n")
file(APPEND ${OUTFILE} "enable_alignment: ${ENABLE_ALIGNMENT}\n")
file(APPEND ${OUTFILE} "enable_verbose_errors: ${ENABLE_VERBOSE_ERRORS}\n")
file(APPEND ${OUTFILE} "enable_pre_sleep: ${ENABLE_PRE_SLEEP}\n")

giveExecString()

message("${COLOR_BLUE_BOLD}-------------------------------------------------------------${RESET_FORMAT}")
message("")
