# module to download, build and install the Monado ILLIXR_vk plugin

set(MONADO_VK_DEPS "${OpenCV_DEP_STR}")

find_package(Vulkan MODULE)
if(NOT Vulkan_FOUND)
    pkg_check_modules(Vulkan REQUIRED vulkan)
endif()

if(BUILD_OPENCV)
    set(MONADO_VK_DEPS "${MONADO_VK_DEPS} OpenCV_Viz ")
endif()

set(MONADO_CMAKE_ARGS "")

# if building on CentOS make sure we use the correct OpenCV
if(HAVE_CENTOS)
    set(MONADO_CMAKE_ARGS "-DINTERNAL_OPENCV=${OpenCV_DIR}")
endif()

get_external_for_plugin(OpenXR_APP)

ExternalProject_Add(MonadoVK
        GIT_REPOSITORY https://github.com/ILLIXR/monado_vulkan_integration.git   # Git repo for source code
        GIT_TAG c6acfc130cfe4a181a111981ba99905eb085d0a9       # sha5 hash for specific commit to pull (if there is no specific tag to use)
        PREFIX ${CMAKE_BINARY_DIR}/_deps/monado_vk             # the build directory
        DEPENDS ${MONADO_VK_DEPS}                              # dependencies of this module
        #arguments to pass to CMake
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DILLIXR_PATH=${CMAKE_SOURCE_DIR}/include -DXRT_HAVE_LIBUDEV=ON -DXRT_HAVE_LIBUSB=ON -DXRT_HAVE_LIBUVC=OFF -DXRT_HAVE_V4L2=ON -DXRT_HAVE_SDL2=OFF -DXRT_BUILD_DRIVER_ANDROID=OFF -DXRT_BUILD_DRIVER_ARDUINO=OFF -DXRT_BUILD_DRIVER_DAYDREAM=OFF -DXRT_BUILD_DRIVER_DEPTHAI=OFF -DXRT_BUILD_DRIVER_EUROC=OFF -DXRT_BUILD_DRIVER_HANDTRACKING=OFF -DXRT_BUILD_DRIVER_HDK=OFF -DXRT_BUILD_DRIVER_HYDRA=OFF -DXRT_BUILD_DRIVER_NS=OFF -DXRT_BUILD_DRIVER_OHMD=OFF -DXRT_BUILD_DRIVER_OPENGLOVES=OFF -DXRT_BUILD_DRIVER_PSMV=OFF -DXRT_BUILD_DRIVER_PSVR=OFF -DXRT_BUILD_DRIVER_QWERTY=OFF -DXRT_BUILD_DRIVER_REALSENSE=OFF -DXRT_BUILD_DRIVER_REMOTE=OFF -DXRT_BUILD_DRIVER_RIFT_S=OFF -DXRT_BUILD_DRIVER_SURVIVE=OFF -DXRT_BUILD_DRIVER_ULV2=OFF -DXRT_BUILD_DRIVER_VF=OFF -DXRT_BUILD_DRIVER_VIVE=OFF -DXRT_BUILD_DRIVER_HANDTRACKING=OFF -DXRT_BUILD_DRIVER_WMR=OFF -DXRT_BUILD_DRIVER_SIMULAVR=OFF -DXRT_BUILD_DRIVER_SIMULATED=OFF -DXRT_BUILD_SAMPLES=OFF -DXRT_FEATURE_TRACING=OFF -DXRT_FEATURE_SERVICE=ON -DXRT_FEATURE_WINDOW_PEEK=OFF -DXRT_OPENXR_INSTALL_ABSOLUTE_RUNTIME_PATH=ON ${MONADO_CMAKE_ARGS}

        # custom install command to get the name of the plugin correct
        INSTALL_COMMAND make install && ln -sf ${CMAKE_INSTALL_PREFIX}/lib/libopenxr_monado_vk.so ${CMAKE_INSTALL_PREFIX}/lib/libopenxr_monado_vk${ILLIXR_BUILD_SUFFIX}.so
        )
set(Monado_vk_EXTERNAL Yes)
set(Monado_vk_DEP_STR "Monado_VK")
set(MONADO_RUNTIME_VK "${CMAKE_INSTALL_PREFIX}/share/openxr/1/openxr_monado_vk.json" PARENT_SCOPE)
