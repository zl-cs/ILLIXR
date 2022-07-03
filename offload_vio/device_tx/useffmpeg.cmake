###################
# UseFFmpeg.cmake #
###################

OPTION(WITH_FFMPEG "Build with FFmpeg support?" ON)

if (WITH_FFMPEG)
    # use pkg-config to get the directories and then use these values
    # in the FIND_PATH() and FIND_LIBRARY() calls
    find_package(PkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(_FFMPEG_AVCODEC libavcodec)
        pkg_check_modules(_FFMPEG_AVFORMAT libavformat)
        pkg_check_modules(_FFMPEG_AVFILTER libavfilter)
        pkg_check_modules(_FFMPEG_AVUTIL libavutil)
    endif (PKG_CONFIG_FOUND)

    find_path(FFMPEG_AVCODEC_INCLUDE_DIR
    NAMES libavcodec/avcodec.h
    PATHS ${_FFMPEG_AVCODEC_INCLUDE_DIRS} /usr/include /usr/local/include /opt/local/include /sw/include
    PATH_SUFFIXES ffmpeg libav
    )

    find_library(FFMPEG_LIBAVCODEC
    NAMES avcodec
    PATHS ${_FFMPEG_AVCODEC_LIBRARY_DIRS} /usr/lib64 /usr/local/lib /opt/local/lib /sw/lib
    )

    find_library(FFMPEG_LIBAVFORMAT
    NAMES avformat
    PATHS ${_FFMPEG_AVFORMAT_LIBRARY_DIRS} /usr/lib64 /usr/local/lib /opt/local/lib /sw/lib
    )

    find_library(FFMPEG_LIBAVFILTER
    NAMES avfilter
    PATHS ${_FFMPEG_AVFILTER_LIBRARY_DIRS} /usr/lib64 /usr/local/lib /opt/local/lib /sw/lib
    )

    find_library(FFMPEG_LIBAVUTIL
    NAMES avutil
    PATHS ${_FFMPEG_AVUTIL_LIBRARY_DIRS} /usr/lib64 /usr/local/lib /opt/local/lib /sw/lib
    )

    if (FFMPEG_LIBAVCODEC AND FFMPEG_LIBAVFORMAT)
        set(FFMPEG_FOUND TRUE)
    endif()

    if (FFMPEG_FOUND)
        set(FFMPEG_INCLUDE_DIR ${FFMPEG_AVCODEC_INCLUDE_DIR})
        include_directories(${FFMPEG_INCLUDE_DIR})

        add_definitions(-DCOMPILE_WITH_FFMPEG)
        set(FFMPEG_LIBRARIES
            ${FFMPEG_LIBAVCODEC}
            ${FFMPEG_LIBAVFORMAT}
            ${FFMPEG_LIBAVFILTER}
            ${FFMPEG_LIBAVUTIL}
        )
    else (FFMPEG_FOUND)
        message(FATAL_ERROR "Could not find libavcodec or libavformat or libavutil")
    endif (FFMPEG_FOUND)
endif()
