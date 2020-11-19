// Common parameters. Ultimately, these need to be moved to a yaml file.

#pragma once

#include "relative_clock.hpp"

#include <math.h>
#include <stdexcept>
#include <string>

namespace ILLIXR {

/// Display parameters
struct display_params {
    // Display width in pixels
    static constexpr unsigned width_pixels = 2560;

    // Display height in pixels
    static constexpr unsigned height_pixels = 1440;

    // Display width in meters
    static constexpr float width_meters = 0.11047f;

    // Display height in meters
    static constexpr float height_meters = 0.06214f;

    // Separation between lens centers in meters
    static constexpr float lens_separation = width_meters / 2.0f;

    // Vertical position of the lens in meters
    static constexpr float lens_vertical_position = height_meters / 2.0f;

    // Display horizontal field-of-view in degrees
    static constexpr float fov_x = 90.0f;

    // Display vertical field-of-view in degrees
    static constexpr float fov_y = 90.0f;

    // Meters per tangent angle at the center of the HMD (required by timewarp_gl's distortion correction)
    static constexpr float meters_per_tan_angle = width_meters / (2 * (fov_x * M_PI / 180.0f));

    // Inter-pupilary distance (ipd) in meters
    static constexpr float ipd = 0.064f;

    // Display refresh rate in Hz
    static constexpr float frequency = 120.0f;

    // Display period in nanoseconds
    static constexpr duration period = freq2period(frequency);

    // Chromatic aberration constants
    static constexpr float aberration[4] = {-0.016f, 0.0f, 0.024f, 0.0f};
};

/// Rendering parameters
struct rendering_params {
    // Near plane distance in meters
    static constexpr float near_z = 0.1f;

    // Far plane distance in meters
    static constexpr float far_z = 20.0f;

// CONFIGURABLE FUNCS from branch issue-94


/**
 * (Macro expansion) Function used to create string literals from variable names
 */
#ifndef STRINGIFY
#define STRINGIFY_HELP(X) #X
#define STRINGIFY(X) STRINGIFY_HELP(X)
#endif // STRINGIFY

/**
 * (Macro expansion) Function used to declare constants used by modules
 *
 * _NAME:       The constant's name to be used by module code
 * _TYPE:       The type name for the constant
 * _FROM_STR:   The conversion function from a std::string to a value of type _TYPE
 * _VALUE:      The default value for the constant
 *
 * TODO: Support dynamic type casting
 * TODO: Support more than just longs/integers
 * TODO: Support constexpr
 */
#ifndef DECLARE_CONST
#define DECLARE_CONST(_NAME, _TYPE, _FROM_STR, _VALUE) \
static const std::string VAL_STR_##_NAME{ var_from_env(STRINGIFY(_NAME)) }; \
static const _TYPE _NAME{ (VAL_STR_##_NAME.empty()) ? _VALUE : _FROM_STR(VAL_STR_##_NAME) }
#endif // DECLARE_CONST


std::string var_from_env(const char * const var_name) noexcept
{
    char* const c_str{ std::getenv(var_name) };
    return (c_str == nullptr) ? std::string{""} : std::string{c_str};
}


/**
 * @brief Convert a string containing a (python) boolean to the bool type
 */
inline bool str_to_bool(std::string var) {
    return (var == "True") ? true
        : (var == "False") ? false
                           : throw std::runtime_error("Invalid conversion from std::string to bool");
}


DECLARE_CONST(DEFAULT_FB_WIDTH,     int,    std::stoi,  2560); // Pixels
DECLARE_CONST(DEFAULT_FB_HEIGHT,    int,    std::stoi,  1440); // Pixels
DECLARE_CONST(DEFAULT_RUN_DURATION, long,   std::stol,  60L ); // Seconds
DECLARE_CONST(DEFAULT_REFRESH_RATE, double, std::stod,  60.0); // Hz

}
};
