#include <GL/glew.h>
#include <GL/gl.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>


static void GLAPIENTRY MessageCallback([[maybe_unused]] GLenum source, [[maybe_unused]] GLenum type, [[maybe_unused]] GLuint id,
                                       [[maybe_unused]] GLenum severity, [[maybe_unused]] GLsizei length,
                                       [[maybe_unused]] const GLchar* message, [[maybe_unused]] const void* userParam)
{
#ifndef NDEBUG
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) {
        return; // Don't show notification-severity messages.
    }
    fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
            (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, severity, message);
#endif
}

static auto init_and_link(const char* vertex_shader, const char* fragment_shader) -> GLuint
{

    // GL handles for intermediary objects.
    GLint result;
    GLint vertex_shader_handle;
    GLint fragment_shader_handle;
    GLint shader_program;


    vertex_shader_handle = glCreateShader(GL_VERTEX_SHADER);
    GLint vshader_len    = strlen(vertex_shader);
    glShaderSource(vertex_shader_handle, 1, &vertex_shader, &vshader_len);
    glCompileShader(vertex_shader_handle);
    glGetShaderiv(vertex_shader_handle, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        constexpr std::size_t              MSG_BUF_LENGTH = 4096U;
        std::array<GLchar, MSG_BUF_LENGTH> msg;
        const std::size_t                  msg_size = MSG_BUF_LENGTH * sizeof(GLchar);
        GLsizei                            length;
        glGetShaderInfoLog(vertex_shader_handle, msg_size, &length, msg.data());
        printf("1 Error: %s\n", msg.data());
    }

    GLint fragResult       = GL_FALSE;
    fragment_shader_handle = glCreateShader(GL_FRAGMENT_SHADER);
    GLint fshader_len      = strlen(fragment_shader);
    glShaderSource(fragment_shader_handle, 1, &fragment_shader, &fshader_len);
    glCompileShader(fragment_shader_handle);
    if (glGetError() != 0U) {
        printf("Fragment shader compilation failed\n");
    }
    glGetShaderiv(fragment_shader_handle, GL_COMPILE_STATUS, &fragResult);
    if (fragResult == GL_FALSE) {
        constexpr std::size_t              MSG_BUF_LENGTH = 4096U;
        std::array<GLchar, MSG_BUF_LENGTH> msg;
        const std::size_t                  msg_size = MSG_BUF_LENGTH * sizeof(GLchar);
        GLsizei                            length;
        glGetShaderInfoLog(fragment_shader_handle, msg_size, &length, msg.data());
        printf("2 Error: %s\n", msg.data());
    }

    // Create program and link shaders
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader_handle);
    glAttachShader(shader_program, fragment_shader_handle);
    if (glGetError() != 0U) {
        printf("AttachShader or createProgram failed\n");
    }

    ///////////////////
    // Link and verify

    glLinkProgram(shader_program);

    if (glGetError() != 0U) {
        printf("Linking failed\n");
    }

    glGetProgramiv(shader_program, GL_LINK_STATUS, &result);
    GLenum err = glGetError();
    if (err != 0U) {
        printf("initGL, error getting link status, %x", err);
    }
    if (result == GL_FALSE) {
        GLsizei length = 0;

        std::vector<GLchar> infoLog(length);
        glGetProgramInfoLog(shader_program, length, &length, &infoLog[0]);

        std::string error_msg(infoLog.begin(), infoLog.end());
        std::cout << error_msg;
    }

    if (glGetError() != 0U) {
        printf("initGL, error at end of initGL");
    }

    // After successful link, detach shaders from shader program
    glDetachShader(shader_program, vertex_shader_handle);
    glDetachShader(shader_program, fragment_shader_handle);

    return shader_program;
}
