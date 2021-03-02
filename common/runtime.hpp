#pragma once

#include "extended_window.hpp"
#include <GL/glx.h>
#include <memory>
#include <vector>

namespace ILLIXR
{
class plugin;

typedef plugin* (*plugin_factory)(phonebook*);

class runtime
{
public:
    virtual void load_so(const std::vector<std::string>& so) = 0;
    virtual void load_so(std::string_view so)                = 0;
    virtual void load_plugin_factory(plugin_factory plugin)  = 0;
    virtual void wait()                                      = 0;
    virtual void stop()                                      = 0;
    virtual ~runtime()                                       = default;
};

extern "C" auto runtime_factory(GLXContext appGLCtx) -> runtime*;
}
