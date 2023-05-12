#include "common/data_format.hpp"
#include "common/error_util.hpp"
#include "common/global_module_defs.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"

#include <iostream>

#include "ipc_server.cpp"

using namespace ILLIXR;

class unreal_ar : public threadloop {
public:
    unreal_ar(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
    {
        std::cout << "SERVER CONSTRUCTOR" << std::endl;
    }

private:
    IPCServer* _m_ipcServer;
    ILLIXRIPC::Mesh* _m_mesh;

public:
    virtual void _p_thread_setup() override {
        std::cout << "SERVER THREAD SETUP" << std::endl;
        _m_ipcServer = new IPCServer;
        _m_ipcServer->setupServer();
        // Add mesh to payload
        // mesh = _m_ipcServer->payload->add_meshes();
        std::cout << "SERVER THREAD SETUP COMPLETE" << std::endl;
    }

    virtual skip_option _p_should_skip() override {        
        // Ideally, we check if the buffer has any meaningful information first.
        // Otherwise, we can just skip.
        // std::cout << "SERVER SHOULD SKIP" << std::endl;
        // if (strcmp(_m_buffer, "close") == 0) {
        //     return skip_option::stop;
        // } else {
        //     return skip_option::run;
        // }
        return skip_option::run;
    }

    virtual void _p_one_iteration() override {
        std::cout << "SERVER ONE ITERATION" << std::endl;
        
        // Use to set latest draco buffer
        //_m_ipcServer->payload->set_dracomesh(buffer.str());

        // Use to decode draco buffer into ipc payload mesh
        // dracoDecode(buffer.str(), mesh);

        _m_ipcServer->handleConnection();
    }

    virtual void stop() override {        
        delete _m_ipcServer;
    }
};

PLUGIN_MAIN(unreal_ar)