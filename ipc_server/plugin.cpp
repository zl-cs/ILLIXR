#include "common/data_format.hpp"
#include "common/error_util.hpp"
#include "common/global_module_defs.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"

#include <iostream>
#include "ipc_server.cpp"

using namespace ILLIXR;

class ipc_server : public threadloop {
public:
    ipc_server(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
    //require monitoring switchboard for latest payload
        , sb{pb->lookup_impl<switchboard>()}
        , _m_payload{sb->get_reader<payload_type>("mesh_payload")}
    {
        std::cout << "SERVER CONSTRUCTOR" << std::endl;
        latest_id = 0;
    }

private:
    IPCServer* _m_ipcServer;
    const std::shared_ptr<switchboard>     sb;
    switchboard::reader<payload_type> _m_payload;
    unsigned latest_id;

public:
    virtual void _p_thread_setup() override {
        std::cout << "SERVER THREAD SETUP" << std::endl;
        _m_ipcServer = new IPCServer;
        _m_ipcServer->setupServer();
        // Add mesh to payload
        // _m_mesh = _m_ipcServer->payload->add_meshes();
        std::cout << "SERVER THREAD SETUP COMPLETE" << std::endl;
    }

    virtual skip_option _p_should_skip() override {        
        //this runs at the highest frequency
        return skip_option::run;
    }

    virtual void _p_one_iteration() override {
        //std::cout << "IPC SERVER ONE ITERATION" << std::endl;
        
        //519 only accept connection when necessary
        //std::cout<<"IPC server reached setupConnection\n";
        _m_ipcServer->setupConnection();
        //std::cout<<"IPC server finished setupConnection\n";
        
        
        switchboard::ptr<const payload_type> latest_payload = _m_payload.get_ro_nullable(); 
        if (latest_payload != nullptr && latest_payload->id > latest_id)
        {
            printf("new payload detected, old id: %u, old size: %zu, new id: %u, new size: %zu \n", latest_id, _m_ipcServer->payload_msg.size(), latest_payload->id, latest_payload->payload.size());
            latest_id = latest_payload->id;
            _m_ipcServer->payload_msg = latest_payload->payload;
            _m_ipcServer->payload_id = latest_payload->id;
        }else if (latest_payload !=nullptr)
        {
        //    printf("payload detected with id: %u\n", latest_payload->id);
        }
        //std::cout<<"IPC server reached recvMessage\n";
        _m_ipcServer->recvMessage();
        
        //_m_ipcServer->handleConnection();
    }

    virtual void stop() override {        
        delete _m_ipcServer;
    }
};

PLUGIN_MAIN(ipc_server)
