#include "common/data_format.hpp"
#include "common/error_util.hpp"
#include "common/global_module_defs.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"

#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#include "json.hpp"

#define PORT 12345
#define MESSAGE_SIZE 1024

using namespace ILLIXR;
using json = nlohmann::json;

class IPCPayload {
private:
    json json_data;

    void UpdateJSON()
    {
        json_data["position"] = json{ {"x", position(0)}, {"y", position(1)}, {"z", position(2)} };
        json_data["normal"] = json{ {"x", normal(0)}, {"y", normal(1)}, {"z", normal(2)} };
    }

public: 
    Eigen::Vector3f position, normal;

    IPCPayload()
    {
        UpdateJSON();
    }

    std::string stringify()
    {
        UpdateJSON();   
        return json_data.dump();
    }

};

class unreal_ar : public threadloop {
public:
    unreal_ar(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
    {
        std::cout << "SERVER CONSTRUCTOR" << std::endl;
    }

private:
    IPCPayload* _m_payload;
    int _m_socket_server, _m_socket_connection;
    int _m_valread;
    char _m_buffer[1024];
    char *_m_msg_array;

public:
    virtual void _p_thread_setup() override {
        std::cout << "SERVER THREAD SETUP" << std::endl;
        _m_payload = new IPCPayload;

        struct sockaddr_in socket_address;
        socket_address.sin_family = AF_INET;
        socket_address.sin_addr.s_addr = INADDR_ANY;
        socket_address.sin_port = htons(PORT);

        int option = 1, address_length = sizeof(socket_address);
        _m_msg_array = new char[MESSAGE_SIZE + 1];

        // Creating socket file descriptor
        _m_socket_server = socket(AF_INET, SOCK_STREAM, 0);
        
        // if (server_fd < 0) {
        //     perror("socket failed");
        //     exit(EXIT_FAILURE);
        // }

        // Forcefully attaching socket to the port 8080
        if (setsockopt(_m_socket_server, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option))) {
            std::cout << "SERVER SETSOCKOPT FAILED" << std::endl;
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port        
        if (bind(_m_socket_server, (struct sockaddr*)&socket_address, address_length) < 0) {
            std::cout << "SERVER BIND FAILED" << std::endl;
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        
        if (listen(_m_socket_server, 3) < 0) {
            std::cout << "SERVER LISTEN FAILED" << std::endl;
            perror("listen");
            exit(EXIT_FAILURE);
        }

        _m_socket_connection = accept(_m_socket_server, (struct sockaddr*)&socket_address, (socklen_t*)&address_length); // Seems to have an issue here
        if (_m_socket_connection < 0) {
            std::cout << "SERVER SOCKET CONNECTION FAILED" << std::endl;
            perror("accept");
            exit(EXIT_FAILURE);
        }

        std::cout << "SERVER THREAD SETUP COMPLETE" << std::endl;
    }

    virtual skip_option _p_should_skip() override {
        std::cout << "SERVER SHOULD SKIP" << std::endl;

        _m_valread = read(_m_socket_connection, _m_buffer, 1024);

        // Ideally, we check if the buffer has any meaningful information first.
        // Otherwise, we can just skip.

        if (strcmp(_m_buffer, "close") == 0) {
            return skip_option::stop;
        } else {
            return skip_option::run;
        }
    }

    virtual void _p_one_iteration() override {
        std::cout << "SERVER ONE ITERATION" << std::endl;

        if (strcmp(_m_buffer, "data") == 0) {
            // TODO: update the payload with the latest plane here

            std::string msg = _m_payload->stringify();
            msg += '\0';
            msg.append(MESSAGE_SIZE - msg.length(), ' '); // Add padding                

            strcpy(_m_msg_array, msg.c_str());                        
            send(_m_socket_connection, _m_msg_array, MESSAGE_SIZE, 0);
        } else {
            std::string err_msg = "Invalid Command. Use one of: \"data\", \"close\"";
            send(_m_socket_connection, err_msg.c_str(), err_msg.length(), 0);
        }
    }

    virtual void stop() override {
        // closing the connected socket
        close(_m_socket_connection);
        // closing the listening socket
        shutdown(_m_socket_server, SHUT_RDWR); 

        delete _m_payload;
        delete[] _m_msg_array;
    }
};

PLUGIN_MAIN(unreal_ar)