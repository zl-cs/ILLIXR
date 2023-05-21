#ifndef IPC_SERVER
#define IPC_SERVER

#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <random>
#include <mutex>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <chrono>

//#include "illixr_ipc.pb.h"
#include <sys/select.h>

#define PORT 12345
#define MESSAGE_SIZE 1024

class IPCServer {

public:
    //ILLIXRIPC::IPCPayload* payload;
    unsigned payload_id;
    std::string payload_msg;

    IPCServer()
    {
        //payload = new ILLIXRIPC::IPCPayload;
    }

    ~IPCServer()
    {
        std::cout << "Shutting down" << std::endl;
        shutdownServer();
    }

    void setupServer()
    {
        socketAddress.sin_family = AF_INET;
        socketAddress.sin_addr.s_addr = INADDR_ANY;
        socketAddress.sin_port = htons(PORT);
        addrlen = sizeof(socketAddress);

        int option = 1;
        // Creating socket file descriptor
        socketServer = socket(AF_INET, SOCK_STREAM, 0);
        
        //fcntl(socketServer, F_SETFL, fcntl(socketServer, F_GETFL, 0) | O_NONBLOCK);

        if (socketServer < 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port 8080
        if (setsockopt(socketServer, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port        
        if (bind(socketServer, (struct sockaddr*)&socketAddress, addrlen) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        
        if (listen(socketServer, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
    }

    void acceptConnection()
    {        
        std::cout << "Ready for Connection" << std::endl;
        socketConnection = accept(socketServer, (struct sockaddr*)&socketAddress, (socklen_t*)&addrlen);

        if (socketConnection < 0)
            isConnected = false;
        else
        {
            isConnected = true;
            std::cout << "Connected to client" << std::endl;
        }
    }
    void setupConnection()
    {
        //set up connection only
        if(!isConnected)
        {
            fd_set fds;
            struct timeval timeout;
            FD_ZERO(&fds);
            FD_SET(socketServer, &fds);
            timeout.tv_usec = 100000;
            
            int selectResult = select(socketServer + 1, &fds, nullptr, nullptr, &timeout);
            if(selectResult == -1){
                perror("select");
                return;
            }else if (selectResult == 0){
                std::cout << "Timeout: No connection request received within the specified time." << std::endl;
                return;
            }else
            {
                printf("received request\n");
                acceptConnection();
            }
        }
    }
    void recvMessage()
    {
        char buffer[MESSAGE_SIZE];
        memset(&buffer, 0, MESSAGE_SIZE);
        
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socketConnection, &read_fds);
        
        struct timeval recvTimeout;
        recvTimeout.tv_sec = 0;
        recvTimeout.tv_usec=8333;//120fps
        int select_status = select(socketConnection + 1, &read_fds, NULL, NULL, &recvTimeout);
        if(select_status < 0) {
            perror("select");
            return;
        }
        else if (select_status > 0 && FD_ISSET(socketConnection, &read_fds)) 
        {
            if (recv(socketConnection, &buffer, MESSAGE_SIZE, 0) == 0)
            {
                std::cout<<"triggered close connection\n";
                return closeConnection();
            }
            std::string request(buffer);
            std::cout << "[SERVER] Request: " << request << std::endl;


            if (request == "data")
            {
                if(payload_msg.empty())
                {
                    std::string msg = "Invalid Command";
                    printf("IPCServer: no payload found\n");
                    send(socketConnection, msg.c_str(), msg.size(), 0);
                }
                else
                {
                    printf("IPC server: sending payload id %u, size: %zu\n",payload_id, payload_msg.size()); 
                    send(socketConnection, payload_msg.c_str(), payload_msg.size(), 0);
                }
            }
        }
    }

    void closeConnection()
    {
        close(socketConnection);
        isConnected = false;
        std::cout << "Connection closed" << std::endl;
    }

    void shutdownServer() {
        closeConnection();
        shutdown(socketServer, SHUT_RDWR); 
        //delete payload;
    }


private:

    int socketServer, socketConnection;
    struct sockaddr_in socketAddress;
    int addrlen;

    bool isConnected = false;

};

#endif
