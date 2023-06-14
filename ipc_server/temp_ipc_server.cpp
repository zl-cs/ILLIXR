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
        //int flags = fcntl(socketServer, F_GETFL,0);
        //if (flags == -1){
        //    std::cerr << "Error getting socket flags: " << strerror(errno) << std::endl;
        //    return;
        //} 
        ////this set up the socket to be non-blocking otherwise it will be blocking on recv(), thus handle connection will not return until new message is arrived
        //if(fcntl(socketServer, F_SETFL, flags |  O_NONBLOCK) == -1){
        //    std::cerr << "Error setting socket to non-blocking mode: " << strerror(errno) << std::endl;
        //    return;
        //}

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
    //519 now this just set up connection decoupling the task to recev message
    void setupConnection()
    {
        try {
            //pyh this will hang if no connection is accept which will not be good since we want the plugin to constantly getting new payload
            //modifcation use timeout mechanism instead
            if(!isConnected)
            {
                fd_set fds;
                struct timeval timeout;
                FD_ZERO(&fds);
                FD_SET(socketServer, &fds);
                timeout.tv_usec = 100000;
                std::cout<<"reached select\n";
                int selectResult = select(socketServer + 1, &fds, nullptr, nullptr, &timeout);
                std::cout<<"finished select\n";
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
                    return;
                }
                std::cout<<"finish select\n";
            //TODO SPLIT THIS INTO TWO  one for just creating connection, one for sole responding to request
            }
            else
            {
                std::cout<<"somehow reached here\n";
            }
        }
        catch(const std::exception& e) {
            std::cerr << "Socket Thread: " << e.what() << std::endl;
            closeConnection();
        }
    }
    void recvMessage()
    {
        char buffer[MESSAGE_SIZE];
        memset(&buffer, 0, MESSAGE_SIZE);
        ssize_t bytesRead = recv(socketConnection, &buffer, MESSAGE_SIZE, 0);
        if (bytesRead == 0) 
            return closeConnection();
        else if (bytesRead == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::cout << "No data available at the moment." << std::endl;
                return;
            }
            else{
                std::cerr << "Error receiving data: " << strerror(errno) << std::endl;
                return;
            }
        }
        else
        {
            std::cout << "Received " << bytesRead << " bytes of data." << std::endl;
        }


        std::string request(buffer);
        std::cout << "[SERVER] Request: " << request << std::endl;

        //std::string msg;

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
