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

#include "illixr_ipc.pb.h"

#define PORT 12345
#define MESSAGE_SIZE 1024

std::mutex m;

class IPCServer {

public:
    ILLIXRIPC::IPCPayload* payload;

    IPCServer()
    {
        payload = new ILLIXRIPC::IPCPayload;        
    }

    ~IPCServer()
    {
        std::cout << "Shutting down" << std::endl;
        shutdownServer();
    }

    void setupServer()
    {
        int option = 1;
        // Creating socket file descriptor
        socketServer = socket(AF_INET, SOCK_STREAM, 0);
        
        // fcntl(socketServer, F_SETFL, fcntl(socketServer, F_GETFL, 0) | O_NONBLOCK);

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
        if (bind(socketServer, (struct sockaddr*)&socketAddress, sizeof(socketAddress)) < 0) {
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

    void handleConnection()
    {
        try {
            if (!isConnected)
                acceptConnection();
                                  
            char buffer[MESSAGE_SIZE];
            memset(&buffer, 0, MESSAGE_SIZE);
            if (recv(socketConnection, &buffer, MESSAGE_SIZE, 0) == 0)
                return closeConnection();

            std::string request(buffer);
            std::cout << "[SERVER] Request: " << request << std::endl;

            std::string msg;

            if (request == "data")
            {       
                readPayloadFromFile("mesh.objp");
                msg = serializeMessage();
            }
            else
            {
                msg = "Invalid Command";
                return;
            }
            
            send(socketConnection, msg.c_str(), msg.size(), 0);
        }
        catch(const std::exception& e) {
            std::cerr << "Socket Thread: " << e.what() << std::endl;
            closeConnection();
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
        delete payload;
    }

    void parseObjToPayload(std::string filename)
    {
        auto start = std::chrono::high_resolution_clock::now();

        ILLIXRIPC::Mesh* mesh = payload->add_meshes();
        std::ifstream file(filename, std::ios::in);
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);

        std::string line;
        std::string type;
        
        while (std::getline(buffer, line))
        {
            std::istringstream iss(line);
            iss >> type;        
            if (type == "v")
            {
                float x, y, z;
                iss >> x >> y >> z;
                ILLIXRIPC::Vector3* vertex = mesh->add_vertices();
                vertex->set_x(x); vertex->set_y(y); vertex->set_z(z);
            }
            else if (type == "vn")
            {
                float x, y, z;
                iss >> x >> y >> z;            
                ILLIXRIPC::Vector3* normal = mesh->add_normals();
                normal->set_x(x); normal->set_y(y); normal->set_z(z);
            }
            else if (type == "f")
            {
                std::string faceData;                    
                iss >> faceData;                
                mesh->add_faces(stoi(faceData.substr(0, faceData.find("/"))) - 1);
                iss >> faceData;
                mesh->add_faces(stoi(faceData.substr(0, faceData.find("/"))) - 1);
                iss >> faceData;
                mesh->add_faces(stoi(faceData.substr(0, faceData.find("/"))) - 1);
            }
        }

        payload->set_name("Mesh Vertex Count: " + std::to_string(mesh->vertices().size()));
        
        auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);

        std::cout << "File " + filename + " "
            + "read in " + std::to_string(duration1.count()) + "ms, "
            + "processed in " + std::to_string(duration2.count()) + "ms" 
            << std::endl;
    }

    void savePayloadToFile(std::string filename)
    {
        std::string msg;        
        payload->SerializeToString(&msg);
        std::ofstream file(filename, std::ios::out);
        file << msg;
        file.close();
    }

    void readPayloadFromFile(std::string filename)
    {        
        auto start = std::chrono::high_resolution_clock::now();

        std::ifstream file(filename, std::ios::in);
        std::stringstream buffer;
        buffer << file.rdbuf();
        payload->ParseFromString(buffer.str());

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);

        std::cout << "File " + filename + " " + "read in " + std::to_string(duration.count()) + "ms" << std::endl;
        // std::cout << payload->DebugString() << std::endl;

        file.close();
    }

private:

    int socketServer, socketConnection, port;
    struct sockaddr_in socketAddress {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(PORT)
    };    
    int addrlen = sizeof(socketAddress);

    bool isConnected = false;

    std::string serializeMessage()
    {
        std::string msg;        
        payload->SerializeToString(&msg);        
        std::string msgSize = std::to_string(msg.size());
        msgSize.insert(msgSize.size(), 10 - msgSize.size(), ' ');
        msg = msgSize + msg;
        return msg;
    }
};

#endif