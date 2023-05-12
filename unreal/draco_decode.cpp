#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cinttypes>
#include <chrono>

#include "draco/compression/decode.h"
#include "illixr_ipc.pb.h"

void dracoDecode(std::string data, ILLIXRIPC::Mesh* mesh) 
{
    auto start = std::chrono::high_resolution_clock::now();

    // Create a draco decoding buffer. Note that no data is copied in this step.
    draco::DecoderBuffer buffer;
    buffer.Init(data.data(), data.size());
    
    // Decode the input data into a geometry.
    std::unique_ptr<draco::PointCloud> pc;
    draco::Mesh *dracoMesh = nullptr;
    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
    //   if (!type_statusor.ok()) {
    //     return ReturnError(type_statusor.status());
    //   }
    const draco::EncodedGeometryType geom_type = type_statusor.value();
    if (geom_type == draco::TRIANGULAR_MESH) {        
        draco::Decoder decoder;
        auto statusor = decoder.DecodeMeshFromBuffer(&buffer);        
        std::unique_ptr<draco::Mesh> in_mesh = std::move(statusor).value();
        if (in_mesh) {
            dracoMesh = in_mesh.get();
            pc = std::move(in_mesh);
        }
    } 
    else if (geom_type == draco::POINT_CLOUD) {
        // Failed to decode it as mesh, so let's try to decode it as a point cloud.        
        draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
        pc = std::move(statusor).value();
    }
    
    const draco::PointAttribute* attribute;        
    ILLIXRIPC::Mesh* ipcMesh = mesh;    

    std::cout << dracoMesh->num_attributes() << std::endl;

    attribute = dracoMesh->GetNamedAttribute(draco::GeometryAttribute::Type::POSITION);    
    if (attribute != nullptr)
    {
        std::cout << "Vertices: " << (int)attribute->num_components() << std::endl;
        std::cout << "Vertices: " << (int)attribute->indices_map_size() << std::endl;
        float dracoVertex[3];        
        for (int i = 0; i < attribute->indices_map_size(); ++i) { 
            attribute->GetMappedValue(draco::PointIndex(i), dracoVertex);        
            ILLIXRIPC::Vector3* ipcVertex = ipcMesh->add_vertices();
            ipcVertex->set_x(dracoVertex[0]);
            ipcVertex->set_y(dracoVertex[1]);
            ipcVertex->set_z(dracoVertex[2]);
        }
    }
    
    std::cout << "Faces: " << (int)dracoMesh->num_faces() << std::endl;
    for (draco::FaceIndex faceIndex(0); faceIndex < dracoMesh->num_faces(); ++faceIndex) 
    {
        ipcMesh->add_faces(dracoMesh->face(faceIndex).data()[0].value());
        ipcMesh->add_faces(dracoMesh->face(faceIndex).data()[1].value());
        ipcMesh->add_faces(dracoMesh->face(faceIndex).data()[2].value());
    }

    attribute = dracoMesh->GetNamedAttribute(draco::GeometryAttribute::Type::COLOR);        
    if (attribute != nullptr)
    {
        std::cout << "Colors: " << attribute->num_components() << std::endl;    
        float dracoColors[3];        
        for (int i = 0; i < attribute->indices_map_size(); ++i) { 
            attribute->GetMappedValue(draco::PointIndex(i), dracoColors);
            ILLIXRIPC::Color* ipcColor = ipcMesh->add_vertexcolors();
            ipcColor->set_r(dracoColors[0]);
            ipcColor->set_g(dracoColors[1]);
            ipcColor->set_b(dracoColors[2]);
        }
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "Decoded in " + std::to_string(duration.count()) + "ms" << std::endl;

    return;
}