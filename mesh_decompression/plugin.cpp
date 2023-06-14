//pyh this code takes insight from my earlier version of InfiniTAM and plugin and a later attempt by Henry, 
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/relative_clock.hpp"
#include "common/phonebook.hpp"


//draco related libraries
#include <draco/io/ply_reader.h>
#include <draco/io/file_reader_factory.h>
#include <draco/io/file_writer_factory.h>
#include <draco/io/stdio_file_reader.h>
#include <draco/io/stdio_file_writer.h>
#include "draco/io/ply_property_writer.h"
#include "draco/io/ply_decoder.h"
#include "draco/io/ply_encoder.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/io/file_utils.h"

#include "draco/compression/decode.h"

#include <iostream>
#include <stdio.h>
#include <ctime>

//PYH: This plugin recovers the mesh from the received draco format (incorporates Sainath's draco_decode.cpp & original draco-decoder.cc) 
using namespace ILLIXR;
class mesh_decompression : public plugin {
    public:
        mesh_decompression(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_compressed_mesh{sb->get_reader<mesh_type>("compressed_scene")}
        , _m_mesh{sb->get_writer<mesh_demo_type>("decompressed_scene")}
        {
            draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open); 
            sb->schedule<mesh_type>(id,"compressed_scene",[&](switchboard::ptr<const mesh_type> datum, std::size_t){
                    this->ProcessFrame(datum);
            });
        }

        void ProcessFrame(switchboard::ptr<const mesh_type> datum)
        {
            if(!datum->mesh.empty())
            {
               if(!datum->compressed)
               {
                   std::cout<< "ERROR!!!!receives uncompressed mesh size :"<< datum->mesh.size()<<std::endl;
               }
               printf("================================Mesh DeCompression: Scene %u received==========================\n", datum->id);
               std::time_t currentTime = std::time(nullptr);
               std::string timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<< datum->id<<" DeCompression start time: " << timeString <<std::endl;
               
               draco::DecoderBuffer buffer;
               buffer.Init(datum->mesh.data(), datum->mesh.size());
               
               std::unique_ptr<draco::PointCloud> pc;
               draco::Mesh *dracoMesh = nullptr;
               draco::Decoder decoder;

               auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
               const draco::EncodedGeometryType geom_type = type_statusor.value(); 
               if (geom_type == draco::TRIANGULAR_MESH) { 
                   auto statusor = decoder.DecodeMeshFromBuffer(&buffer);
                   std::unique_ptr<draco::Mesh> in_mesh = std::move(statusor).value();
                   if(in_mesh){
                       dracoMesh = in_mesh.get();
                       pc = std::move(in_mesh);
                   }
               }
               //similar to decodemeshforunity() in original draco
               std::vector<int> f_1;
               std::vector<int> f_2;
               std::vector<int> f_3;
               std::vector<float> v_x;
               std::vector<float> v_y;
               std::vector<float> v_z;
               std::vector<unsigned> c_r;
               std::vector<unsigned> c_g;
               std::vector<unsigned> c_b;

               const draco::PointAttribute*  attribute = dracoMesh->GetNamedAttribute(draco::GeometryAttribute::Type::POSITION);
               if(attribute != nullptr){
                   std::cout << "Vertices: " << (int)attribute->num_components() << std::endl;
                   std::cout << "Vertices: " << (int)attribute->indices_map_size() << std::endl;
                   float dracoVertex[3];
                   for(unsigned i = 0; i < attribute->indices_map_size();++i){
                        attribute->GetMappedValue(draco::PointIndex(i), dracoVertex);
                        v_x.push_back(dracoVertex[0]);
                        v_y.push_back(dracoVertex[1]);
                        v_z.push_back(dracoVertex[2]);
                   }
               }

               std::cout << "Faces: " << (int)dracoMesh->num_faces() << std::endl;
               for (draco::FaceIndex faceIndex(0); faceIndex < dracoMesh->num_faces(); ++faceIndex)
               {
                   f_1.push_back(dracoMesh->face(faceIndex).data()[0].value());
                   f_2.push_back(dracoMesh->face(faceIndex).data()[1].value());
                   f_3.push_back(dracoMesh->face(faceIndex).data()[2].value());
               }
               attribute = dracoMesh->GetNamedAttribute(draco::GeometryAttribute::Type::COLOR);
               //const int color_att_id = dracoMesh->GetNamedAttributeId(draco::GeometryAttribute::COLOR);
               //std::cout<<"color type: "<<draco::GetAttributeDataType(color_att_id)<<std::endl;
               if(attribute != nullptr){
                   uint8_t dracoColors[3];
                   std::cout << "Colors: " << attribute->num_components() << std::endl;    
                   for (unsigned i = 0; i < attribute->indices_map_size(); ++i) {
                       attribute->GetMappedValue(draco::PointIndex(i), dracoColors);
                        //this is something between 0-2.55 something need to look at how EncodeToFile works
                        //printf("color r: %d, g: %d, b:%d\n", dracoColors[0], dracoColors[1], dracoColors[2]);
                        c_r.push_back(dracoColors[0]);
                        c_g.push_back(dracoColors[1]);
                        c_b.push_back(dracoColors[2]);
                   }
               } 
               //Test by printing out the decoded mesh
               std::string out_mesh = std::to_string(datum->id) + ".ply";
               draco::PlyEncoder *ply_encoder = new draco::PlyEncoder();
               ply_encoder->EncodeToFile(*dracoMesh, out_mesh);
               std::string merge_name = std::to_string(datum->id);

               _m_mesh.put(_m_mesh.allocate<mesh_demo_type>(mesh_demo_type{v_x,v_y,v_z,c_r,c_g,c_b,f_1,f_2,f_3, datum->id})); 
                                          //
               //_m_compressed_mesh.put(_m_compressed_mesh.allocate<mesh_type>(mesh_type{*(draco_buffer.buffer()),true, datum->id})); 
               currentTime = std::time(nullptr);
               timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<< datum->id<<" DeCompression complete time: " << timeString<<std::endl;;
            }
            else{  
               printf("no mesh\n"); 
            }
            frame_count++;
        }

        virtual ~mesh_decompression() override{
            //mainEngine->SaveSceneToMesh(output_mesh_name.c_str());
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::reader<mesh_type> _m_compressed_mesh;
        switchboard::writer<mesh_demo_type> _m_mesh;
        
        //draco related variables

        unsigned frame_count;
};

PLUGIN_MAIN(mesh_decompression)
