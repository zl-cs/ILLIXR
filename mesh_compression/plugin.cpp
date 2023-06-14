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
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/io/file_utils.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>
using namespace ILLIXR;
class mesh_compression : public plugin {
    public:
        mesh_compression(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_mesh{sb->get_reader<mesh_type>("original_scene")}
        , _m_compressed_mesh{sb->get_writer<mesh_type>("compressed_scene")}
        {
            draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
            draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,pos_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,tex_coords_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,normals_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,generic_quantization_bits);
            encoder.SetSpeedOptions(speed, speed);
            sb->schedule<mesh_type>(id,"original_scene",[&](switchboard::ptr<const mesh_type> datum, std::size_t){
                    this->ProcessFrame(datum);
            });
        }

        void ProcessFrame(switchboard::ptr<const mesh_type> datum)
        {
            if(!datum->mesh.empty())
            {
               if(!datum->compressed)
               {
                   std::cout<< "receives uncompressed mesh size :"<< datum->mesh.size()<<std::endl;
               }
               printf("================================Mesh Compression: Scene %u received==========================\n", datum->id);
               std::time_t currentTime = std::time(nullptr);
               std::string timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<<datum->id<<" Compression start time: " << timeString <<std::endl;
               
               draco::PlyDecoder *ply_decoder = new draco::PlyDecoder();
               //ply_decoder->buffer_.Init(&buffer[0], buffer.size());
               ply_decoder->buffer_.Init(datum->mesh.data(), datum->mesh.size());
               
               std::unique_ptr<draco::Mesh> draco_mesh(new draco::Mesh()); 
               ply_decoder->DecodeFromBuffer(&ply_decoder->buffer_, draco_mesh.get());

               std::unique_ptr<draco::PointCloud> draco_pc;
               draco::Mesh *temp_mesh = draco_mesh.get();
               draco_pc = std::move(draco_mesh);
               
               expert_encoder.reset(new draco::ExpertEncoder(*temp_mesh));
               expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*draco_pc));
               
               draco::EncoderBuffer draco_buffer;
               const draco::Status status = expert_encoder->EncodeToBuffer(&draco_buffer);
               if(!status.ok()){
                   printf("Failed to encode the mesh\n");
               }
               printf("compressed draco buffer size %zu\n", draco_buffer.size());
               //compressed_mesh.clear();
               //compressed_mesh.reserve(draco_buffer.size());
               //compressed_mesh.assign(draco_buffer.data(), draco_buffer.data()+draco_buffer.size());
               //_m_mesh.put(_m_mesh.allocate<mesh_type>(mesh_type{compressed_mesh,true})); 
               _m_compressed_mesh.put(_m_compressed_mesh.allocate<mesh_type>(mesh_type{*(draco_buffer.buffer()),true, datum->id})); 
              
               //for testing compression correctness 
               //std::string draco_name_1 = "infini" + std::to_string(datum->id) +".drc";
               //FILE *file = fopen(draco_name_1.c_str(), "wb");
               //fwrite(draco_buffer.data(), 1, draco_buffer.size(), file);
               //fclose(file);

               //print time
               currentTime = std::time(nullptr);
               timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<<datum->id<<" Compression complete time: " << timeString<<std::endl;;
            }
            else{  
               printf("no mesh\n"); 
            }
            frame_count++;
        }

        virtual ~mesh_compression() override{
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::writer<mesh_type> _m_compressed_mesh;
        switchboard::reader<mesh_type> _m_mesh;
        
        //draco related variables
        std::unique_ptr<draco::ExpertEncoder> expert_encoder;
        draco::Encoder encoder;
        int pos_quantization_bits = 11;
        int tex_coords_quantization_bits = 10;
        bool tex_coords_deleted = false;
        int normals_quantization_bits = 8;
        bool normals_deleted = false;
        int generic_quantization_bits = 8;
        bool generic_deleted = false;
        int compression_level = 7;
        int speed = 10-compression_level;

        std::vector<char> compressed_mesh;
        unsigned frame_count;
};

PLUGIN_MAIN(mesh_compression)
