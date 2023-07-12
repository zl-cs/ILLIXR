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

#include <utility>
using namespace ILLIXR;
class mesh_encoding : public plugin {
    public:
        mesh_encoding(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_preprocessed_mesh{sb->get_reader<draco_type>("preprocessed_scene")}
        , _m_compressed_mesh{sb->get_writer<mesh_type>("compressed_scene")}
        {
            draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
            draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);
            
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,pos_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,tex_coords_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,normals_quantization_bits);
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,generic_quantization_bits);
            encoder.SetSpeedOptions(speed, speed);
            
            sb->schedule<draco_type>(id,"preprocessed_scene",[&](switchboard::ptr<const draco_type> datum, std::size_t){
                    this->ProcessFrame(datum);
            });
        }

        void ProcessFrame(switchboard::ptr<const draco_type> datum)
        {
            printf("================================Mesh Encoding: Scene %u received==========================\n", datum->frame_id);
            std::time_t currentTime = std::time(nullptr);
            std::string timeString = std::ctime(&currentTime);
            std::cout << "Mesh: "<<datum->frame_id<<" Encoding start time: " << timeString <<std::endl;

            //7/1 idea use protobuf to serialize the string
            //use std::move instead of release for the following reasons:
            //std::move() is a standard library utility that casts its argument to an r-value reference, which is a way to indicate that resources of that object can be reused or moved.
            //When std::move() is used with a std::unique_ptr, it allows the unique_ptr to transfer ownership of the managed object to another unique_ptr.
            //This is a clean way to transfer ownership, because it ensures that the moved-from unique_ptr is set to nullptr and thus is in a safe state after the move. The moved-from unique_ptr will not delete the managed object when it goes out of scope.
            //If using release()
            //the responsibility of deleting the object (when it is no longer needed) is transferred to the programmer. This can lead to memory leaks if not handled properly.
            //release() is often used when you need to transfer ownership from a unique_ptr to a raw pointer or a different smart pointer type that doesn't have a constructor for taking ownership from a unique_ptr.
            expert_encoder.reset(new draco::ExpertEncoder(*(datum->preprocessed_mesh)));
            draco::PointCloud *draco_pc = datum->preprocessed_mesh.get();

            expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*draco_pc));
            draco::EncoderBuffer draco_buffer;

            const draco::Status status = expert_encoder->EncodeToBuffer(&draco_buffer);


            if(!status.ok()){
                printf("Failed to encode the mesh\n");
            }
            printf("compressed draco buffer size %zu\n", draco_buffer.size());
            _m_compressed_mesh.put(_m_compressed_mesh.allocate<mesh_type>(mesh_type{*(draco_buffer.buffer()),true, datum->frame_id})); 

            //for testing compression correctness 
            //std::string draco_name_1 = "infini" + std::to_string(datum->id) +".drc";
            //FILE *file = fopen(draco_name_1.c_str(), "wb");
            //fwrite(draco_buffer.data(), 1, draco_buffer.size(), file);
            //fclose(file);

            //print time
            currentTime = std::time(nullptr);
            timeString = std::ctime(&currentTime);
            std::cout << "Mesh: "<<datum->frame_id<<" Encoding complete time: " << timeString<<std::endl;;
            frame_count++;
        }

        virtual ~mesh_encoding() override{
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::writer<mesh_type> _m_compressed_mesh;
        switchboard::reader<draco_type> _m_preprocessed_mesh;
        
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

PLUGIN_MAIN(mesh_encoding)
