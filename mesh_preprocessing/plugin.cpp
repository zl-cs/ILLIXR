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
class mesh_preprocessing : public plugin {
    public:
        mesh_preprocessing(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_mesh{sb->get_reader<mesh_type>("original_scene")}
        , _m_preprocessed_mesh{sb->get_writer<draco_type>("preprocessed_scene")}
        {
            sb->schedule<mesh_type>(id,"original_scene",[&](switchboard::ptr<const mesh_type> datum, std::size_t){
                    this->ProcessFrame(datum);
            });
        }

        void ProcessFrame(switchboard::ptr<const mesh_type> datum)
        {
            if(!datum->mesh.empty())
            {
               //if(!datum->compressed){
               //    std::cout<< "receives uncompressed mesh size :"<< datum->mesh.size()<<std::endl;
               //}
               printf("================================Mesh PreProcessing: Scene %u received==========================\n", datum->id);
               std::time_t currentTime = std::time(nullptr);
               std::string timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<<datum->id<<" Preprocessing start time: " << timeString <<std::endl;
               
               std::unique_ptr<draco::PlyDecoder> ply_decoder = std::make_unique<draco::PlyDecoder>();
               //ply_decoder->buffer_.Init(&buffer[0], buffer.size());
               ply_decoder->buffer_.Init(datum->mesh.data(), datum->mesh.size());
               
               std::unique_ptr<draco::Mesh> draco_mesh = std::make_unique<draco::Mesh>(); 
               //7/1 Decouple DecodeFromBuffer (pre-processing) from EncodeToBuffer (Mesh Compression)
               ply_decoder->DecodeFromBuffer(&ply_decoder->buffer_, draco_mesh.get());
               
               _m_preprocessed_mesh.put(_m_preprocessed_mesh.allocate<draco_type>(draco_type{std::move(draco_mesh), datum->id})); 

               //print time
               currentTime = std::time(nullptr);
               timeString = std::ctime(&currentTime);
               std::cout << "Mesh: "<<datum->id<<" Preprocessing complete time: " << timeString<<std::endl;
            }
            else{  
               printf("no mesh\n"); 
            }
            frame_count++;
        }

        virtual ~mesh_preprocessing() override{
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::reader<mesh_type> _m_mesh;
        switchboard::writer<draco_type> _m_preprocessed_mesh;
        
        //draco related variables
        unsigned frame_count;
};

PLUGIN_MAIN(mesh_preprocessing)
