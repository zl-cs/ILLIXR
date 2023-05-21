//this code decouples mesh compression from InfiniTAM, this receives the obj file from InfiniTAM and performs draco compression on it and publishes in compressed draco format
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/relative_clock.hpp"
#include "common/phonebook.hpp"

#include <iostream>
#include <stdio.h>
using namespace ILLIXR;
class TestMesh : public plugin {
    public:
        TestMesh(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}
        , _m_compressed_mesh{sb->get_reader<mesh_type>("compressed_scene")}
        //, _m_compressed_mesh{sb->get_reader<mesh_type>("original_scene")}
        {
            sb->schedule<mesh_type>(id,"compressed_scene",[&](switchboard::ptr<const mesh_type> datum, std::size_t){ 
                    this->ProcessFrame(datum);
            });
            frame_count=0;
        }

        void ProcessFrame(switchboard::ptr<const mesh_type> datum)
        {
            printf("================================Test Mesh: compressed mesh %d received==========================\n", datum->id);
            if(!datum->mesh.empty())
            {
               if(datum->compressed)
               {
                   printf("retrived compressed mesh\n");
               }
               printf("size %zu \n", datum->mesh.size());
               std::string draco_name = "draco_" + std::to_string(datum->id) +".drc";
               FILE *file = fopen(draco_name.c_str(), "wb");
               size_t write_size = fwrite(datum->mesh.data(), 1, datum->mesh.size(), file);
               fclose(file);
               if(write_size != datum->mesh.size())
               {
                   printf("error!!!!!!!!!!!!!!\n");
               }
               frame_count++;
           }else{
               printf("no mesh\n");
           }
        }

        virtual ~TestMesh() override{
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::reader<mesh_type> _m_compressed_mesh;
        
        std::vector<char> compressed_mesh;

        unsigned frame_count;
};

PLUGIN_MAIN(TestMesh)
