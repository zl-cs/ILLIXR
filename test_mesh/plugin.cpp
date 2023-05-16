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
        , _m_compressed_mesh{sb->get_reader<mesh_demo_type>("original_scene")}
        //, _m_compressed_mesh{sb->get_reader<mesh_type>("original_scene")}
        {
            sb->schedule<mesh_demo_type>(id,"original_scene",[&](switchboard::ptr<const mesh_demo_type> datum, std::size_t){ 
                    this->ProcessFrame(datum);
            });
        }

        void ProcessFrame(switchboard::ptr<const mesh_demo_type> datum)
        {
            printf("================================Test Mesh: mesh %d received==========================\n", datum->id);
            printf("mesh path: %s\n", datum->path.c_str());
        }

        virtual ~TestMesh() override{
        }

    private:
        //ILLIXR related variables
        const std::shared_ptr<switchboard> sb;
        switchboard::reader<mesh_demo_type> _m_compressed_mesh;
};

PLUGIN_MAIN(TestMesh)
