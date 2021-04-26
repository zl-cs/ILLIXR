
#include "common/threadloop.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"


//3/18 use RBGD dataset
#include "data_loading.hpp"

using namespace ILLIXR;
//yihan: using threadloop
class infinitam : public threadloop {
public:
	infinitam(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}        
    {
		printf("================================InfiniTAM: setup finished==========================\n");

	}

    virtual skip_option _p_should_skip() override{
            return skip_option::skip_and_yield;
    }
    void _p_one_iteration() override 
	{
		printf("================================InfiniTAM: Info received==========================\n");
	}

	virtual ~infinitam() override {
		printf("================================InfiniTAM: Finished ==========================\n");
	}

private:
	const std::shared_ptr<switchboard> sb;
};

PLUGIN_MAIN(infinitam)
