#include "common/threadloop.hpp"
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

// #include <cstdlib>
// #include <iostream>

#include "UIEngine.h"
// 
// #include "InputSource/OpenNIEngine.h"
// #include "InputSource/Kinect2Engine.h"
// #include "InputSource/LibUVCEngine.h"
// #include "InputSource/PicoFlexxEngine.h"
// #include "InputSource/RealSenseEngine.h"
// #include "InputSource/LibUVCEngine.h"
// #include "InputSource/RealSense2Engine.h"
// #include "InputSource/FFMPEGReader.h"
// #include "ITMLib/ITMLibDefines.h"
// #include "ITMLib/Core/ITMBasicEngine.h"
// #include "ITMLib/Core/ITMBasicSurfelEngine.h"
// #include "ITMLib/Core/ITMMultiEngine.h"


// #include "../../InputSource/OpenNIEngine.h"
// #include "../../InputSource/Kinect2Engine.h"
// #include "../../InputSource/LibUVCEngine.h"
// #include "../../InputSource/PicoFlexxEngine.h"
// #include "../../InputSource/RealSenseEngine.h"
// #include "../../InputSource/LibUVCEngine.h"
// #include "../../InputSource/RealSense2Engine.h"
// #include "../../InputSource/FFMPEGReader.h"
// #include "../../ITMLib/ITMLibDefines.h"
// #include "../../ITMLib/Core/ITMBasicEngine.h"
// #include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
// #include "../../ITMLib/Core/ITMMultiEngine.h"


// #include "data_loading.hpp"

using namespace ILLIXR;

class infinitam : public threadloop {

public:
	infinitam(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sb{pb->lookup_impl<switchboard>()}        
    {
		std::cout << "================================InfiniTAM: setup finished==========================" << std::endl;
	}

    virtual skip_option _p_should_skip() override
	{
            return skip_option::skip_and_yield;
    }

    void _p_one_iteration() override 
	{
		std::cout << "================================InfiniTAM: Info received==========================" << std::endl;
	}

	virtual ~infinitam() override
	{
		std::cout << "destructor" << std::endl;
	}

private:
	const std::shared_ptr<switchboard> sb;
};

PLUGIN_MAIN(infinitam)
