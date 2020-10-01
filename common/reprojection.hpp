#include "phonebook.hpp"
#include "data_format.hpp"

using namespace ILLIXR;

class reprojection : public phonebook::service {
public:

	// Returns true when the reprojection service
	// has been properly initialized and is ready
	// to reproject frames.
	virtual bool initialized() = 0;

	// frame:      rendered frame
	// fresh_pose: fast pose sampled at warp time
	// result:     resulting frames. Must be pre-allocated
	virtual void reproject(const rendered_frame& frame, const fast_pose_type& fresh_pose, reprojected_frame& result) = 0;
	virtual ~reprojection() { }
};
