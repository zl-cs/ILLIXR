#include <GL/glew.h>
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

#include <zlib.h>

#include "gldemo_output.pb.h"

static constexpr int   EYE_TEXTURE_WIDTH   = ILLIXR::FB_WIDTH;
static constexpr int   EYE_TEXTURE_HEIGHT  = ILLIXR::FB_HEIGHT;

using namespace ILLIXR;

class client_reader : public plugin {
public:
    client_reader(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
		, _m_fast_pose{sb->get_buffered_reader<switchboard::event_wrapper<fast_pose_type>>("fast_pose")}
		, uncompressed_size{ILLIXR::FB_HEIGHT*ILLIXR::FB_WIDTH*3}
    { 

		eCAL::Initialize(0, NULL, "GLdemo Offloading Frame Reader");
		subscriber = eCAL::protobuf::CSubscriber<gldemo_output_proto::Rendered_frame>("gldemo_output");
		subscriber.AddReceiveCallback(
		std::bind(&client_reader::ReceiveGLdemoOutput, this, std::placeholders::_2));

		createSharedEyebuffer(&eyetextures[0]);
		createSharedEyebuffer(&eyetextures[1]);
	}

private:
	const std::shared_ptr<switchboard> sb; 
	const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<rendered_frame> _m_eyebuffer; 
	switchboard::buffered_reader<switchboard::event_wrapper<fast_pose_type>> _m_fast_pose; 

	eCAL::protobuf::CSubscriber<gldemo_output_proto::Rendered_frame> subscriber;

	uLong uncompressed_size;
	unsigned char* outbuff[2] = {nullptr};

	GLuint eyetextures[2];

	int createSharedEyebuffer(GLuint* texture_handle){

		// Create the shared eye texture handle.
		glGenTextures(1, texture_handle);
		glBindTexture(GL_TEXTURE_2D, *texture_handle);

		// Set the texture parameters for the texture that the FBO will be
		// mapped into.
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, EYE_TEXTURE_WIDTH, EYE_TEXTURE_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

		glBindTexture(GL_TEXTURE_2D, 0); // unbind texture, will rebind later

        const GLenum gl_err = glGetError();
		if (gl_err != GL_NO_ERROR) {
			RAC_ERRNO_MSG("[device_rx] failed error check in createSharedEyebuffer");
			return 1;
		} else {
			RAC_ERRNO();
			return 0;
		}
	}

    void ReceiveGLdemoOutput(const gldemo_output_proto::Rendered_frame& frame) {
		std::cout << "Received one rendered_frame\n";
		auto fast_pose = _m_fast_pose.dequeue(); 
		if (fast_pose == nullptr) return;

		// uncompress the texture images
		for (int eye_idx = 0; eye_idx < 2; eye_idx++) {
			if (outbuff[eye_idx] != nullptr) { free(outbuff[eye_idx]); outbuff[eye_idx] = nullptr; } 
			outbuff[eye_idx] = (unsigned char*)malloc(uncompressed_size);
			int is_success = uncompress(outbuff[eye_idx], &uncompressed_size, 
										eye_idx == 0 ? reinterpret_cast<const unsigned char*>(frame.left().pixels().c_str()) : reinterpret_cast<const unsigned char*>(frame.right().pixels().c_str()), 
										eye_idx == 0 ? frame.left().size() : frame.right().size()); 
			if (is_success != Z_OK) { 
				ILLIXR::abort("Image uncompression failed !!! ");
			}
			// copy texture image from CPU to GPU
			glBindTexture(GL_TEXTURE_2D, eyetextures[eye_idx]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, EYE_TEXTURE_WIDTH, EYE_TEXTURE_HEIGHT, 0, GL_RED, GL_UNSIGNED_BYTE, outbuff[eye_idx]);
		}

		// GLint swizzleMask[] = {GL_RED, GL_RED, GL_RED, GL_RED};
		// glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);

	
		_m_eyebuffer.put(_m_eyebuffer.allocate<rendered_frame>(
			rendered_frame{
                std::array<GLuint, 2>{ eyetextures[0] , eyetextures[1] },
                std::array<GLuint, 2>{ frame.si().front(), frame.si().back() },
                **fast_pose,
                (**fast_pose).predict_computed_time,
                _m_clock->now()
			}
		));
		std::cout << "Published one rendered_frame\n"; 

	}; 

};
PLUGIN_MAIN(client_reader)