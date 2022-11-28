#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <cmath>
#include <array>
#include <numeric> 
#include <GL/glew.h>
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/extended_window.hpp"
#include "common/shader_util.hpp"
#include "common/math_util.hpp"
#include "common/pose_prediction.hpp"
#include "common/gl_util/obj.hpp"
#include "shaders/demo_shader.hpp"
#include "common/global_module_defs.hpp"
#include "common/error_util.hpp"

#include <iomanip>
#include <fstream>
#include <numeric>
#include <boost/filesystem.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "common/gl_util/lib/stb_image_write.h"

using namespace ILLIXR;

static constexpr int   EYE_TEXTURE_WIDTH   = ILLIXR::FB_WIDTH;
static constexpr int   EYE_TEXTURE_HEIGHT  = ILLIXR::FB_HEIGHT;

static constexpr duration VSYNC_PERIOD {freq2period(60.0)};
static constexpr duration VSYNC_DELAY_TIME {std::chrono::milliseconds{2}};

// Monado-style eyebuffers:
// These are two eye textures; however, each eye texture
// represnts a swapchain. eyeTextures[0] is a swapchain of
// left eyes, and eyeTextures[1] is a swapchain of right eyes


class gldemo_ : public threadloop {
public:
	// Public constructor, create_component passes Switchboard handles ("plugs")
	// to this constructor. In turn, the constructor fills in the private
	// references to the switchboard plugs, so the component can read the
	// data whenever it needs to.

	gldemo_(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, xwin{new xlib_gl_extended_window{1, 1, pb->lookup_impl<xlib_gl_extended_window>()->glc}}
		, sb{pb->lookup_impl<switchboard>()}
		//, xwin{pb->lookup_impl<xlib_gl_extended_window>()}
		// , pp{pb->lookup_impl<pose_prediction>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_vsync{sb->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")}
		, _m_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}	
		, _m_frame{sb->get_writer<frame_from_gldemo>("frame")}
		, _m_pose{sb->get_buffered_reader<pose_type>("pose_to_gldemo")}
		, enable_dump(false)
	{ 

	}

	virtual void stop() override {
		timer_file << "Rendering takes " << std::reduce(durationRendering.begin(), durationRendering.end(), 0) / durationRendering.size() << " ns on average\n"; 
		timer_file << "Copying textures (GPU->CPU) takes " << std::reduce(durationCopy.begin(), durationCopy.end(), 0) / durationCopy.size() << " ns on average\n";

		threadloop::stop();
	}

	void _p_thread_setup() override {
		RAC_ERRNO_MSG("gldemo at start of _p_thread_setup");

		lastTime = _m_clock->now();

		// Note: glXMakeContextCurrent must be called from the thread which will be using it.
        [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));
		assert(gl_result && "glXMakeCurrent should not fail");

		RAC_ERRNO_MSG("gldemo at end of _p_thread_setup");
	}

	void _p_one_iteration() override {
		{
			// Essentially, XRWaitFrame.
			// wait_vsync();

			startRenderTime = _m_clock->now(); 

			glUseProgram(demoShaderProgram);
			glBindFramebuffer(GL_FRAMEBUFFER, eyeTextureFBO);

			glUseProgram(demoShaderProgram);
			glBindVertexArray(demo_vao);
			glViewport(0, 0, EYE_TEXTURE_WIDTH, EYE_TEXTURE_HEIGHT);

			glEnable(GL_CULL_FACE);
			glEnable(GL_DEPTH_TEST);

			glClearDepth(1);

			// We'll calculate this model view matrix
			// using fresh pose data, if we have any.
			Eigen::Matrix4f modelViewMatrix;

			Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();

			// const fast_pose_type fast_pose = pp->get_fast_pose();
			// pose_type pose = fast_pose.pose;
			std::cout << "POSE QUEUE SIZE: " << _m_pose.size() << "\n"; 
			auto pose = _m_pose.dequeue();
			if (pose == nullptr) return;
			
			// pose_file << "position: " << pose->position.x() << "\t" << pose->position.y() << "\t" << pose->position.z() << "\n";
			// pose_file << "orientation: " << pose->orientation.w() << "\t" << pose->orientation.x() << "\t" << pose->orientation.y() << "\t" << pose->orientation.z() << "\n"; 

			Eigen::Matrix3f head_rotation_matrix = pose->orientation.toRotationMatrix();

			// 64mm IPD, why not
			// (TODO FIX, pull from centralized config!)
			// 64mm is also what TW currently uses through HMD::GetDefaultBodyInfo.
			// Unfortunately HMD:: namespace is currently private to TW. Need to
			// integrate as a config topic that can share HMD info.
			float ipd = 0.0640f; 

			// Excessive? Maybe.
			constexpr int LEFT_EYE = 0;

			for(auto eye_idx = 0; eye_idx < 2; eye_idx++) {

				// Offset of eyeball from pose
				auto eyeball = Eigen::Vector3f((eye_idx == LEFT_EYE ? -ipd/2.0f : ipd/2.0f), 0, 0);

				// Apply head rotation to eyeball offset vector
				eyeball = head_rotation_matrix * eyeball;

				// Apply head position to eyeball
				eyeball += pose->position;

				// Build our eye matrix from the pose's position + orientation.
				Eigen::Matrix4f eye_matrix = Eigen::Matrix4f::Identity();
				eye_matrix.block<3,1>(0,3) = eyeball; // Set position to eyeball's position
				eye_matrix.block<3,3>(0,0) = pose->orientation.toRotationMatrix();

				// Objects' "view matrix" is inverse of eye matrix.
				auto view_matrix = eye_matrix.inverse();

				Eigen::Matrix4f modelViewMatrix = modelMatrix * view_matrix;
				glUniformMatrix4fv(modelViewAttr, 1, GL_FALSE, (GLfloat*)(modelViewMatrix.data()));
				glUniformMatrix4fv(projectionAttr, 1, GL_FALSE, (GLfloat*)(basicProjection.data()));
				
				glBindTexture(GL_TEXTURE_2D, eyeTextures[eye_idx]);
				glFramebufferTexture(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, eyeTextures[eye_idx], 0);
				glBindTexture(GL_TEXTURE_2D, 0);
				glClearColor(0.9f, 0.9f, 0.9f, 1.0f);

				RAC_ERRNO_MSG("gldemo before glClear");
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                RAC_ERRNO_MSG("gldemo after glClear");
				
				demoscene.Draw();
			}

			glFinish();

			endRenderTime = _m_clock->now();
			durationRendering.push_back((endRenderTime-startRenderTime).count()); 
			timer_file << (endRenderTime-startRenderTime).count() << "\n"; 

#ifndef NDEBUG
			const double frame_duration_s = duration2double(_m_clock->now() - lastTime);
            const double fps = 1.0 / frame_duration_s;

			if (log_count > LOG_PERIOD) {
                std::cout << "\033[1;32m[GL DEMO APP]\033[0m Submitting frame to buffer " << which_buffer
                          << ", frametime: " << frame_duration_s
                          << ", FPS: " << fps
                          << std::endl;
			}
#endif
			lastTime = _m_clock->now();

			startCopyTime = _m_clock->now();

			GLubyte* images[2]; // FIXME

			// copy the texture image from the GPU to the CPU
			for (auto eye_idx = 0; eye_idx < 2; eye_idx++) {
				glBindTexture(GL_TEXTURE_2D, eyeTextures[eye_idx]);
				images[eye_idx] = readTextureImage();
			}

			endCopyTime = _m_clock->now();
			durationCopy.push_back((endCopyTime-startCopyTime).count()); 
			timer_copy_file << (endCopyTime-startCopyTime).count() << "\n"; 

			_m_frame.put(_m_frame.allocate<frame_from_gldemo>(
				frame_from_gldemo{
					images[0],
					images[1],
					std::array<GLuint, 2>{ which_buffer, which_buffer },
				}
			));

			if (enable_dump) {
				// dump the images
				std::string image_name = dump_dir + std::to_string(dump_idx++) + ".png";
				// Write image
				is_success = stbi_write_png(image_name.c_str(), ILLIXR::FB_WIDTH, ILLIXR::FB_HEIGHT, 3, images[0], 0); // what should be the stride? 
				if (!is_success)
				{
					ILLIXR::abort("Image dump failed !!! ");
				} 
			}

			which_buffer = !which_buffer;
		}

#ifndef NDEBUG
		if (log_count > LOG_PERIOD) {
			log_count = 0;
		} else {
			log_count++;
		}
#endif

        RAC_ERRNO_MSG("gldemo at end of _p_one_iteration");
	}

#ifndef NDEBUG
	size_t log_count = 0;
	size_t LOG_PERIOD = 20;
#endif

private:
	const std::unique_ptr<const xlib_gl_extended_window> xwin;
	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> pp;
	const std::shared_ptr<const RelativeClock> _m_clock;
	const switchboard::reader<switchboard::event_wrapper<time_point>> _m_vsync;

	// Switchboard plug for application eye buffer.
	// We're not "writing" the actual buffer data,
	// we're just atomically writing the handle to the
	// correct eye/framebuffer in the "swapchain".
	switchboard::writer<rendered_frame> _m_eyebuffer;
	switchboard::writer<frame_from_gldemo> _m_frame;
	switchboard::buffered_reader<pose_type> _m_pose; 

	// int itr; // FIXME for dubuging, print received pose
	// std::string pose_name = "metrics/offloaded_poses.txt";
	// std::ofstream pose_file;

	GLuint eyeTextures[2];
	GLuint eyeTextureFBO;
	GLuint eyeTextureDepthTarget;

	unsigned char which_buffer = 0;

	GLuint demo_vao;
	GLuint demoShaderProgram;

	GLuint vertexPosAttr;
	GLuint vertexNormalAttr;
	GLuint modelViewAttr;
	GLuint projectionAttr;

	GLuint colorUniform;

	ObjScene demoscene;

	Eigen::Matrix4f basicProjection;

	time_point lastTime;

	bool enable_dump; 
	int dump_idx = 0; 
	bool is_success;
	std::string dump_dir = "metrics/offload_gldemo/";

	// to record timing
	time_point startRenderTime, endRenderTime; 
	std::vector<_clock_rep> durationRendering;
	time_point startCopyTime, endCopyTime; 
	std::vector<_clock_rep> durationCopy;
	std::string timer_file_name = "metrics/render_time.txt";
	std::ofstream timer_file;
	std::string timer_copy_name = "metrics/copy_time.txt";
	std::ofstream timer_copy_file;

	// PBO buffer for reading texture image
	GLuint PBO_buffer;
	GLenum err;

	GLubyte* readTextureImage(){

		GLubyte* pixels = new GLubyte[EYE_TEXTURE_WIDTH * EYE_TEXTURE_HEIGHT * 3];

		// Start timer
		// startGetTexTime = std::chrono::high_resolution_clock::now();

		// Enable PBO buffer
		glBindBuffer(GL_PIXEL_PACK_BUFFER, PBO_buffer);
		err = glGetError();
		if (err){
			std::cerr << "Timewarp: glBindBuffer to PBO_buffer failed" << std::endl;
		}

		// Read texture image to PBO buffer
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)0);
		err = glGetError();
		if (err){
			std::cerr << "Timewarp: glGetTexImage failed" << std::endl;
		}

		// Transfer texture image from GPU to Pinned Memory(CPU)
		GLubyte *ptr = (GLubyte*)glMapNamedBuffer(PBO_buffer, GL_READ_ONLY);
		err = glGetError();
		if (err){
			std::cerr << "Timewarp: glMapNamedBuffer failed" << std::endl;
		}

		// Copy texture to CPU memory
		memcpy(pixels, ptr, EYE_TEXTURE_WIDTH * EYE_TEXTURE_HEIGHT * 3);

		// Unmap the buffer
		glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
		err = glGetError();
		if (err){
			std::cerr << "Timewarp: glUnmapBuffer failed" << std::endl;
		}

		// Unbind the buffer
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
		err = glGetError();
		if (err){
			std::cerr << "Timewarp: glBindBuffer to 0 failed" << std::endl;
		}

		// Terminate timer
		// endGetTexTime = std::chrono::high_resolution_clock::now();

		// Record the image collection time
		// offload_time = std::chrono::duration_cast<std::chrono::milliseconds>(endGetTexTime - startGetTexTime).count();

// #ifndef NDEBUG
// 		std::cout << "Texture image collecting time: " << offload_time << std::endl;
// #endif

		return pixels;
	}

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
			RAC_ERRNO_MSG("[gldemo] failed error check in createSharedEyebuffer");
			return 1;
		} else {
			RAC_ERRNO();
			return 0;
		}
	}

	void createFBO(GLuint* texture_handle, GLuint* fbo, GLuint* depth_target){
        RAC_ERRNO_MSG("gldemo at start of createFBO");

		// Create a framebuffer to draw some things to the eye texture
		glGenFramebuffers(1, fbo);

		// Bind the FBO as the active framebuffer.
    	glBindFramebuffer(GL_FRAMEBUFFER, *fbo);
		glGenRenderbuffers(1, depth_target);
    	glBindRenderbuffer(GL_RENDERBUFFER, *depth_target);
    	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, EYE_TEXTURE_WIDTH, EYE_TEXTURE_HEIGHT);
    	//glRenderbufferStorageMultisample(GL_RENDERBUFFER, fboSampleCount, GL_DEPTH_COMPONENT, EYE_TEXTURE_WIDTH, EYE_TEXTURE_HEIGHT);

    	glBindRenderbuffer(GL_RENDERBUFFER, 0);

		// Bind eyebuffer texture
        std::cout << "About to bind eyebuffer texture, texture handle: " << *texture_handle << std::endl;

		glBindTexture(GL_TEXTURE_2D, *texture_handle);
		glFramebufferTexture(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, *texture_handle, 0);
    	glBindTexture(GL_TEXTURE_2D, 0);

		// attach a renderbuffer to depth attachment point
    	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, *depth_target);

		if (glGetError()) {
            std::cerr << "displayCB, error after creating fbo" << std::endl;
    	}
        RAC_ERRNO_MSG("gldemo after calling glGetError");

		// Unbind FBO.
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

public:
	/* compatibility interface */

	// Dummy "application" overrides _p_start to control its own lifecycle/scheduling.
	virtual void start() override {
		RAC_ERRNO_MSG("gldemo at start of gldemo start function");

        [[maybe_unused]] const bool gl_result_0 = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc));
		assert(gl_result_0 && "glXMakeCurrent should not fail");
		RAC_ERRNO_MSG("gldemo after glXMakeCurrent");

		// Init and verify GLEW
		const GLenum glew_err = glewInit();
		if (glew_err != GLEW_OK) {
            std::cerr << "[gldemo] GLEW Error: " << glewGetErrorString(glew_err) << std::endl;
            ILLIXR::abort("[gldemo] Failed to initialize GLEW");
		}
		RAC_ERRNO_MSG("gldemo after glewInit");

		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);

		// Create two shared eye textures.
		// Note; each "eye texture" actually contains two eyes.
		// The two eye textures here are actually for double-buffering
		// the Switchboard connection.
		createSharedEyebuffer(&(eyeTextures[0]));
		createSharedEyebuffer(&(eyeTextures[1]));

        RAC_ERRNO_MSG("gldemo after creating eye buffers");

		// Initialize FBO and depth targets, attaching to the frame handle
		createFBO(&(eyeTextures[0]), &eyeTextureFBO, &eyeTextureDepthTarget);

        RAC_ERRNO_MSG("gldemo after creating FBO");

		// Create and bind global VAO object
		glGenVertexArrays(1, &demo_vao);
    	glBindVertexArray(demo_vao);

		demoShaderProgram = init_and_link(demo_vertex_shader, demo_fragment_shader);
#ifndef NDEBUG
		std::cout << "Demo app shader program is program " << demoShaderProgram << std::endl;
#endif

		vertexPosAttr = glGetAttribLocation(demoShaderProgram, "vertexPosition");
		vertexNormalAttr = glGetAttribLocation(demoShaderProgram, "vertexNormal");
		modelViewAttr = glGetUniformLocation(demoShaderProgram, "u_modelview");
		projectionAttr = glGetUniformLocation(demoShaderProgram, "u_projection");
		colorUniform = glGetUniformLocation(demoShaderProgram, "u_color");
		RAC_ERRNO_MSG("gldemo after glGetUniformLocation");

		// Load/initialize the demo scene.

		char* obj_dir = std::getenv("ILLIXR_DEMO_DATA");
		if (obj_dir == nullptr) {
            ILLIXR::abort("Please define ILLIXR_DEMO_DATA.");
		}

		demoscene = ObjScene(std::string(obj_dir), "scene.obj");

		// Construct a basic perspective projection
		math_util::projection_fov( &basicProjection, 40.0f, 40.0f, 40.0f, 40.0f, 0.03f, 20.0f );

		// for offloading, reading texture images
		glGenBuffers(1, &PBO_buffer);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, PBO_buffer);
        glBufferData(GL_PIXEL_PACK_BUFFER, EYE_TEXTURE_WIDTH * EYE_TEXTURE_HEIGHT * 3, 0, GL_STREAM_DRAW);

		RAC_ERRNO_MSG("gldemo before glXMakeCurrent");
        [[maybe_unused]] const bool gl_result_1 = static_cast<bool>(glXMakeCurrent(xwin->dpy, None, nullptr));
		assert(gl_result_1 && "glXMakeCurrent should not fail");
		RAC_ERRNO_MSG("gldemo after glXMakeCurrent");

		// pose_file.open("metrics/offloaded_pose.txt"); 
		timer_file.open(timer_file_name); 
		timer_copy_file.open(timer_copy_name); 

		if (enable_dump) {
			boost::filesystem::path p(dump_dir);
			boost::filesystem::remove_all(p);
			boost::filesystem::create_directories(p);
			stbi_flip_vertically_on_write(true);
		}

		// Effectively, last vsync was at zero.
		// Try to run gldemo right away.
		threadloop::start();

		RAC_ERRNO_MSG("gldemo at end of start()");
	}

};

PLUGIN_MAIN(gldemo_)
