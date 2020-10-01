#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <shared_mutex>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/reprojection.hpp"
#include "common/extended_window.hpp"
#include "common/shader_util.hpp"
#include "utils/hmd.hpp"
#include "common/math_util.hpp"
#include "common/gl_util/obj.hpp"
#include "shaders/basic_shader.hpp"
#include "shaders/openwarp_mesh_shader.hpp"
#include "common/pose_prediction.hpp"

using namespace ILLIXR;

class openwarp_gl : public reprojection {

public:

	static constexpr size_t MESH_WIDTH = 32;
	static constexpr size_t MESH_HEIGHT = 32;

	openwarp_gl(const phonebook* pb)
		: sb{pb->lookup_impl<switchboard>()}
	{ }


	Eigen::Matrix4f getEyeballMatrix(int eye, const Eigen::Vector3f& head_pos, const Eigen::Quaternionf& head_rotation, float ipd) {
		// Offset of eyeball from pose
		auto eyeball = Eigen::Vector3f((eye == 0 ? -ipd/2.0f : ipd/2.0f), 0, 0);

		auto head_rotation_matrix = head_rotation.toRotationMatrix();

		// Apply head rotation to eyeball offset vector
		eyeball = head_rotation_matrix * eyeball;

		// Apply head position to eyeball
		eyeball += head_pos;

		// Build our eye matrix from the pose's position + orientation.
		Eigen::Matrix4f eye_matrix = Eigen::Matrix4f::Identity();
		eye_matrix.block<3,1>(0,3) = eyeball; // Set position to eyeball's position
		eye_matrix.block<3,3>(0,0) = head_rotation_matrix;

		return eye_matrix;
	}

	
	virtual bool initialized() override {
		// Acquire shared lock on R/W mutex
		// to check initialization state.
		std::shared_lock lock(init_mutex);
		return isInitialized;
	}
	virtual void reproject(const rendered_frame& frame, const fast_pose_type& fresh_pose, reprojected_frame& result) override {
		
		// Acquire shared lock on R/W mutex
		// to check initialization state.
		init_mutex.lock_shared();
		if(!isInitialized) {
			
			// If we're not initialized, we re-lock with
			// exclusive ownership, and perform initialization.
			init_mutex.unlock_shared();
			init_mutex.lock();

			// Check again if we're initialized; make sure
			// no race condition against another callback.
			if(!isInitialized) {
				
				// Good to go, initialize().
				initialize(MESH_WIDTH, MESH_HEIGHT);
			}

			isInitialized = true;
			
			// Release exclusive lock.
			init_mutex.unlock();
		} else {
			// We're initialized; release shared lock.
			init_mutex.unlock();
		}
		
		// DRAW REPROJECTION.
	}

	// virtual ~openwarp_gl() override {
	// 	// TODO: Need to cleanup resources here!
	// 	glXMakeCurrent(xwin->dpy, None, NULL);
 	// 	glXDestroyContext(xwin->dpy, xwin->glc);
 	// 	XDestroyWindow(xwin->dpy, xwin->win);
 	// 	XCloseDisplay(xwin->dpy);
	// }

private:
	// Allows reproject() to check for initialization
	// race-free.
	mutable std::shared_mutex init_mutex;
	bool isInitialized = false;
	const std::shared_ptr<switchboard> sb;

	GLuint openwarpShaderProgram;

	// Eye sampler array
	GLuint eye_sampler;

	// Depth sampler
	GLuint depth_sampler;

	// VAOs
	GLuint vao;

	// Reprojection mesh CPU buffers and GPU VBO handles
	std::vector<vertex_t> mesh_vertices;
	GLuint mesh_vertices_vbo;
	std::vector<GLuint> mesh_indices;
	GLuint mesh_indices_vbo;

	// Inverse V and P matrices of the rendered pose
	GLuint u_render_inverse_p;
	GLuint u_render_inverse_v;

	// VP matrix of the fresh pose
	GLuint u_warp_vp;

	// Basic perspective projection matrix
	Eigen::Matrix4f basicProjection;

	void initialize(size_t meshWidth, size_t meshHeight)
	{
		// Build the reprojection mesh.
		BuildMesh(meshWidth, meshHeight, mesh_indices, mesh_vertices);

		// Create and bind global VAO object
		glGenVertexArrays(1, &vao);
    	glBindVertexArray(vao);

		// Build and link shaders.
		openwarpShaderProgram = init_and_link(meshWarpVertexProgramGLSL, meshWarpFragmentProgramGLSL);

		// Get the color + depth samplers
		eye_sampler = glGetUniformLocation(openwarpShaderProgram, "Texture");
		depth_sampler = glGetUniformLocation(openwarpShaderProgram, "_Depth");

		// Get the warp matrix uniforms
		// Inverse V and P matrices of the rendered pose
		u_render_inverse_p = glGetUniformLocation(openwarpShaderProgram, "u_renderInverseInverseP");
		u_render_inverse_v = glGetUniformLocation(openwarpShaderProgram, "u_renderInverseV");
		// VP matrix of the fresh pose
		u_warp_vp = glGetUniformLocation(openwarpShaderProgram, "u_warpVP");

		// Generate, bind, and fill mesh VBOs.
		glGenBuffers(1, &mesh_vertices_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, mesh_vertices_vbo);
		glBufferData(GL_ARRAY_BUFFER, mesh_vertices.size() * sizeof(vertex_t), &mesh_vertices.at(0), GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glGenBuffers(1, &mesh_indices_vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_indices_vbo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_indices.size() * sizeof(GLuint), &mesh_indices.at(0), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	}

	// Build a rectangular plane.
	void BuildMesh(size_t width, size_t height, std::vector<GLuint>& indices, std::vector<vertex_t>& vertices){
		
		// Compute the size of the vectors we'll need to store the
		// data, ahead of time.

		// width and height are not in # of verts, but in # of faces.
		size_t num_indices = 2 * 3 * width * height;
		size_t num_vertices = (width + 1)*(height + 1);

		// Size the vectors accordingly
		indices.resize(num_indices);
		vertices.resize(num_vertices);

		// Build indices.
		for ( int y = 0; y < height; y++ ) {
			for ( int x = 0; x < width; x++ ) {

				const int offset = ( y * width + x ) * 6;

				indices[offset + 0] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 1] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 2] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 1 ) );

				indices[offset + 3] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 1 ) );
				indices[offset + 4] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 5] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 1 ) );
			}
		}

		// Build vertices
		for (int y = 0; y < height + 1; y++){
			for (int x = 0; x < width + 1; x++){

				int index = y * ( width + 1 ) + x;
				vertices[num_vertices + index].position[0] = ( -1.0f + ( (float)x / width ) );
				vertices[num_vertices + index].position[1] = ( -1.0f + 2.0f * ( ( height - (float)y ) / height ));
				vertices[num_vertices + index].position[2] = 0.0f;
			}
		}
	}



};

class openwarp_gl_plugin : public plugin {
public:
    openwarp_gl_plugin(const std::string& name, phonebook* pb)
    	: plugin{name, pb}
	{
		pb->register_impl<reprojection>(
			std::static_pointer_cast<reprojection>(
				std::make_shared<openwarp_gl>(pb)
			)
		);
	}
};

PLUGIN_MAIN(openwarp_gl_plugin)
