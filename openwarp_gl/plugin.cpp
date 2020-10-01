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

	static constexpr size_t MESH_WIDTH = 128;
	static constexpr size_t MESH_HEIGHT = 128;

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
				initialize(MESH_WIDTH, MESH_HEIGHT, frame.eyebuffer_width, frame.eyebuffer_height);
			}

			isInitialized = true;
			
			// Release exclusive lock.
			init_mutex.unlock();
		} else {
			// We're initialized; release shared lock.
			init_mutex.unlock();
		}
		
		// DRAW REPROJECTION.

		glUseProgram(openwarpShaderProgram);
		
		glBindVertexArray(vao);
		
		

		// Upload the inverse projection matrix
		glUniformMatrix4fv(u_render_inverse_p, 1, GL_FALSE, (GLfloat*)(frame.render_projection.inverse().eval().data()));



		for(auto eye_idx = 0; eye_idx < 2; eye_idx++) {
			
			auto render_eye_matrix = getEyeballMatrix(eye_idx, frame.render_pose.pose.position, frame.render_pose.pose.orientation, 0.0640f);
			// Upload the inverse view matrix (eye matrix) from the rendered pose
			glUniformMatrix4fv(u_render_inverse_v, 1, GL_FALSE, (GLfloat*)render_eye_matrix.data());

			auto fresh_eye_matrix = getEyeballMatrix(eye_idx, fresh_pose.pose.position, fresh_pose.pose.orientation, 0.0640f);
			auto fresh_vp = frame.render_projection * fresh_eye_matrix.inverse();
			
			// Upload the VP matrix from the predicted pose
			glUniformMatrix4fv(u_warp_vp, 1, GL_FALSE, (GLfloat*)fresh_vp.eval().data());


			glBindFramebuffer(GL_FRAMEBUFFER, result_FBOs[eye_idx]);
			glViewport(0, 0, frame.eyebuffer_width, frame.eyebuffer_height);
			glDisable(GL_CULL_FACE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(GL_TRUE);
			glClear(GL_DEPTH_BUFFER_BIT);

			glBindBuffer(GL_ARRAY_BUFFER, mesh_vertices_vbo);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_t), (void*)offsetof(vertex_t, position));
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vertex_t), (void*)offsetof(vertex_t, uv));

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, frame.texture_handles[eye_idx]);
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, frame.depth_handles[eye_idx]);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_indices_vbo);
			glDrawElements(GL_TRIANGLES, mesh_indices.size(), GL_UNSIGNED_INT, NULL);
		}

		glBindVertexArray(0);

		result.texture_handles[0] = result_textures[0];
		result.texture_handles[1] = result_textures[1];

		return;
		
	}

private:
	// Allows reproject() to check for initialization
	// race-free.
	mutable std::shared_mutex init_mutex;
	bool isInitialized = false;
	const std::shared_ptr<switchboard> sb;

	GLuint result_textures[2];
	GLuint result_FBOs[2];

	GLuint openwarpShaderProgram;

	GLuint depthTargets[2];

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

	int createSharedEyebuffer(GLuint* texture_handle, GLuint width, GLuint height){

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
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

		glBindTexture(GL_TEXTURE_2D, 0); // unbind texture, will rebind later

		if(glGetError()){
			return 0;
		} else {
			return 1;
		}
	}

	int createFBO(GLuint* texture_handle, GLuint* fbo, GLuint* depth_target, GLuint width, GLuint height){
		// Create a framebuffer to draw some things to the eye texture
		glGenFramebuffers(1, fbo);
		// Bind the FBO as the active framebuffer.
    	glBindFramebuffer(GL_FRAMEBUFFER, *fbo);

		glGenRenderbuffers(1, depth_target);
		glBindRenderbuffer(GL_RENDERBUFFER, *depth_target);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, *depth_target);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glBindTexture(GL_TEXTURE_2D, *texture_handle);
		glFramebufferTexture(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, *texture_handle, 0);
    	glBindTexture(GL_TEXTURE_2D, 0);
		

		// Unbind FBO.
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		if(glGetError()){
			return 0;
		} else {
			return 1;
		}
	}

	void initialize(size_t meshWidth, size_t meshHeight, size_t eyebufferWidth, size_t eyebufferHeight)
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
		u_render_inverse_p = glGetUniformLocation(openwarpShaderProgram, "u_renderInverseP");
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

		createSharedEyebuffer(&result_textures[0], eyebufferWidth, eyebufferHeight);
		createSharedEyebuffer(&result_textures[1], eyebufferWidth, eyebufferHeight);

		createFBO(&result_textures[0], &result_FBOs[0], &depthTargets[0], eyebufferWidth, eyebufferHeight);
		createFBO(&result_textures[1], &result_FBOs[1], &depthTargets[1], eyebufferWidth, eyebufferHeight);

		glBindVertexArray(0);

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
		for (size_t y = 0; y < height + 1; y++){
			for (size_t x = 0; x < width + 1; x++){

				size_t index = y * ( width + 1 ) + x;
				// vertices[index].position[0] = ( -1.0f + 2.0f * ( (float)x / width ) );
				// vertices[index].position[1] = ( -1.0f + 2.0f * ( ( height - (float)y ) / height ));
				// vertices[index].position[2] = 0.0f;

				vertices[index].uv[0] = ((float)x / width);
				vertices[index].uv[1] = ((( height - (float)y) / height));

				if(x == 0) {
					vertices[index].uv[0] = -0.5f;
				}
				if(x == width) {
					vertices[index].uv[0] = 1.5f;
				}

				if(y == 0) {
					vertices[index].uv[1] = 1.5f;
				}
				if(y == height) {
					vertices[index].uv[1] = -0.5f;
				}


				
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
