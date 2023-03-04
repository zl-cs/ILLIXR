//
// Created by steven on 7/3/22.
//

#include "video_encoder.h"

#include "common/error_util.hpp"

#include <chrono>
#include <gst/app/gstappsrc.h>
#include <gst/gl/x11/gstgldisplay_x11.h>
#include <gst/video/video-info.h>
#include <thread>

namespace ILLIXR {
// }

GstFlowReturn cb_new_sample(GstElement* appsink, gpointer* user_data) {
    return reinterpret_cast<video_encoder*>(user_data)->cb_appsink(appsink);
}

video_encoder::video_encoder(std::function<void(const GstMapInfo&, const GstMapInfo&)> callback, Display* display)
    : _callback(std::move(callback))
    , _display{display} { }

//
//    void cb_need_data(GstElement *appsrc, guint size, gpointer* user_data) {
//        reinterpret_cast<video_encoder*>(user_data)->want = 1;

void video_encoder::init_gl() {
    GstGLDisplay* display = GST_GL_DISPLAY(gst_gl_display_x11_new_with_display(_display));
    _context              = gst_gl_context_new(display);
    gst_object_unref(display);

    GError* error = nullptr;
    if (!gst_gl_context_create(_context, nullptr, &error)) {
        std::cerr << "Failed to create GL context: " << error->message << std::endl;
        g_error_free(error);
        exit(1);
    }
}

void video_encoder::create_pipelines() {
    gst_init(nullptr, nullptr);

    _appsrc_img0  = gst_element_factory_make("appsrc", "appsrc_img0");
    _appsrc_img1  = gst_element_factory_make("appsrc", "appsrc_img1");
    _appsink_img0 = gst_element_factory_make("appsink", "appsink_img0");
    _appsink_img1 = gst_element_factory_make("appsink", "appsink_img1");

    auto caps_filter_0 = gst_element_factory_make("capsfilter", "caps_filter0");
    auto caps_filter_1 = gst_element_factory_make("capsfilter", "caps_filter1");

    auto gl_color_convert_0 = gst_element_factory_make("glcolorconvert", "glcolorconvert0");
    auto gl_color_convert_1 = gst_element_factory_make("glcolorconvert", "glcolorconvert1");

    auto encoder_img0 = gst_element_factory_make("nvh265enc", "encoder_img0");
    auto encoder_img1 = gst_element_factory_make("nvh265enc", "encoder_img1");

    auto caps_str = "video/x-raw,format=RGB,width=" + std::to_string(display_params::width_pixels) +
        ",height=" + std::to_string(display_params::height_pixels) + ",framerate=0/1";
    auto caps_8uc1 = gst_caps_from_string(caps_str.c_str());
    g_object_set(G_OBJECT(_appsrc_img0), "caps", caps_8uc1, nullptr);
    g_object_set(G_OBJECT(_appsrc_img1), "caps", caps_8uc1, nullptr);
    gst_caps_unref(caps_8uc1);

    auto caps_convert_to_str = "video/x-raw,format=NV12,width=" + std::to_string(display_params::width_pixels) +
        ",height=" + std::to_string(display_params::height_pixels);
    auto caps_convert_to = gst_caps_from_string(caps_convert_to_str.c_str());
    g_object_set(G_OBJECT(caps_filter_0), "caps", caps_convert_to, nullptr);
    g_object_set(G_OBJECT(caps_filter_1), "caps", caps_convert_to, nullptr);
    gst_caps_unref(caps_convert_to);

    g_object_set(G_OBJECT(_appsrc_img0), "stream-type", 0, "format", GST_FORMAT_BYTES, "is-live", TRUE, nullptr);
    g_object_set(G_OBJECT(_appsrc_img1), "stream-type", 0, "format", GST_FORMAT_BYTES, "is-live", TRUE, nullptr);

    // g_signal_connect (_appsrc_img1, "need-data", G_CALLBACK (cb_need_data), this);

    g_object_set(_appsink_img0, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_object_set(_appsink_img1, "emit-signals", TRUE, "sync", FALSE, nullptr);

    g_signal_connect(_appsink_img0, "new-sample", G_CALLBACK(cb_new_sample), this);
    g_signal_connect(_appsink_img1, "new-sample", G_CALLBACK(cb_new_sample), this);

    _pipeline_img0 = gst_pipeline_new("pipeline_img0");
    _pipeline_img1 = gst_pipeline_new("pipeline_img1");

    gst_bin_add_many(GST_BIN(_pipeline_img0), _appsrc_img0, gl_color_convert_0, encoder_img0, caps_filter_0,
                     _appsink_img0, nullptr);
    gst_bin_add_many(GST_BIN(_pipeline_img1), _appsrc_img1, gl_color_convert_1, encoder_img1, caps_filter_1,
                     _appsink_img1, nullptr);

    // link elements
    if (!gst_element_link_many(_appsrc_img0, gl_color_convert_0, caps_filter_0, encoder_img0, _appsink_img0, nullptr) ||
        !gst_element_link_many(_appsrc_img1, gl_color_convert_1, caps_filter_1, encoder_img1, _appsink_img1, nullptr)) {
        abort("Failed to link elements");
    }

    gst_element_set_state(_pipeline_img0, GST_STATE_PLAYING);
    gst_element_set_state(_pipeline_img1, GST_STATE_PLAYING);
}

void video_encoder::enqueue(GLuint tex0, GLuint tex1) {
    GstVideoInfo info;
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_RGB, display_params::width_pixels, display_params::height_pixels);

    GstAllocator* allocator = GST_ALLOCATOR(gst_gl_memory_allocator_get_default(_context));

    // tex0
    GstGLVideoAllocationParams* params = gst_gl_video_allocation_params_new_wrapped_texture(
        _context, nullptr, &info, 0, nullptr, GST_GL_TEXTURE_TARGET_2D, GST_GL_RGB, tex0,
        new gl_destroy_notification_param{this, tex0}, (GDestroyNotify) gl_tex_free);
    GstGLMemory* gl_mem = GST_GL_MEMORY_CAST(
        gst_gl_base_memory_alloc(GST_GL_BASE_MEMORY_ALLOCATOR_CAST(allocator), (GstGLAllocationParams*) params));
    gst_gl_allocation_params_free((GstGLAllocationParams*) params);

    GstBuffer* buffer = gst_buffer_new();
    gst_buffer_append_memory(buffer, GST_MEMORY_CAST(gl_mem));
    GST_BUFFER_OFFSET(buffer) = _num_samples;

    auto ret_img0 = gst_app_src_push_buffer(reinterpret_cast<GstAppSrc*>(_appsrc_img0), buffer);
    gst_buffer_unref(buffer);

    // tex1
    params = gst_gl_video_allocation_params_new_wrapped_texture(_context, nullptr, &info, 0, nullptr, GST_GL_TEXTURE_TARGET_2D,
                                                                GST_GL_RGB, tex1, new gl_destroy_notification_param{this, tex1},
                                                                (GDestroyNotify) gl_tex_free);
    gl_mem = GST_GL_MEMORY_CAST(
        gst_gl_base_memory_alloc(GST_GL_BASE_MEMORY_ALLOCATOR_CAST(allocator), (GstGLAllocationParams*) params));
    gst_gl_allocation_params_free((GstGLAllocationParams*) params);

    buffer = gst_buffer_new();
    gst_buffer_append_memory(buffer, GST_MEMORY_CAST(gl_mem));
    GST_BUFFER_OFFSET(buffer) = _num_samples;

    auto ret_img1 = gst_app_src_push_buffer(reinterpret_cast<GstAppSrc*>(_appsrc_img1), buffer);

    gst_buffer_unref(buffer);
    gst_object_unref(allocator);

    if (ret_img0 != GST_FLOW_OK || ret_img1 != GST_FLOW_OK) {
        abort("Failed to push buffer");
    }
}

void video_encoder::init() {
    init_gl();
    create_pipelines();
}

GstFlowReturn video_encoder::cb_appsink(GstElement* sink) {
    // print thread id
    GstSample* sample;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample) {
        GstBuffer* buffer = gst_sample_get_buffer(sample);

        std::unique_lock<std::mutex> lock(_pipeline_sync_mutex); // lock acquired
        if (sink == _appsink_img0) {
            gst_buffer_map(buffer, &_img0_map, GST_MAP_READ);
            _img0_ready = true;
        } else {
            gst_buffer_map(buffer, &_img1_map, GST_MAP_READ);
            _img1_ready = true;
        }

        if (_img0_ready && _img1_ready) {
            _callback(_img0_map, _img1_map);
            _img0_ready = false;
            _img1_ready = false;
            lock.unlock(); // unlock and notify the waiting thread to clean up
            _pipeline_sync.notify_one();
        } else {
            _pipeline_sync.wait(lock, [&]() {
                return !_img0_ready && !_img1_ready;
            });            // wait unlocks the mutex if condition is not met
            lock.unlock(); // wait has acquired the lock. unlock it and start cleaning up
        }

        if (sink == _appsink_img0) {
            gst_buffer_unmap(buffer, &_img0_map);
        } else {
            gst_buffer_unmap(buffer, &_img1_map);
        }
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    return GST_FLOW_ERROR;
}

void video_encoder::gl_tex_free(video_encoder::gl_destroy_notification_param* param) {
    delete param;
}

} // namespace ILLIXR