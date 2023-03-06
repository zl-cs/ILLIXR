//
// Created by steven on 7/8/22.
//

#include "video_decoder.h"

#include "common/error_util.hpp"

#include <gst/app/gstappsrc.h>
#include <gst/gl/x11/gstgldisplay_x11.h>
#include <gst/video/video-info.h>
#include <utility>

namespace ILLIXR {

GstFlowReturn cb_new_sample(GstElement* appsink, gpointer* user_data) {
    return reinterpret_cast<video_decoder*>(user_data)->cb_appsink(appsink);
}

void video_decoder::create_pipelines() {
    gst_init(nullptr, nullptr);

    _appsrc_img0  = gst_element_factory_make("appsrc", "appsrc_img0");
    _appsrc_img1  = gst_element_factory_make("appsrc", "appsrc_img1");
    _appsink_img0 = gst_element_factory_make("appsink", "appsink_img0");
    _appsink_img1 = gst_element_factory_make("appsink", "appsink_img1");

    auto glcolorconvert_0 = gst_element_factory_make("glcolorconvert", "glcolorconvert0");
    auto glcolorconvert_1 = gst_element_factory_make("glcolorconvert", "glcolorconvert1");

    // auto h265parse_0 = gst_element_factory_make("h265parse", "h265parse0");
    // auto h265parse_1 = gst_element_factory_make("h265parse", "h265parse1");

    auto decoder_img0 = gst_element_factory_make("nvdec", "decoder_img0");
    auto decoder_img1 = gst_element_factory_make("nvdec", "decoder_img1");

    auto caps_filter_0 = gst_element_factory_make("capsfilter", "caps_filter_0");
    auto caps_filter_1 = gst_element_factory_make("capsfilter", "caps_filter_1");

    auto convert_to_caps_str = "video/x-raw,format=RGB,width=" + std::to_string(display_params::width_pixels) +
        ",height=" + std::to_string(display_params::height_pixels);
    g_object_set(G_OBJECT(caps_filter_0), "caps", gst_caps_from_string(convert_to_caps_str.c_str()), nullptr);
    g_object_set(G_OBJECT(caps_filter_1), "caps", gst_caps_from_string(convert_to_caps_str.c_str()), nullptr);

    // create caps with width and height
    auto caps_x265 = gst_caps_new_simple("video/x-h265", "stream-format", G_TYPE_STRING, "byte-stream", "alignment",
                                         G_TYPE_STRING, "au", nullptr);
    g_object_set(G_OBJECT(_appsrc_img0), "caps", caps_x265, nullptr);
    g_object_set(G_OBJECT(_appsrc_img1), "caps", caps_x265, nullptr);
    gst_caps_unref(caps_x265);

    g_object_set(G_OBJECT(_appsrc_img0), "stream-type", 0, "format", GST_FORMAT_BYTES, "is-live", TRUE, nullptr);
    g_object_set(G_OBJECT(_appsrc_img1), "stream-type", 0, "format", GST_FORMAT_BYTES, "is-live", TRUE, nullptr);

    g_object_set(G_OBJECT(decoder_img0), "low-latency-mode", TRUE, nullptr);
    g_object_set(G_OBJECT(decoder_img1), "low-latency-mode", TRUE, nullptr);

    g_object_set(_appsink_img0, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_object_set(_appsink_img1, "emit-signals", TRUE, "sync", FALSE, nullptr);

    g_signal_connect(_appsink_img0, "new-sample", G_CALLBACK(cb_new_sample), this);
    g_signal_connect(_appsink_img1, "new-sample", G_CALLBACK(cb_new_sample), this);

    _pipeline_img0 = gst_pipeline_new("pipeline_img0");
    _pipeline_img1 = gst_pipeline_new("pipeline_img1");

    // auto identity = gst_element_factory_make("identity", "identity");
    // g_object_set(G_OBJECT(identity), "dump", TRUE, nullptr);
    // g_object_set(G_OBJECT(identity), "signal-handoffs", TRUE, nullptr);

    gst_bin_add_many(GST_BIN(_pipeline_img0), _appsrc_img0, decoder_img0, glcolorconvert_0, caps_filter_0, _appsink_img0,
                     nullptr);
    gst_bin_add_many(GST_BIN(_pipeline_img1), _appsrc_img1, decoder_img1, glcolorconvert_1, caps_filter_1, _appsink_img1,
                     nullptr);

    // link elements
    if (!gst_element_link_many(_appsrc_img0, decoder_img0, glcolorconvert_0, caps_filter_0, _appsink_img0, nullptr) ||
        !gst_element_link_many(_appsrc_img1, decoder_img1, glcolorconvert_1, caps_filter_1, _appsink_img1, nullptr)) {
        abort("Failed to link elements");
    }

    gst_element_set_state(_pipeline_img0, GST_STATE_PLAYING);
    gst_element_set_state(_pipeline_img1, GST_STATE_PLAYING);
}

video_decoder::video_decoder(std::function<void(gl_tex&&, gl_tex&&)> callback)
    : _callback(std::move(callback)) { }

void video_decoder::init() {
    create_pipelines();
}

void video_decoder::enqueue(std::vector<char>& img0, std::vector<char>& img1) {
    auto buffer_img0 =
        gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, img0.data(), img0.size(), 0, img0.size(), nullptr, nullptr);
    auto buffer_img1 =
        gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, img1.data(), img1.size(), 0, img1.size(), nullptr, nullptr);

    GST_BUFFER_OFFSET(buffer_img0) = _num_samples;
    GST_BUFFER_OFFSET(buffer_img1) = _num_samples;

    _num_samples++;

    auto ret_img0 = gst_app_src_push_buffer(reinterpret_cast<GstAppSrc*>(_appsrc_img0), buffer_img0);
    auto ret_img1 = gst_app_src_push_buffer(reinterpret_cast<GstAppSrc*>(_appsrc_img1), buffer_img1);

    if (ret_img0 != GST_FLOW_OK || ret_img1 != GST_FLOW_OK) {
        abort("Failed to push buffer");
    }
}

GstFlowReturn video_decoder::cb_appsink(GstElement* sink) {
    std::cout << "cb_appsink" << std::endl;

    GstSample* sample;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (!sample) {
        return GST_FLOW_ERROR;
    }

    GstBuffer*   buffer = gst_sample_get_buffer(sample);
    GstMemory*   memory = gst_buffer_get_memory(buffer, 0);
    GstGLMemory* gl_mem = GST_GL_MEMORY_CAST(memory);

    auto params = new gl_handle_tex_params{this, gl_mem, sink == _appsink_img1};
    // blocking call - handler is guaranteed to finish running bef; see gstglcontext.c:1679
    gst_gl_context_thread_add(gl_mem->mem.context, (GstGLContextThreadFunc) gl_handle_tex, &params);
    gst_memory_unref(memory);
    gst_sample_unref(sample);
    delete params;
    return GST_FLOW_OK;
}

void video_decoder::gl_handle_tex(GstGLContext* context, video_decoder::gl_handle_tex_params* params) {
    auto                         self = params->self;
    std::unique_lock<std::mutex> lock(self->_pipeline_sync_mutex); // lock acquired

    int    width  = gst_gl_memory_get_texture_width(params->gl_mem);
    int    height = gst_gl_memory_get_texture_height(params->gl_mem);
    GLuint tex_id = gst_gl_memory_get_texture_id(params->gl_mem);

    if (params->id == 0) {
        self->_img0_ready = true;
        self->_img0_tex   = tex_id;
    } else {
        self->_img1_ready = true;
        self->_img1_tex   = tex_id;
    }

    if (self->_img0_ready && self->_img1_ready) {
        self->_callback(gl_tex { self->_img0_tex, width, height }, gl_tex { self->_img1_tex, width, height });
        self->_img0_ready = false;
        self->_img1_ready = false;
        lock.unlock(); // unlock and notify the waiting thread to clean up
        self->_pipeline_sync.notify_one();
    } else {
        self->_pipeline_sync.wait(lock, [&]() {
            return !self->_img0_ready && !self->_img1_ready; // we've reset the ready flags before notifying
        });
        lock.unlock(); // wait has acquired the lock. unlock it and start cleaning up
    }
}

} // namespace ILLIXR