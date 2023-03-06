//
// Created by steven on 7/8/22.
//

#ifndef PLUGIN_VIDEO_DECODER_H
#define PLUGIN_VIDEO_DECODER_H

#include "gst/gst.h"
// import gstglconfig.h
#include <gst/gl/gl.h>

#include <opencv2/core/mat.hpp>
#include <utility>
#include <queue>
#include <condition_variable>
#include "common/extended_window.hpp"


namespace ILLIXR {

    class video_decoder {
    public:
        struct gl_tex {
            GLuint tex;
            int width;
            int height;
        };
    private:
        struct gl_handle_tex_params {
            video_decoder *self;
            GstGLMemory *gl_mem;
            int id;
        };

        unsigned int _num_samples = 0;
        std::function<void(gl_tex&&, gl_tex&&)> _callback;
        GstElement *_pipeline_img0;
        GstElement *_pipeline_img1;
        GstElement *_appsrc_img0;
        GstElement *_appsrc_img1;
        GstElement *_appsink_img0;
        GstElement *_appsink_img1;

        std::condition_variable _pipeline_sync;
        std::mutex _pipeline_sync_mutex;
        bool _img0_ready = false;
        bool _img1_ready = false;
        GLuint _img0_tex;
        GLuint _img1_tex;

        void create_pipelines();

        static void gl_handle_tex(GstGLContext * context, gl_handle_tex_params *params);
    public:
        explicit video_decoder(std::function<void(gl_tex&&, gl_tex&&)> callback);

        void init();

        void enqueue(std::vector<char>& img0, std::vector<char>& img1);

        GstFlowReturn cb_appsink(GstElement *sink);
    };

} // ILLIXR

#endif //PLUGIN_VIDEO_DECODER_H
