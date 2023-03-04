//
// Created by steven on 7/3/22.
//

#ifndef ILLIXR_COMPRESSION_VIDEO_ENCODER_H
#define ILLIXR_COMPRESSION_VIDEO_ENCODER_H

#include "gst/gst.h"
// import gstglconfig.h
#include <gst/gl/gl.h>
#include <opencv2/core/mat.hpp>
#include <utility>
#include <queue>
#include <condition_variable>
#include "common/extended_window.hpp"

namespace ILLIXR {

    class video_encoder {
    private:
        struct gl_destroy_notification_param {
            video_encoder *self;
            GLuint tex;
        };

        std::function<void(const GstMapInfo &, const GstMapInfo &)> _callback;
        Display *_display;
        GstGLContext *_context;

        unsigned int _num_samples = 0;

        GstElement *_pipeline_img0;
        GstElement *_pipeline_img1;
        GstElement *_appsrc_img0;
        GstElement *_appsrc_img1;
        GstElement *_appsink_img0;
        GstElement *_appsink_img1;

        std::condition_variable _pipeline_sync;
        std::mutex _pipeline_sync_mutex;
        GstMapInfo _img0_map;
        GstMapInfo _img1_map;
        bool _img0_ready = false;
        bool _img1_ready = false;

        void init_gl();
        void create_pipelines();

        static void gl_tex_free(gl_destroy_notification_param* param);
    public:
        explicit video_encoder(std::function<void(const GstMapInfo&, const GstMapInfo&)> callback, Display *display);

        void init();

        void enqueue(GLuint tex0, GLuint tex1);

        GstFlowReturn cb_appsink(GstElement *sink);
    };

} // ILLIXR

#endif //ILLIXR_COMPRESSION_VIDEO_ENCODER_H
