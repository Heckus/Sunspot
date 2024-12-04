#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>
#include <glib-object.h>

class IMX219Processor {
private:
    GstElement *pipeline;
    GstElement *sink;

    cv::Mat processImage(const cv::Mat& rawImage) {
        cv::Mat rgbImage, denoised, balanced, gammaImage, sharpened;

        // Demosaicing
        cv::demosaicing(rawImage, rgbImage, cv::COLOR_BayerRG2RGB);

        // Noise Reduction
        cv::fastNlMeansDenoisingColored(rgbImage, denoised);

        // White Balance
        cv::normalize(denoised, balanced, 0, 255, cv::NORM_MINMAX);

        // Gamma Correction
        double gamma = 2.2;
        cv::Mat lookupTable(1, 256, CV_8U);
        uchar* p = lookupTable.ptr();
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
        }
        cv::LUT(balanced, lookupTable, gammaImage);

        // Sharpening
        cv::detailEnhance(gammaImage, sharpened);

        return sharpened;
    }

    static GstFlowReturn onNewSample(GstElement* sink, gpointer data) {
        IMX219Processor* processor = static_cast<IMX219Processor*>(data);

        // Retrieve sample from appsink
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) return GST_FLOW_ERROR;

        // Get buffer from sample
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        // Convert to OpenCV Mat (assumes raw Bayer format)
        cv::Mat rawImage(480, 640, CV_8UC1, map.data);

        // Process image
        cv::Mat processedImage = processor->processImage(rawImage);

        // Do something with processed image (display, save, etc.)
        cv::imshow("IMX219 Stream", processedImage);
        cv::waitKey(1);

        // Cleanup
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }

public:
    IMX219Processor() {
        gst_init(NULL, NULL);

        // GStreamer pipeline for IMX219 via libcamerarc
        pipeline = gst_parse_launch(
            "libcamerarc src ! video/x-raw,format=GRAY8,width=640,height=480 ! appsink name=sink emit-signals=true", NULL);

        sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

        // Set callbacks
        g_signal_connect(sink, "new-sample", G_CALLBACK(onNewSample), this);
        gst_app_sink_set_emit_signals(GST_APP_SINK(sink), true);
    }

    void start() {
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void stop() {
        gst_element_set_state(pipeline, GST_STATE_NULL);
    }

    ~IMX219Processor() {
        gst_object_unref(sink);
        gst_object_unref(pipeline);
    }
};

int main() {
    IMX219Processor processor;
    processor.start();

    // Keep main thread running
    pause();

    return 0;
}
