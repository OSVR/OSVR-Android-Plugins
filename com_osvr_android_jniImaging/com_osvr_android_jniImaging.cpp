/** @file
    @brief Android sensor based tracker plugin, currently supporting orientation only.

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
    */

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <jni.h>
#include <android/log.h>

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/DeviceInterfaceC.h>
#include <osvr/PluginKit/ImagingInterfaceC.h>
#include <osvr/Util/ChannelCountC.h>
#include <osvr/Util/TimeValueC.h>
#include <osvr/Util/StringLiteralFileToString.h>
//#include <opencv2/imgproc/imgproc.hpp>

// Generated JSON header file
#include "com_osvr_android_jniImaging_json.h"

// Library/third-party includes
// - none

// Standard includes
#include <iostream>
#include <sstream>
#include <cstring>

#define  LOG_TAG    "libgl2jni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern OSVR_ImageBufferElement *gLastFrame;
extern OSVR_ImagingMetadata gLastFrameMetadata;


// Based on algorithm by andre chen
// https://github.com/andrechen/yuv2rgb

static void writePixel(unsigned char* &d, int r, int g, int b) {
	*d++ = (r > 0) ? (r < 65535 ? (r >> 8) : 0xff) : 0; // r
    *d++ = (g > 0) ? (g < 65535 ? (g >> 8) : 0xff) : 0; // g
    *d++ = (b > 0) ? (b < 65535 ? (b >> 8) : 0xff) : 0; // b
	*d++ = 255; // alpha
}

static void NV21_to_RGBA(OSVR_ImageBufferElement *rgba, OSVR_ImageBufferElement * nv21, int width, int height) {
    OSVR_ImageBufferElement *d0 = rgba;
    OSVR_ImageBufferElement *y0 = nv21;
    OSVR_ImageBufferElement *uv = nv21 + (width * height);

    int const halfHeight = height >> 1;
    int const halfWidth = width >> 1;

    for(int h = 0; h < halfHeight; ++h) {
        OSVR_ImageBufferElement *y1 = y0 + width;
        OSVR_ImageBufferElement *d1 = d0 + width * 4;
        for(int w = 0; w < halfWidth; ++w) {
            int y00 = (*y0++) - 16;
            int y01 = (*y0++) - 16;
            int y10 = (*y1++) - 16;
            int y11 = (*y1++) - 16;

            int v = (*uv++) - 128;
            int u = (*uv++) - 128;

            y00 = (y00 > 0) ? (298 * y00) : 0;
            y01 = (y01 > 0) ? (298 * y01) : 0;
            y10 = (y10 > 0) ? (298 * y10) : 0;
            y11 = (y11 > 0) ? (298 * y11) : 0;

            int r = 128 + 409 * v;
            int g = 128 - 100 * u - 208 * v;
            int b = 128 + 516 * u;

            writePixel(d0, y00 + r, y00 + g, y00 + b);
            writePixel(d0, y01 + r, y01 + g, y01 + b);
            writePixel(d1, y10 + r, y10 + g, y10 + b);
            writePixel(d1, y11 + r, y11 + g, y11 + b);
        }
        y0 = y1;
        d0 = d1;
    }
}

// Anonymous namespace to avoid symbol collision
namespace {

    class AndroidJniImagingDevice {
    private:
        OSVR_DeviceToken m_dev;
        OSVR_ImagingDeviceInterface m_imaging;
        OSVR_ImageBufferElement * m_frame;
        int m_camera;
        int m_channel;
        int m_reportNumber;
    public:
        AndroidJniImagingDevice(OSVR_PluginRegContext ctx, int cameraNum = 0, int channel = 0)
          : m_camera(cameraNum), m_channel(channel), m_reportNumber(0) {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null
            LOGI("[com_osvr_android_jniImaging] AndroidJniImagingDevice instantiated");
            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Configure an imaging interface (with the default number of sensors,
            /// 1)
            //m_imaging = osvr::pluginkit::ImagingInterface(opts);
            if(OSVR_RETURN_FAILURE == osvrDeviceImagingConfigure(opts, &m_imaging, 1)) {
                LOGE("[com_osvr_android_jniImaging] Could not configure imaging interface.");
            }

            /// Come up with a device name
            std::ostringstream os;
            os << "Camera" << cameraNum << "_" << m_channel;

            /// Create an asynchronous (threaded) device
            // Puts an object in m_dev that knows it's a
            // threaded device so osvrDeviceSendData knows
            // that it needs to get a connection lock first.
            //m_dev.initAsync(ctx, os.str(), opts);
            if(OSVR_RETURN_FAILURE == osvrDeviceAsyncInitWithOptions(ctx, os.str().c_str(), opts, &m_dev)) {
                LOGE("[com_osvr_android_jniImaging] osvrDeviceAsyncInitWithOptions failed.");
            }

            /// Send the JSON.
            // m_dev.sendJsonDescriptor(
            //     osvr::util::makeString(com_osvr_android_jniImaging_json));
            std::string jsonDescriptorStr = osvr::util::makeString(com_osvr_android_jniImaging_json);
            if(OSVR_RETURN_FAILURE == osvrDeviceSendJsonDescriptor(m_dev, jsonDescriptorStr.c_str(), jsonDescriptorStr.size())) {
                LOGE("[com_osvr_android_jniImaging] osvrDeviceSendJsonDescriptor call failed.");
            }

            /// Sets the update callback
            //m_dev.registerUpdateCallback(this);
            if(OSVR_RETURN_FAILURE == osvrDeviceRegisterUpdateCallback(m_dev, update, this)) {
                LOGE("[com_osvr_android_jniImaging] osvrDeviceRegisterUpdateCallback failed.");
            }
        }

        static OSVR_ReturnCode update(void* userData) {
            AndroidJniImagingDevice* imagingDevice = (AndroidJniImagingDevice*)userData;

            //LOGI("[com_osvr_android_jniImaging] AndroidJniImagingDevice::update # %d called", imagingDevice->m_reportNumber++);

            // Get a timestamp for the upcoming camera grab.
            //osvr::util::time::TimeValue frameTime = osvr::util::time::getNow();
            OSVR_TimeValue nowTimeValue;
            osvrTimeValueGetNow(&nowTimeValue);

            // Send the image.
            // Note that if larger than 160x120 (RGB), will used shared memory
            // backend only.

            if(gLastFrame) {
                //LOGI("[com_osvr_android_jniImaging] image report # %d", imagingDevice->m_reportNumber++);
                gLastFrameMetadata.channels = 4;
                gLastFrameMetadata.depth = 1;
                unsigned int width = gLastFrameMetadata.width;
                unsigned int height = gLastFrameMetadata.height;
                unsigned int channels = gLastFrameMetadata.channels;
                OSVR_ImageBufferElement* buffer
                    = new OSVR_ImageBufferElement[width * height * channels];

                NV21_to_RGBA(buffer, gLastFrame, width, height);
                //cv::Mat frame(height, width, CV_8UC4, buffer);

                //m_dev.send(m_imaging, osvr::pluginkit::ImagingMessage(frame), frameTime);

                if(OSVR_RETURN_FAILURE ==
                    osvrDeviceImagingReportFrame(
                        imagingDevice->m_dev, imagingDevice->m_imaging, 
                        gLastFrameMetadata, buffer, imagingDevice->m_channel, &nowTimeValue)) {
                        LOGE("[com_osvr_android_jniImaging] osvrDeviceImagingReportFrame failed.");
                        
                }

                // The interface takes ownership of the image data. Also, we don't
                // want to send the image more than once.
                delete[] buffer;
                gLastFrame = NULL;
            }

            return OSVR_RETURN_SUCCESS;
        }
    };

    class HardwareDetection {
    public:
        HardwareDetection() : m_found(false) {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
            if (m_found) {
                return OSVR_RETURN_SUCCESS;
            }

            LOGI("[com_osvr_android_jniImaging] Android Jni Imaging plugin: Got a hardware detection request");

            /// Create our device object
            osvr::pluginkit::registerObjectForDeletion(ctx,
                new AndroidJniImagingDevice(ctx));

            m_found = true;
            return OSVR_RETURN_SUCCESS;
        }

    private:
        bool m_found;
    };
} // namespace

OSVR_PLUGIN(com_osvr_android_jniImaging) {
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
