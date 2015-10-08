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
#include <osvr/PluginKit/ImagingInterface.h>
#include <osvr/Util/ChannelCountC.h>
#include <osvr/PluginKit/DeviceInterface.h>
#include <osvr/Util/StringLiteralFileToString.h>

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

// based off of code from here: http://www.java2s.com/Open-Source/Android_Free_Code/WallPaper/screen/com_yaji_viewfinderImageUtil_java.htm
// which in turn was based off the android open source project. Both Apache licensed.
static void decodeYUV420SP(OSVR_ImageBufferElement* rgb, OSVR_ImageBufferElement* yuv420sp, int width, int height) {
  int frameSize = width * height;

  for (int j = 0, yp = 0; j < height; j++) {
    int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
    for (int i = 0; i < width; i++, yp++) {
      int y = (0xff & ((int) yuv420sp[yp])) - 16;
      if (y < 0) y = 0;
      if ((i & 1) == 0) {
        v = (0xff & yuv420sp[uvp++]) - 128;
        u = (0xff & yuv420sp[uvp++]) - 128;
      }

      int y1192 = 1192 * y;
      int r = (y1192 + 1634 * v);
      int g = (y1192 - 833 * v - 400 * u);
      int b = (y1192 + 2066 * u);

      if (r < 0) r = 0; else if (r > 262143) r = 262143;
      if (g < 0) g = 0; else if (g > 262143) g = 262143;
      if (b < 0) b = 0; else if (b > 262143) b = 262143;

      rgb[yp * 3] = r;
      rgb[yp * 3 + 1] = g;
      rgb[yp * 3 + 2] = b;
      //rgb[yp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
    }
  }
}

// Anonymous namespace to avoid symbol collision
namespace {

    class AndroidJniImagingDevice {
    public:
        AndroidJniImagingDevice(OSVR_PluginRegContext ctx, int cameraNum = 0, int channel = 0)
          : m_camera(cameraNum), m_channel(channel), m_reportNumber(0) {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null
            LOGI("[OSVR] AndroidJniImagingDevice instantiated");
            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Configure an imaging interface (with the default number of sensors,
            /// 1)
            m_imaging = osvr::pluginkit::ImagingInterface(opts);

            /// Come up with a device name
            std::ostringstream os;
            os << "Camera" << cameraNum << "_" << m_channel;

            /// Create an asynchronous (threaded) device
            m_dev.initAsync(ctx, os.str(), opts);
            // Puts an object in m_dev that knows it's a
            // threaded device so osvrDeviceSendData knows
            // that it needs to get a connection lock first.

            /// Send the JSON.
            m_dev.sendJsonDescriptor(
                osvr::util::makeString(com_osvr_android_jniImaging_json));

            /// Sets the update callback
            m_dev.registerUpdateCallback(this);
        }

        OSVR_ReturnCode update() {
          //LOGI("[OSVR] AndroidJniImagingDevice::update # %d called", m_reportNumber++);

          // Get a timestamp for the upcoming camera grab.
          osvr::util::time::TimeValue frameTime = osvr::util::time::getNow();

          // Send the image.
          // Note that if larger than 160x120 (RGB), will used shared memory
          // backend only.

          if(gLastFrame) {
              LOGI("[OSVR] image report # %d called", m_reportNumber++);
              OSVR_ImageBufferElement* buffer
                  = new OSVR_ImageBufferElement[gLastFrameMetadata.width * gLastFrameMetadata.height * gLastFrameMetadata.channels];

              decodeYUV420SP(buffer, gLastFrame, gLastFrameMetadata.width, gLastFrameMetadata.height);

              cv::Mat frame(gLastFrameMetadata.width, gLastFrameMetadata.height, CV_8UC3, buffer);

              m_dev.send(m_imaging, osvr::pluginkit::ImagingMessage(frame), frameTime);

              // The interface takes ownership of the image data. Also, we don't
              // want to send the image more than once.
              delete[] buffer;
              gLastFrame = NULL;
          }

          return OSVR_RETURN_SUCCESS;
        }

    private:

      osvr::pluginkit::DeviceToken m_dev;
      osvr::pluginkit::ImagingInterface m_imaging;
      int m_camera;
      int m_channel;
      OSVR_ImageBufferElement * m_frame;
      int m_reportNumber;
    };

    class HardwareDetection {
    public:
        HardwareDetection() : m_found(false) {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
            if (m_found) {
                return OSVR_RETURN_SUCCESS;
            }

            LOGI("[OSVR] Android Jni Imaging plugin: Got a hardware detection request");

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
