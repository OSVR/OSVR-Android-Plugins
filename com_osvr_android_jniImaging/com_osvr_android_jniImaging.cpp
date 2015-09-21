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

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/ImagingInterfaceC.h>
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

OSVR_ImageBufferElement *gLastFrame = NULL;
OSVR_ImagingMetadata gLastFrameMetadata;

extern "C" {
    JNIEXPORT void JNICALL Java_com_osvr_android_JniBridge_reportFrame(JNIEnv * env, jclass clazz,
      jbyteArray data, jlong width, jlong height, jshort channels, jshort depth);
}

JNIEXPORT void JNICALL Java_com_osvr_android_JniBridge_reportFrame(JNIEnv * env, jclass clazz,
    jbyteArray data, jlong width, jlong height, jshort channels, jshort depth) {

    gLastFrameMetadata.height = (OSVR_ImageDimension)height;
    gLastFrameMetadata.width = (OSVR_ImageDimension)width;
    gLastFrameMetadata.channels = (OSVR_ImageChannels)channels;
    gLastFrameMetadata.depth = (OSVR_ImageDepth)depth;

    // @todo determine whether the current metadata matches the last metadata,
    // and if so, reuse the last frame buffer instead of deleting and recreating.
    // better yet, use a ring buffer so that image reports aren't lost if update
    // isn't called frequently enough.
    if(gLastFrame) {
        // this image will be lost. the plugin's update method wasn't called
        // fast enough.
        delete[] gLastFrame;
    }
    int size = env->GetArrayLength(data);
    gLastFrame = new OSVR_ImageBufferElement[size];
    env->GetByteArrayRegion(data, 0, size, reinterpret_cast<jbyte*>(gLastFrame));
}

namespace osvr {
namespace pluginkit {
    /** @defgroup PluginKitCppImaging Imaging interface (C++)
        @brief Sending image reports from a device in your plugin.
        @ingroup PluginKit

        @{
    */

    /// @brief A class wrapping a OSVR_ImageBufferElement* representing a frame, as well as the
    /// sensor ID it corresponds to. Pass to
    /// osvr::pluginkit::DeviceToken::send() along with your
    /// osvr::pluginkit::RawImagingInterface.
    class RawImagingMessage {
      public:
        /// @brief Constructor, optionally taking a sensor number. Does NOT clone the
        /// supplied image/data, but does take ownership of the pointer passed.
        RawImagingMessage(OSVR_ImageBufferElement *frame, OSVR_ImagingMetadata frameMetadata, OSVR_ChannelCount sensor = 0)
            : m_frame(frame), m_frameMetadata(frameMetadata), m_sensor(sensor) {
        }

        /// @brief Retrieves a reference to the internal (cloned) data.
        OSVR_ImageBufferElement *getFrame() const { return m_frame; }

        /// @brief Retrieves the frame metadata.
        OSVR_ImagingMetadata getFrameMetadata() const { return m_frameMetadata; }

        /// @brief Gets the sensor number.
        OSVR_ChannelCount getSensor() const { return m_sensor; }

      private:
        OSVR_ImageBufferElement *m_frame;
        OSVR_ImagingMetadata m_frameMetadata;
        OSVR_ChannelCount m_sensor;
    };

    /// @brief A class wrapping an imaging interface for a device.
    class RawImagingInterface {
      public:
        /// @brief Default constructor or constructor from an existing
        /// OSVR_ImagingDeviceInterface (assumed to be registered with an
        /// OSVR_DeviceInitOptions already)
        RawImagingInterface(OSVR_ImagingDeviceInterface iface = NULL)
            : m_iface(iface) {}

        /// @brief Constructor that registers an imaging interface with the
        /// device init options object.
        explicit RawImagingInterface(OSVR_DeviceInitOptions opts,
                                  OSVR_ChannelCount numSensors = 1) {
            OSVR_ReturnCode ret =
                osvrDeviceImagingConfigure(opts, &m_iface, numSensors);
            if (OSVR_RETURN_SUCCESS != ret) {
                throw std::logic_error("Could not initialize an Imaging "
                                       "Interface with the device options "
                                       "given!");
            }
        }

        /// @brief Send method - usually called by
        /// osvr::pluginkit::DeviceToken::send()
        void send(DeviceToken &dev, const RawImagingMessage &message,
                  OSVR_TimeValue const &timestamp) {
            if (!m_iface) {
                throw std::logic_error(
                    "Must initialize the imaging interface before using it!");
            }
            OSVR_ImagingMetadata metadata = message.getFrameMetadata();

            // metadata.channels = frame.channels();
            // metadata.depth = typedata.getSize();
            // metadata.width = frame.cols;
            // metadata.height = frame.rows;
            // metadata.type = typedata.isFloatingPoint()
            //                     ? OSVR_IVT_FLOATING_POINT
            //                     : (typedata.isSigned() ? OSVR_IVT_SIGNED_INT
            //                                            : OSVR_IVT_UNSIGNED_INT);

            OSVR_ReturnCode ret =
                osvrDeviceImagingReportFrame(dev, m_iface,
                    message.getFrameMetadata(), message.getFrame(), message.getSensor(),
                    &timestamp);

            if (OSVR_RETURN_SUCCESS != ret) {
                throw std::runtime_error("Could not send imaging message!");
            }
        }

      private:
        OSVR_ImagingDeviceInterface m_iface;
    };
    /// @}
} // namespace pluginkit
} // namespace osvr

// Anonymous namespace to avoid symbol collision
namespace {

    class AndroidJniImagingDevice {
    public:
        AndroidJniImagingDevice(OSVR_PluginRegContext ctx, int cameraNum = 0, int channel = 0)
          : m_camera(cameraNum), m_channel(channel) {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Configure an imaging interface (with the default number of sensors,
            /// 1)
            m_imaging = osvr::pluginkit::RawImagingInterface(opts);

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

          // Get a timestamp for the upcoming camera grab.
          osvr::util::time::TimeValue frameTime = osvr::util::time::getNow();

          // Send the image.
          // Note that if larger than 160x120 (RGB), will used shared memory
          // backend only.
          if(gLastFrame) {
              m_dev.send(m_imaging, osvr::pluginkit::RawImagingMessage(gLastFrame, gLastFrameMetadata),
                  frameTime);

              // The interface takes ownership of the image data. Also, we don't
              // want to send the image more than once.
              gLastFrame = NULL;
          }

          return OSVR_RETURN_SUCCESS;
        }

    private:

      osvr::pluginkit::DeviceToken m_dev;
      osvr::pluginkit::RawImagingInterface m_imaging;
      int m_camera;
      int m_channel;
      OSVR_ImageBufferElement * m_frame;
    };

    class HardwareDetection {
    public:
        HardwareDetection() : m_found(false) {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
            if (m_found) {
                return OSVR_RETURN_SUCCESS;
            }

            std::cout << "[OSVR] Android Jni Imaging plugin: Got a hardware detection request" << std::endl;

            /// Create our device object
            osvr::pluginkit::registerObjectForDeletion(ctx,
                new AndroidJniImagingDevice(ctx));

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
