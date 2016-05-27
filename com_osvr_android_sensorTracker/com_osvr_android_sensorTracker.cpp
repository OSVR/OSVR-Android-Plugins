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

// Android includes
#include <android/sensor.h>
#include <cmath>

//#include <android_native_app_glue.h>

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_osvr_android_sensorTracker_json.h"

// Library/third-party includes
// - none

// Standard includes
#include <iostream>

// Anonymous namespace to avoid symbol collision
namespace {
    static const int TYPE_GAME_ROTATION_VECTOR = 15;
    static const int TYPE_ROTATION_VECTOR = 11;

    class AndroidSensorTrackerDevice {
    public:
        AndroidSensorTrackerDevice(OSVR_PluginRegContext ctx,
            ALooper *looper, ASensorManager *sensorManager, const ASensor *sensor, ASensorEventQueue *sensorEventQueue)
            : m_looper(looper), m_sensorManager(sensorManager), m_sensor(sensor), m_sensorEventQueue(sensorEventQueue)
        {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Indicate that we'll want 1 analog channel.
            osvrDeviceTrackerConfigure(opts, &m_tracker);

            /// Create the sync device token with the options
            m_dev.initSync(ctx, "AndroidSensorTracker", opts);

            /// Send JSON descriptor
            m_dev.sendJsonDescriptor(com_osvr_android_sensorTracker_json);

            /// Register update callback
            m_dev.registerUpdateCallback(this);
        }

        ~AndroidSensorTrackerDevice() {
            if(ASensorManager_destroyEventQueue(m_sensorManager, m_sensorEventQueue) < 0) {
              std::cout << "[OSVR]: Android sensor tracker - Could not destroy sensor event queue.";
            }
        }

        OSVR_ReturnCode update() {
            ASensorEvent e;
            while (ASensorEventQueue_getEvents(m_sensorEventQueue, &e, 1) > 0)
            {
                if (e.type == TYPE_GAME_ROTATION_VECTOR || e.type == TYPE_ROTATION_VECTOR) {
                    // if things look weird (like swapping x and y), it's because we're transforming
                    // the rotation vector from the world coordinate system to OSVR.
                    
                    //int64_t timestamp = e.timestamp;
                    float x2 = -e.data[1];
                    float y2 = e.data[0];
                    float z2 = e.data[2];
                    float w2 = e.data[3];
                    // Added in SDK Level 18. Might use it for custom filtering?
                    // float estimatedHeadingAccuracy = e.data[4]; // in radians

                    // originally optional prior to SDK Level 18
                    if (w2 < 0.0f) {
                        w2 = std::sqrt(1.0f - (x2 * x2 + y2 * y2 + z2 * z2));
                    }
                    
                    // By default, (w2, x2, y2, z2) points straight down when you're looking ahead,
                    // so rotate it up (negative) by 90 degrees (M_PI_2)
                    float sinRThetaOver2 = std::sin(-M_PI_4);
                    float x1 = sinRThetaOver2;
                    float y1 = 0.0f;
                    float z1 = 0.0f;
                    float w1 = std::cos(M_PI_4);
                    
                    float fw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
                    float fx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
                    float fy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
                    float fz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

                    // @todo we may need to renormalize here

                    OSVR_OrientationState orientation;
                    osvrQuatSetIdentity(&orientation);
                    osvrQuatSetW(&orientation, fw);
                    osvrQuatSetX(&orientation, fx);
                    osvrQuatSetY(&orientation, fy);
                    osvrQuatSetZ(&orientation, fz);

                    // @todo look into whether we can convert/use the timestamp
                    // from the sensor event. For now, just let osvr use the
                    // current time.
                    osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &orientation, 0);
                }
            }
            return OSVR_RETURN_SUCCESS;
        }

    private:
        osvr::pluginkit::DeviceToken m_dev;
        OSVR_TrackerDeviceInterface m_tracker;
        ALooper* m_looper;
        ASensorManager* m_sensorManager;
        const ASensor* m_sensor;
        ASensorEventQueue* m_sensorEventQueue;
    };

    class HardwareDetection {
    public:
        HardwareDetection() {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

            std::cout << "[OSVR] Android plugin: Got a hardware detection request" << std::endl;
            // Get the ALooper for the current thread
            ALooper* looper = ALooper_prepare(0);
            if (NULL == looper) {
                std::cout << "[OSVR] Android plugin: There is no ALooper instance for the current thread. Can't get sensor data without one."
                    << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            // The sensor manager is a singleton
            ASensorManager* sensorManager = ASensorManager_getInstance();
            if (NULL == sensorManager) {
                std::cout << "[OSVR] Android plugin: Couldn't get the ASensorManager for this thread." << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            // get the default Accelerometer sensor and enable it
            const ASensor* sensor = ASensorManager_getDefaultSensor(sensorManager, TYPE_GAME_ROTATION_VECTOR);
            if (NULL == sensor) {
                std::cout << "[OSVR] Android plugin: Couldn't get the default ASensor instance for TYPE_GAME_ROTATION_VECTOR" << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            // Create a default event queue
            ASensorEventQueue *sensorEventQueue = ASensorManager_createEventQueue(sensorManager, looper, 3 /*LOOPER_ID_USER*/, NULL, NULL);
            if (NULL == sensorEventQueue) {
                std::cout << "[OSVR] Android plugin: Couldn't create a sensor event queue." << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            if (ASensorEventQueue_enableSensor(sensorEventQueue, sensor) < 0) {
                std::cout << "[OSVR] Android plugin: Couldn't enable the game rotation vector sensor." << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            // it's an error to set the desired event rate to something less than
            // the minimum sensor delay.
            int minSensorDelay = ASensor_getMinDelay(sensor);
            if(minSensorDelay == 0) {
              // the sensor reports continuously, not at a fixed rate
              // so just set the event rate to 100Hz
              minSensorDelay = 100000;
            }

            // desired event rate
            if (ASensorEventQueue_setEventRate(sensorEventQueue, sensor, minSensorDelay) < 0) {
                std::cout << "[OSVR] Android plugin: Couldn't set the event rate." << std::endl;
                // this probably isn't fatal. We'll just let it send us events as often as it wants
            }

            std::cout << "[OSVR] Android plugin: sensor tracker plugin enabled!" << std::endl;

            /// Create our device object
            osvr::pluginkit::registerObjectForDeletion(ctx,
                new AndroidSensorTrackerDevice(ctx, looper, sensorManager, sensor, sensorEventQueue));

            return OSVR_RETURN_SUCCESS;
        }

    private:

    };
} // namespace

OSVR_PLUGIN(com_osvr_android_sensorTracker) {
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
