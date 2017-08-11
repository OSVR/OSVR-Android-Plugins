/** @file
    @brief Android sensor based tracker plugin, currently supporting orientation only.

    @date 2017

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
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "org_osvr_android_moverio_json.h"

// Library/third-party includes
#include <android/sensor.h>
#include <android/log.h>

// Standard includes
#include <iostream>
#include <cmath>

#define  LOG_TAG    "org_osvr_android_moverio"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

// Anonymous namespace to avoid symbol collision
namespace {
    static const int TYPE_GAME_ROTATION_VECTOR = 15;
    static const int TYPE_ROTATION_VECTOR = 11;

    // Moverio specific sensor types
    static const int TYPE_HEADSET_TAP = 0x00002001;
    static const int TYPE_CONTROLLER_ROTATION_VECTOR = 0x0010000b;

    // OSVR tracker channels
    static const int MOVERIO_TRACKER_CHANNEL_HEAD = 0;
    static const int MOVERIO_TRACKER_CHANNEL_CONTROLLER = 1;

    static bool ASensorEventToOSVR_OrientationState(const ASensorEvent* e, OSVR_OrientationState& orientationOut) {
        if(!e) {
            LOGE("[org_osvr_android_moverio]: ASensorEventToOSVR_OrientationState - Expecting a non-NULL ASensorEvent.");
            return false;
        }
        if (e->type == TYPE_GAME_ROTATION_VECTOR ||
            e->type == TYPE_ROTATION_VECTOR ||
            e->type == TYPE_CONTROLLER_ROTATION_VECTOR) {
            // if things look weird (like swapping x and y), it's because we're transforming
            // the rotation vector from the world coordinate system to OSVR.
            
            //int64_t timestamp = e.timestamp;
            // float x2 = -e.data[1];
            // float y2 = e.data[0];
            // float z2 = e.data[2];
            // float w2 = e.data[3];

            float x2 = e->data[0];
            float y2 = e->data[1];
            float z2 = e->data[2];
            float w2 = e->data[3];

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
            
            // @todo we may need to renormalize here
            float fw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
            float fx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
            float fy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
            float fz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

            osvrQuatSetIdentity(&orientationOut);

            osvrQuatSetW(&orientationOut, fw);
            osvrQuatSetX(&orientationOut, fx);
            osvrQuatSetY(&orientationOut, fy);
            osvrQuatSetZ(&orientationOut, fz);

            // osvrQuatSetW(&orientationOut, w2);
            // osvrQuatSetX(&orientationOut, x2);
            // osvrQuatSetY(&orientationOut, y2);
            // osvrQuatSetZ(&orientationOut, z2);
            return true;
        }
        return false;
    };

    class MoverioTrackerDevice {
    private:
        osvr::pluginkit::DeviceToken m_dev;
        OSVR_TrackerDeviceInterface m_tracker;
        ALooper* m_looper;
        ASensorManager* m_sensorManager;
        ASensorEventQueue* m_sensorEventQueue;

    public:
        MoverioTrackerDevice(OSVR_PluginRegContext ctx,
            ALooper *looper, ASensorManager *sensorManager, ASensorEventQueue *sensorEventQueue)
            : m_looper(looper), m_sensorManager(sensorManager), m_sensorEventQueue(sensorEventQueue)
        {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Indicate that we'll want 1 analog channel.
            osvrDeviceTrackerConfigure(opts, &m_tracker);

            /// Create the sync device token with the options
            m_dev.initSync(ctx, "MoverioTracker", opts);

            /// Send JSON descriptor
            m_dev.sendJsonDescriptor(org_osvr_android_moverio_json);

            /// Register update callback
            m_dev.registerUpdateCallback(this);
        }

        ~MoverioTrackerDevice() {
            if(ASensorManager_destroyEventQueue(m_sensorManager, m_sensorEventQueue) < 0) {
              LOGI("[org_osvr_android_moverio]: Could not destroy sensor event queue.");
            }
        }

        OSVR_ReturnCode update() {
            ASensorEvent e;
            while (ASensorEventQueue_getEvents(m_sensorEventQueue, &e, 1) > 0)
            {
                OSVR_OrientationState orientation;
                osvrQuatSetIdentity(&orientation);

                if(ASensorEventToOSVR_OrientationState(&e, orientation)) {
                    OSVR_ChannelCount trackerChannel = -1;
                    switch(e.type) {
                        case TYPE_GAME_ROTATION_VECTOR:
                        case TYPE_ROTATION_VECTOR:
                            trackerChannel = MOVERIO_TRACKER_CHANNEL_HEAD;
                            break;
                        case TYPE_CONTROLLER_ROTATION_VECTOR:
                            trackerChannel = MOVERIO_TRACKER_CHANNEL_CONTROLLER;
                            break;
                        default:
                            LOGE("Unknown ASensorEvent type supported by ASensorEventToOSVR_OrientationState.");
                            return OSVR_RETURN_FAILURE;
                    }
                    // @todo look into whether we can convert/use the timestamp
                    // from the sensor event. For now, just let osvr use the
                    // current time.
                    osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &orientation, trackerChannel);
                }
            }
            return OSVR_RETURN_SUCCESS;
        }
    };

    // Enable a sensor and set its event rate, ensuring event rate is set to fastest available
    static bool enableSensorAndSetEventRate(const ASensor* sensor, ASensorEventQueue* sensorEventQueue) {
        if (ASensorEventQueue_enableSensor(sensorEventQueue, sensor) < 0) {
            LOGE("[org_osvr_android_moverio]: Couldn't enable the game rotation vector sensor.");
            return false;
        }

        // it's an error to set the desired event rate to something less than
        // the minimum sensor delay.
        int minSensorDelay = ASensor_getMinDelay(sensor);
        if(minSensorDelay == 0) {
            LOGI("[org_osvr_android_moverio]: Sensor reports continuously. Setting event rate to 100Hz");
            // the sensor reports continuously, not at a fixed rate
            // so just set the event rate to 100Hz
            minSensorDelay = 100000;
        }

        LOGI("[org_osvr_android_moverio]: Setting sensor event rate to %d", minSensorDelay);

        // desired event rate
        if (ASensorEventQueue_setEventRate(sensorEventQueue, sensor, minSensorDelay) < 0) {
            LOGI("[org_osvr_android_moverio]: Couldn't set the event rate.");
            // this probably isn't fatal. We'll just let it send us events as often as it wants
        }
        return true;
    }

    class HardwareDetection {
    public:
        HardwareDetection() {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

            LOGI("[org_osvr_android_moverio]: Got a hardware detection request");
            // Get the ALooper for the current thread
            ALooper* looper = ALooper_prepare(0);
            if (NULL == looper) {
                LOGE("[org_osvr_android_moverio]: There is no ALooper instance for the current thread. Can't get sensor data without one.");
                return OSVR_RETURN_FAILURE;
            }

            // The sensor manager is a singleton
            ASensorManager* sensorManager = ASensorManager_getInstance();
            if (NULL == sensorManager) {
                LOGE("[org_osvr_android_moverio]: Couldn't get the ASensorManager for this thread.");
                return OSVR_RETURN_FAILURE;
            }

            // get the default Accelerometer sensor and enable it
            const ASensor* headSensor = ASensorManager_getDefaultSensor(sensorManager, TYPE_GAME_ROTATION_VECTOR);
            if(NULL == headSensor) {
                LOGI("[org_osvr_android_moverio]: Couldn't get the TYPE_GAME_ROTATION_VECTOR, trying for TYPE_ROTATION_VECTOR");
                headSensor = ASensorManager_getDefaultSensor(sensorManager, TYPE_ROTATION_VECTOR);
            }

            if (NULL == headSensor) {
                LOGE("[org_osvr_android_moverio]: Couldn't get the default ASensor instance for TYPE_GAME_ROTATION_VECTOR");
                return OSVR_RETURN_FAILURE;
            }

            // get the controller sensor and enable it
            const ASensor* controllerSensor = ASensorManager_getDefaultSensor(sensorManager, TYPE_CONTROLLER_ROTATION_VECTOR);
            if (NULL == controllerSensor) {
                LOGE("[org_osvr_android_moverio]: Couldn't get the default ASensor instance for TYPE_CONTROLLER_ROTATION_VECTOR");
                return OSVR_RETURN_FAILURE;
            }

            // Create a default event queue
            ASensorEventQueue *sensorEventQueue = ASensorManager_createEventQueue(sensorManager, looper, 3 /*LOOPER_ID_USER*/, NULL, NULL);
            if (NULL == sensorEventQueue) {
                LOGE("[org_osvr_android_moverio]: Couldn't create a sensor event queue.");
                return OSVR_RETURN_FAILURE;
            }

            if(!enableSensorAndSetEventRate(headSensor, sensorEventQueue)) {
                LOGE("[org_osvr_android_moverio]: Failure while enabling the head sensor and setting its event rate");
                return OSVR_RETURN_FAILURE;
            }

            if(!enableSensorAndSetEventRate(controllerSensor, sensorEventQueue)) {
                LOGE("[org_osvr_android_moverio]: Failure while enabling the controller sensor and setting its event rate");
                return OSVR_RETURN_FAILURE;
            }

            LOGI("[org_osvr_android_moverio]: sensor tracker plugin enabled!");

            /// Create our device object
            osvr::pluginkit::registerObjectForDeletion(ctx,
                new MoverioTrackerDevice(ctx, looper, sensorManager, sensorEventQueue));

            return OSVR_RETURN_SUCCESS;
        }

    private:

    };
} // namespace

OSVR_PLUGIN(org_osvr_android_moverio) {
    LOGI("[org_osvr_android_moverio]: Entry point executed!");
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
