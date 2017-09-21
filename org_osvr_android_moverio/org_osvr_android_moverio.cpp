/** @file
    @brief Android sensor based tracker plugin, currently supporting orientation only.

    @date 2017

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
    */

// Copyright 2017 Sensics, Inc.
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
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>

// Generated JSON header file
#include "org_osvr_android_moverio_json.h"

// Library/third-party includes
#include <android/sensor.h>
#include <android/log.h>
#include <android/keycodes.h>

// Standard includes
#include <iostream>
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>
#include <queue>
#include <utility>

#define  LOG_TAG    "org_osvr_android_moverio"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

#include <jni.h>

extern "C" {
    JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportKeyDown(
        JNIEnv * env, jclass clazz, jint keyCode);
    JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportKeyUp(
        JNIEnv * env, jclass clazz, jint keyCode);
    JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportMouse(
        JNIEnv * env, jclass clazz, jfloat x, jfloat y, 
        jfloat xMin, jfloat xMax, jfloat yMin, jfloat yMax);
}

namespace {
    static std::mutex sInputMutex;
    typedef struct AndroidButtonStateWithKeyCode {
        int keyCode;
        OSVR_ButtonState state; 
        OSVR_TimeValue timestamp;
    } AndroidButtonStateWithKeyCode;

    static std::queue<AndroidButtonStateWithKeyCode> sButtonStateQueueSynced;

    void pushButtonState(int keyCode, OSVR_ButtonState state) {
        std::lock_guard<std::mutex> lock(sInputMutex);
        OSVR_TimeValue now;
        osvrTimeValueGetNow(&now);
        AndroidButtonStateWithKeyCode buttonState = {0};
        buttonState.keyCode = keyCode;
        buttonState.state = state;
        buttonState.timestamp = now;
        sButtonStateQueueSynced.push(buttonState);
    }

    static std::mutex sMouseMutex;

    typedef struct AndroidMouseState {
        OSVR_AnalogState xState;
        OSVR_AnalogState yState;
        OSVR_AnalogState xMinState;
        OSVR_AnalogState xMaxState;
        OSVR_AnalogState yMinState;
        OSVR_AnalogState yMaxState;
        OSVR_TimeValue timestamp;
    } AndroidMouseState;

    static std::queue<AndroidMouseState> sMouseStateQueueSynced;

    void pushMouseState(OSVR_AnalogState xState, OSVR_AnalogState yState,
        OSVR_AnalogState xMinState, OSVR_AnalogState xMaxState,
        OSVR_AnalogState yMinState, OSVR_AnalogState yMaxState) {
        std::lock_guard<std::mutex> lock(sMouseMutex);
        OSVR_TimeValue now;
        osvrTimeValueGetNow(&now);
        AndroidMouseState mouseState = {0};
        mouseState.xState = xState;
        mouseState.yState = yState;
        mouseState.xMinState = xMinState;
        mouseState.xMaxState = xMaxState;
        mouseState.yMinState = yMinState;
        mouseState.yMaxState = yMaxState;
        mouseState.timestamp = now;
        sMouseStateQueueSynced.push(mouseState);
    }
}

JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportKeyDown(JNIEnv * env, jclass clazz, jint keyCode) {
    pushButtonState(keyCode, OSVR_BUTTON_PRESSED);
}

JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportKeyUp(JNIEnv * env, jclass clazz, jint keyCode) {
    pushButtonState(keyCode, OSVR_BUTTON_NOT_PRESSED);
}

JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportMouse(JNIEnv * env, jclass clazz, jfloat x, jfloat y, 
    jfloat xMin, jfloat xMax, jfloat yMin, jfloat yMax) {
    pushMouseState(x, y, xMin, xMax, yMin, yMax);
}

// Anonymous namespace to avoid symbol collision
namespace {
    // Android standard sensor types @todo are these defined in a header somewhere?
    static const int TYPE_GAME_ROTATION_VECTOR = 15;
    static const int TYPE_ROTATION_VECTOR = 11;

    // Moverio specific sensor types
    static const int TYPE_HEADSET_TAP = 0x00002001;
    static const int TYPE_CONTROLLER_ROTATION_VECTOR = 0x0010000b;

    // OSVR tracker channels
    enum {
        MOVERIO_TRACKER_CHANNEL_HEAD = 0,
        MOVERIO_TRACKER_CHANNEL_CONTROLLER = 1,

        MOVERIO_NUM_TRACKER_CHANNELS = 2
    };

    // OSVR button channels
    enum {
        ANDROID_BUTTON_CHANNEL_DPAD_CENTER = 0,
        ANDROID_BUTTON_CHANNEL_DPAD_DOWN = 1,
        ANDROID_BUTTON_CHANNEL_DPAD_RIGHT = 2,
        ANDROID_BUTTON_CHANNEL_DPAD_LEFT = 3,
        ANDROID_BUTTON_CHANNEL_DPAD_UP = 4,
        ANDROID_BUTTON_CHANNEL_VOLUME_UP = 5,
        ANDROID_BUTTON_CHANNEL_VOLUME_DOWN = 6,
        ANDROID_BUTTON_CHANNEL_BACK = 7,
        
        MOVERIO_BUTTON_CHANNEL_HEADSET_TAP = 8,

        MOVERIO_NUM_BUTTON_CHANNELS = 9
    };

    enum {
        ANDROID_ANALOG_CHANNEL_MOUSE_X = 0,
        ANDROID_ANALOG_CHANNEL_MOUSE_Y,
        
        ANDROID_NUM_ANALOG_CHANNELS,
    };

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

    class ASensorThread {
    private:
        std::thread mThread;

        // These variables, asside from mMutex, are synchronized,
        // so use a std::lock_guard on mMutex to read/write
        std::mutex mMutex;
        bool mQuitSignalSynced = false;
        bool mDeviceActiveSynced = false;
        std::queue<std::pair<OSVR_OrientationState, OSVR_TimeValue> > mHeadStateQueueSynced;
        std::queue<std::pair<OSVR_OrientationState, OSVR_TimeValue> > mBaseStationStateQueueSynced;
        std::queue<std::pair<OSVR_ButtonState, OSVR_TimeValue> > mHeadTapStateQueueSynced;
        ASensorManager* mSensorManagerSynced = nullptr;
        ASensorEventQueue* mSensorEventQueueSynced = nullptr;

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

        void run() {
            LOGI("[org_osvr_android_moverio]: Got a hardware detection request");
            // Get the ALooper for the current thread
            ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
            if (NULL == looper) {
                LOGE("[org_osvr_android_moverio]: There is no ALooper instance for the current thread. Can't get sensor data without one.");
                return;
            }

            // lock_guard block
            {
                std::lock_guard<std::mutex> lock(mMutex);
                // The sensor manager is a singleton
                mSensorManagerSynced = ASensorManager_getInstance();
                if (NULL == mSensorManagerSynced) {
                    LOGE("[org_osvr_android_moverio]: Couldn't get the ASensorManager for this thread.");
                    return;
                }

                // get the default Accelerometer sensor and enable it
                const ASensor* headSensor = ASensorManager_getDefaultSensor(mSensorManagerSynced, TYPE_GAME_ROTATION_VECTOR);
                if(NULL == headSensor) {
                    LOGI("[org_osvr_android_moverio]: Couldn't get the TYPE_GAME_ROTATION_VECTOR, trying for TYPE_ROTATION_VECTOR");
                    headSensor = ASensorManager_getDefaultSensor(mSensorManagerSynced, TYPE_ROTATION_VECTOR);
                }

                if (NULL == headSensor) {
                    LOGE("[org_osvr_android_moverio]: Couldn't get the default ASensor instance for TYPE_GAME_ROTATION_VECTOR");
                    return;
                }

                // get the controller sensor and enable it
                const ASensor* controllerSensor = ASensorManager_getDefaultSensor(mSensorManagerSynced, TYPE_CONTROLLER_ROTATION_VECTOR);
                if (NULL == controllerSensor) {
                    LOGE("[org_osvr_android_moverio]: Couldn't get the default ASensor instance for TYPE_CONTROLLER_ROTATION_VECTOR");
                    return;
                }

                // tap sensor
                const ASensor* tapSensor = ASensorManager_getDefaultSensor(mSensorManagerSynced, TYPE_HEADSET_TAP);
                if (NULL == controllerSensor) {
                    LOGE("[org_osvr_android_moverio]: Couldn't get the default ASensor instance for TYPE_HEADSET_TAP");
                    return;
                }

                // Create a default event queue
                mSensorEventQueueSynced = ASensorManager_createEventQueue(mSensorManagerSynced, looper, 3 /*LOOPER_ID_USER*/, NULL, NULL);
                if (NULL == mSensorEventQueueSynced) {
                    LOGE("[org_osvr_android_moverio]: Couldn't create a sensor event queue.");
                    return;
                }

                if(!enableSensorAndSetEventRate(headSensor, mSensorEventQueueSynced)) {
                    LOGE("[org_osvr_android_moverio]: Failure while enabling the head sensor and setting its event rate");
                    return;
                }

                if(!enableSensorAndSetEventRate(controllerSensor, mSensorEventQueueSynced)) {
                    LOGE("[org_osvr_android_moverio]: Failure while enabling the controller sensor and setting its event rate");
                    return;
                }

                // if(!enableSensorAndSetEventRate(tapSensor, mSensorEventQueueSynced)) {
                //     LOGE("[org_osvr_android_moverio]: Failure while enabling the headset tap sensor and settings its event rate");
                //     return;
                // }
            } // end lock_guard(mMutex) block

            while(true) {
                std::lock_guard<std::mutex> lock(mMutex);

                // assume active if we get there
                mDeviceActiveSynced = true;
                if(mQuitSignalSynced) {
                    return;
                }
                ASensorEvent e;
                while (ASensorEventQueue_getEvents(mSensorEventQueueSynced, &e, 1) > 0)
                {
                    OSVR_TimeValue nowTimeValue;
                    osvrTimeValueGetNow(&nowTimeValue);

                    switch(e.type) {
                        case TYPE_GAME_ROTATION_VECTOR:
                        case TYPE_ROTATION_VECTOR:
                        case TYPE_CONTROLLER_ROTATION_VECTOR:
                        {
                            OSVR_OrientationState orientation;
                            osvrQuatSetIdentity(&orientation);
    
                            if(ASensorEventToOSVR_OrientationState(&e, orientation)) {
                                switch(e.type) {
                                    case TYPE_GAME_ROTATION_VECTOR:
                                    case TYPE_ROTATION_VECTOR:
                                        mHeadStateQueueSynced.push(std::make_pair(orientation, nowTimeValue));
                                        break;
                                    case TYPE_CONTROLLER_ROTATION_VECTOR:
                                        mBaseStationStateQueueSynced.push(std::make_pair(orientation, nowTimeValue));
                                        break;
                                    default:
                                        LOGE("[org_osvr_android_moverio]: Unknown ASensorEvent type supported by ASensorEventToOSVR_OrientationState.");
										break;
                                }
                            }
                        } break;
                        case TYPE_HEADSET_TAP:
                        {
                            float value = e.data[0];
                            LOGI("[org_osvr_android_moverio]: TYPE_HEADSET_TAP, e.data[0] == %f", value);
                            OSVR_ButtonState buttonState =
                                (value != 0.0f ? OSVR_BUTTON_PRESSED : OSVR_BUTTON_NOT_PRESSED);
                            
                            mHeadTapStateQueueSynced.push(std::make_pair(buttonState, nowTimeValue));
                        } break;
                        default:
                            // not an error and doesn't need to be logged.
                            break;
                    }
                }
            }
        }

    public:
        ASensorThread()
        : mThread(&ASensorThread::run, this) {

        }

        ~ASensorThread() {
            waitForThreadComplete();
            // no need for lock_guard, thread is finished
            if(ASensorManager_destroyEventQueue(mSensorManagerSynced, mSensorEventQueueSynced) < 0) {
                LOGI("[org_osvr_android_moverio]: Could not destroy sensor event queue.");
              }
        }

        void signalStop() {
            std::lock_guard<std::mutex> lock(mMutex);
			mQuitSignalSynced = true;
        }

        void waitForThreadComplete() {
            signalStop();
            mThread.join();
        }

        bool isDeviceActive() {
            std::lock_guard<std::mutex> lock(mMutex);
            return mDeviceActiveSynced;
        }

        OSVR_ReturnCode reportQueuedState(
            osvr::pluginkit::DeviceToken dev,
            OSVR_TrackerDeviceInterface tracker,
            OSVR_ButtonDeviceInterface button) {

            std::lock_guard<std::mutex> lock(mMutex);

            // Tracker interface

            // Head
            while(!mHeadStateQueueSynced.empty()) {
                auto trackerReport = mHeadStateQueueSynced.front();
                mHeadStateQueueSynced.pop();
                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceTrackerSendOrientationTimestamped(
                        dev, tracker, &trackerReport.first, MOVERIO_TRACKER_CHANNEL_HEAD, &trackerReport.second)) {
                    LOGE("[org_osvr_android_moverio]: Failed to send tracker orientation.");
                    return OSVR_RETURN_FAILURE;
                }
            }

            // Base station
            while(!mHeadStateQueueSynced.empty()) {
                auto trackerReport = mHeadStateQueueSynced.front();
                mHeadStateQueueSynced.pop();
                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceTrackerSendOrientationTimestamped(
                        dev, tracker, &trackerReport.first, MOVERIO_TRACKER_CHANNEL_CONTROLLER, &trackerReport.second)) {
                    LOGE("[org_osvr_android_moverio]: Failed to send tracker orientation.");
                    return OSVR_RETURN_FAILURE;
                }
            }

            // Button interface

            // Tap button
            while(!mHeadTapStateQueueSynced.empty()) {
                auto buttonReport = mHeadTapStateQueueSynced.front();
                mHeadTapStateQueueSynced.pop();
                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceButtonSetValueTimestamped(
                            dev, button, buttonReport.first, MOVERIO_BUTTON_CHANNEL_HEADSET_TAP, &buttonReport.second)) {
                    LOGE("[org_osvr_android_moverio]: Failed to send button state.");
                    return OSVR_RETURN_FAILURE;
                }
            }

            return OSVR_RETURN_SUCCESS;
        }
    };

    class MoverioTrackerDevice {
    private:
        osvr::pluginkit::DeviceToken m_dev;
        OSVR_TrackerDeviceInterface m_tracker;
        OSVR_ButtonDeviceInterface m_button;
        OSVR_AnalogDeviceInterface m_analog;
        std::shared_ptr<ASensorThread> m_sensorThread;

    public:
        MoverioTrackerDevice(OSVR_PluginRegContext ctx, std::shared_ptr<ASensorThread> sensorThread)
            : m_sensorThread(sensorThread)
        {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Configure the tracker interfaces
            osvrDeviceTrackerConfigure(opts, &m_tracker);

            /// Configure the button interfaces
            osvrDeviceButtonConfigure(opts, &m_button, MOVERIO_NUM_BUTTON_CHANNELS);

            /// Configure the analog interfaces
            osvrDeviceAnalogConfigure(opts, &m_analog, ANDROID_NUM_ANALOG_CHANNELS);

            /// Create the sync device token with the options
            m_dev.initSync(ctx, "MoverioTracker", opts);

            /// Send JSON descriptor
            m_dev.sendJsonDescriptor(org_osvr_android_moverio_json);

            /// Register update callback
            m_dev.registerUpdateCallback(this);
        }

        ~MoverioTrackerDevice() {
            
        }

        OSVR_ReturnCode reportQueuedAndroidButtonState(osvr::pluginkit::DeviceToken dev, OSVR_ButtonDeviceInterface button) {
            std::lock_guard<std::mutex> lock(sInputMutex);
            while(!sButtonStateQueueSynced.empty()) {
                auto buttonReport = sButtonStateQueueSynced.front();
                sButtonStateQueueSynced.pop();

                OSVR_ChannelCount buttonChannel = -1;
                switch(buttonReport.keyCode) {
                    case AKEYCODE_DPAD_UP:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_DPAD_UP;
                        break;
                    case AKEYCODE_DPAD_DOWN:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_DPAD_DOWN;
                        break;
                    case AKEYCODE_DPAD_LEFT:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_DPAD_LEFT;
                        break;
                    case AKEYCODE_DPAD_RIGHT:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_DPAD_RIGHT;
                        break;
                    case AKEYCODE_DPAD_CENTER:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_DPAD_CENTER;
                        break;
                    case AKEYCODE_VOLUME_UP:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_VOLUME_UP;
                        break;
                    case AKEYCODE_VOLUME_DOWN:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_VOLUME_DOWN;
                        break;
                    case AKEYCODE_BACK:
                        buttonChannel = ANDROID_BUTTON_CHANNEL_BACK;
                        break;
                    default:
                        continue;
                }
                if(buttonChannel >= 0) {
                    if(OSVR_RETURN_SUCCESS !=
                        osvrDeviceButtonSetValueTimestamped(
                                dev, button, buttonReport.state, buttonChannel, &buttonReport.timestamp)) {
                        LOGE("[org_osvr_android_moverio]: Failed to send button state.");
                        return OSVR_RETURN_FAILURE;
                    }
                }
            }
            return OSVR_RETURN_SUCCESS;
        }

        OSVR_ReturnCode reportQueuedAndroidMouseState(osvr::pluginkit::DeviceToken dev, OSVR_AnalogDeviceInterface analog) {
            std::lock_guard<std::mutex> lock(sMouseMutex);
            while(!sMouseStateQueueSynced.empty()) {
                auto mouseReport = sMouseStateQueueSynced.front();
                sMouseStateQueueSynced.pop();

                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceAnalogSetValueTimestamped(
                            dev, analog, mouseReport.xState, ANDROID_ANALOG_CHANNEL_MOUSE_X, &mouseReport.timestamp)) {
                    LOGE("[org_osvr_android_moverio]: Failed to send analog state.");
                    return OSVR_RETURN_FAILURE;
                }

                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceAnalogSetValueTimestamped(
                            dev, analog, mouseReport.yState, ANDROID_ANALOG_CHANNEL_MOUSE_Y, &mouseReport.timestamp)) {
                    LOGE("[org_osvr_android_moverio]: Failed to send analog state.");
                    return OSVR_RETURN_FAILURE;
                }

            }
            return OSVR_RETURN_SUCCESS;
        }

        OSVR_ReturnCode update() {
            OSVR_ReturnCode buttonRet = reportQueuedAndroidButtonState(m_dev, m_button);
            OSVR_ReturnCode sensorRet = m_sensorThread->reportQueuedState(m_dev, m_tracker, m_button);
            OSVR_ReturnCode mouseRet = reportQueuedAndroidMouseState(m_dev, m_analog);

            bool ret = buttonRet == OSVR_RETURN_SUCCESS &&
                sensorRet == OSVR_RETURN_SUCCESS &&
                mouseRet == OSVR_RETURN_SUCCESS;
            return ret ? OSVR_RETURN_SUCCESS : OSVR_RETURN_FAILURE;
        }
    };

    class HardwareDetection {
    public:
        HardwareDetection()  : mSensorThreadPtr(new ASensorThread()) { }
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

            if(!mDeviceAdded) {
                if(!mSensorThreadPtr->isDeviceActive()) {
                    LOGI("[org_osvr_android_moverio]: sensor thread reported no device enabled.");
                    return OSVR_RETURN_FAILURE;
                }

                LOGI("[org_osvr_android_moverio]: sensor tracker plugin enabled!");

                /// Create our device object
                osvr::pluginkit::registerObjectForDeletion(ctx,
                    new MoverioTrackerDevice(ctx, mSensorThreadPtr));
                
                // don't add the same device twice or more
                mDeviceAdded = true;
            }
            return OSVR_RETURN_SUCCESS;
        }

    private:
        bool mDeviceAdded = false;
        std::shared_ptr<ASensorThread> mSensorThreadPtr;
    };
} // namespace

OSVR_PLUGIN(org_osvr_android_moverio) {
    LOGI("[org_osvr_android_moverio]: Entry point executed!");
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
