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
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/Util/QuaternionC.h>
#include <osvr/Util/Vec3C.h>
#include <osvr/Util/EigenCoreGeometry.h>
#include <osvr/Util/EigenInterop.h>
#include <osvr/Util/EigenQuatExponentialMap.h>
#include <osvr/Common/Endianness.h>
#include "vrpn_FixedPoint.h"

// Generated JSON header file
#include "org_osvr_android_hdk2jni_json.h"

// Library/third-party includes
#include <android/log.h>
#include <jni.h>


// Standard includes
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <queue>
#include <cstdint>

#define  LOG_TAG    "hdk2jni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern "C" {
    JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportHeadPose(
        JNIEnv * env, jclass clazz, 
        jfloat x, jfloat y, jfloat z, jfloat w, 
        jfloat px, jfloat py, jfloat pz, 
        jfloat dx, jfloat dy, jfloat dz, jfloat dw, jfloat dt);
}

namespace {
    
    static const uint16_t vrpn_HDK_STATUS_STRIDE = 400;
    static const double vrpn_HDK_DT = 1.0 / 50;
    
    enum OSVR_HDK2JNI_TRACKER_CHANNELS {
        OSVR_HDK2JNI_HEAD_TRACKER_CHANNEL = 0,
        
        OSVR_HDK2JNI_NUM_TRACKER_CHANNELS
    };
    
    enum OSVR_HDK2JNI_ANALOG_CHANNELS {
        OSVR_HDK2JNI_VERSION_ANALOG_CHANNEL = 0,
        OSVR_HDK2JNI_VIDEO_STATUS_ANALOG_CHANNEL = 1,
        
        OSVR_HDK2JNI_NUM_ANALOG_CHANNELS
    };
    
    enum OSVR_HDK2JNI_VIDEO_STATUS {
        STATUS_UNKNOWN = 0,
        STATUS_NO_VIDEO_INPUT = 1,
        STATUS_PORTRAIT_VIDEO_INPUT = 2,
        STATUS_LANDSCAPE_VIDEO_INPUT = 3
    };
    
    static std::mutex sStateQueueMutex;
    
    typedef struct HeadPoseState {
        OSVR_PoseState state;
        OSVR_IncrementalQuaternion angularVelocity;
        bool hasAngularVelocity;
        OSVR_TimeValue timestamp;
    } HeadPoseState;
    static std::queue<HeadPoseState> sHeadPoseStateQueueSynced;

    
    void pushHeadState(OSVR_PoseState state, OSVR_IncrementalQuaternion angularVelocity, bool hasAngularVelocity) {
        std::lock_guard<std::mutex> lock(sStateQueueMutex);
        OSVR_TimeValue now;
        osvrTimeValueGetNow(&now);
        HeadPoseState headPoseState = {0};
        headPoseState.state = state;
        headPoseState.angularVelocity = angularVelocity;
        headPoseState.hasAngularVelocity = hasAngularVelocity;
        headPoseState.timestamp = now;
        sHeadPoseStateQueueSynced.push(headPoseState);
    }
    
    void pushHeadState(OSVR_Quaternion orientation, OSVR_IncrementalQuaternion angularVelocity, bool hasAngularVelocity) {

        OSVR_PoseState state;
        osvrPose3SetIdentity(&state);
        state.rotation = orientation;
        pushHeadState(state, angularVelocity, hasAngularVelocity);
    }

    // analog state is always reported together
    typedef struct HDK2AnalogState {
        OSVR_AnalogState version;
        OSVR_AnalogState videoStatus;
        OSVR_TimeValue timestamp;
    } HDK2AnalogState;
    static std::queue<HDK2AnalogState> sHDK2AnalogStateQueueSynced;

    void pushHDK2AnalogState(OSVR_AnalogState version, OSVR_AnalogState videoStatus) {
        std::lock_guard<std::mutex> lock(sStateQueueMutex);
        HDK2AnalogState state = {0};
        osvrTimeValueGetNow(&state.timestamp);
        state.version = version;
        state.videoStatus = videoStatus;
    }

    /// DUPLICATE CODE - @TODO should be moved to osvrUtil
    /// use only for derivatives - has factor of 2/0.5 in it!
    inline Eigen::Quaterniond
    angVelVecToIncRot(Eigen::Vector3d const &angVelVec, double dt) {
        return osvr::util::quat_exp(angVelVec * dt * 0.5).normalized();
    }

    /// DUPLICATE CODE - @TODO should be moved to osvrUtil
    /// Turn a body-space angular velocity vector into a room-space incremental
    /// rotation quaternion.
    ///
    /// Orientation is assumed to be normalized. dt must be chosen such that the
    /// velocity will not alias.
    template <typename Derived>
    inline Eigen::Quaterniond
    angVelVecToQuat(Eigen::Quaterniond const &orientation, double dt,
                    Eigen::MatrixBase<Derived> const &vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

        Eigen::Quaterniond result =
            orientation * angVelVecToIncRot(vec, dt) * orientation.conjugate();
        return result;
    }

    /// Function template to unbuffer values from a buffer stored in little-
    /// endian byte order. Specify the type to extract T as a template parameter.
    /// The templated buffer type ByteT will be deduced automatically.
    /// The input pointer will be advanced past the unbuffered value.
    template <typename T>
    static inline T vrpn_unbuffer_from_little_endian(uint8_t *&input)
    {
        using namespace osvr::common::serialization;

        /// Union to allow type-punning
        union {
            uint8_t bytes[sizeof(T)];
            T typed;
        } value;

        /// Swap known little-endian into big-endian (aka network byte order)
        for (unsigned int i = 0, j = sizeof(T) - 1; i < sizeof(T); ++i, --j) {
            value.bytes[i] = input[j];
        }

        /// Advance input pointer
        input += sizeof(T);

        /// return value in host byte order
        return ntoh(value.typed);
    }

    class HDK2DataReceiver {
    private:

        /// Timestamp updated during mainloop()
        OSVR_TimeValue _timestamp;
        
        /// Flag indicating whether we were connected last time through the
        /// mainloop.
        /// Used to send a "normal"-severity message when we connect with info on
        /// the device and to handle re-connecting after a USB disconnect.
        bool _wasConnected = false;

        /// Used to forcibly send the analog update every so often
        uint16_t _messageCount = 0;

        uint8_t _reportVersion = 0;
        bool _knownVersion = true;

    public:

        HDK2DataReceiver() {
            osvrTimeValueGetNow(&_timestamp);
        }

        void osvr_hdk2_on_data_received(std::size_t bytes, uint8_t *buffer)
        {
            if (bytes != 32 && bytes != 16)
            {
                // send_text_message(vrpn_TEXT_WARNING)
                //     << "Received a report " << bytes
                //     << " in length, but expected it to be 32 or 16 bytes. Discarding. "
                //        "(May indicate issues with HID!)";
                return;
            }

            uint8_t firstByte = vrpn_unbuffer_from_little_endian<uint8_t>(buffer);

            uint8_t version = uint8_t(0x0f) & firstByte;
            _reportVersion = version;

            switch (version)
            {
            case 1:
                if (bytes != 32 && bytes != 16)
                {
                    // send_text_message(vrpn_TEXT_WARNING)
                    //     << "Received a v1 report " << bytes
                    //     << " in length, but expected it to be 32 or 16 bytes. "
                    //        "Discarding. "
                    //        "(May indicate issues with HID!)";
                    return;
                }
                break;
            case 2:
                if (bytes != 16)
                {
                    // send_text_message(vrpn_TEXT_WARNING)
                    //     << "Received a v2 report " << bytes
                    //     << " in length, but expected it to be 16 bytes. Discarding. "
                    //        "(May indicate issues with HID!)";
                    return;
                }
                break;

            case 3:
                /// @todo once this report format is finalized, tighten up the
                /// requirements.
                if (bytes < 16)
                {
                    // send_text_message(vrpn_TEXT_WARNING)
                    //     << "Received a v3 report " << bytes
                    //     << " in length, but expected it to be at least 16 bytes. "
                    //        "Discarding. "
                    //        "(May indicate issues with HID!)";
                    return;
                }
                break;
            default:
                /// Highlight that we don't know this report version well...
                _knownVersion = false;
                /// Do a minimal check of it.
                if (bytes < 16)
                {
                    // send_text_message(vrpn_TEXT_WARNING)
                    //     << "Received a report claiming to be version " << int(version)
                    //     << " that was " << bytes << " in length, but expected it to be "
                    //                                 "at least 16 bytes. Discarding. "
                    //                                 "(May indicate issues with HID!)";
                    return;
                }
                break;
            }

            // Report version as analog channel 0.
            //channel[0] = version;

            uint8_t msg_seq = vrpn_unbuffer_from_little_endian<uint8_t>(buffer);

            OSVR_Quaternion headOrientation;
            OSVR_IncrementalQuaternion headAngularVelocity;
            bool hasAngularVelocity = false;

            osvrQuatSetIdentity(&headOrientation);
            osvrQuatSetIdentity(&headAngularVelocity.incrementalRotation);
            headAngularVelocity.dt = vrpn_HDK_DT;

            // Signed, 16-bit, fixed-point numbers in Q1.14 format.
            typedef vrpn::FixedPoint<1, 14> FixedPointValue;
            Eigen::Quaterniond d_quat;
            d_quat.x() =
                FixedPointValue(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                    .get<double>();
            d_quat.y() =
                FixedPointValue(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                    .get<double>();
            d_quat.z() =
                FixedPointValue(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                    .get<double>();
            d_quat.w() =
                FixedPointValue(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                    .get<double>();

            osvr::util::toQuat(d_quat, headOrientation);

            if (version >= 2)
            {
                hasAngularVelocity = true;
                // We've got angular velocity in this message too
                // Given XYZ radians per second velocity.
                // Signed Q6.9
                typedef vrpn::FixedPoint<6, 9> VelFixedPoint;
                Eigen::Vector3d angVel;
                angVel.x() =
                    VelFixedPoint(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                        .get<double>();
                angVel.y() =
                    VelFixedPoint(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                        .get<double>();
                angVel.z() =
                    VelFixedPoint(vrpn_unbuffer_from_little_endian<int16_t>(buffer))
                        .get<double>();

                //==================================================================
                // Determine the rotational velocity, which is
                // measured in the rotated coordinate system.  We need to rotate the
                // difference Euler angles back to the canonical orientation, apply
                // the change, and then rotate back to change our coordinates.
                // Be sure to scale by the time value vrpn_HDK_DT.
                Eigen::Quaterniond vel_quat = angVelVecToQuat(d_quat, vrpn_HDK_DT, angVel);
                osvr::util::toQuat(vel_quat, headAngularVelocity.incrementalRotation);
            }

            pushHeadState(headOrientation, headAngularVelocity, hasAngularVelocity);

            double versionAnalog = static_cast<double>(version);
            double videoStatusAnalog = 0.0;

            if (version < 3)
            {
                // No status info hidden in the first byte.
                videoStatusAnalog = STATUS_UNKNOWN;
            }
            else
            {
                // v3+: We've got status info in the upper nibble of the first byte.
                bool gotVideo = (firstByte & (0x01 << 4)) != 0;    // got video?
                bool gotPortrait = (firstByte & (0x01 << 5)) != 0; // portrait mode?
                if (!gotVideo)
                {
                    videoStatusAnalog = STATUS_NO_VIDEO_INPUT;
                }
                else
                {
                    if (gotPortrait)
                    {
                        videoStatusAnalog = STATUS_PORTRAIT_VIDEO_INPUT;
                    }
                    else
                    {
                        videoStatusAnalog = STATUS_LANDSCAPE_VIDEO_INPUT;
                    }
                }
            }

            // report analog state every update
            // @TODO: throttle this analog value?
            pushHDK2AnalogState(versionAnalog, videoStatusAnalog);

            // if (_messageCount == 0)
            // {
            //     // When _messageCount overflows, send a report whether or not there was
            //     // a change.
            //     vrpn_Analog::report();
            // }
            // else
            // {
            //     // otherwise just report if we have a change.
            //     vrpn_Analog::report_changes();
            // };
            _messageCount = (_messageCount + 1) % vrpn_HDK_STATUS_STRIDE;
        }
    };
}

JNIEXPORT void JNICALL Java_com_osvr_common_jni_JNIBridge_reportHeadOrientation(
    JNIEnv * env, jclass clazz, 
    jfloat x, jfloat y, jfloat z, jfloat w, 
    jfloat px, jfloat py, jfloat pz, 
    jfloat dx, jfloat dy, jfloat dz, jfloat dw, jfloat dt)
{
    OSVR_PoseState state;
    OSVR_IncrementalQuaternion angularVelocity;

    osvrQuatSetX(&state.rotation, x);
    osvrQuatSetY(&state.rotation, y);
    osvrQuatSetZ(&state.rotation, z);
    osvrQuatSetW(&state.rotation, w);

    osvrVec3SetX(&state.translation, px);
    osvrVec3SetY(&state.translation, py);
    osvrVec3SetZ(&state.translation, pz);

    angularVelocity.dt = dt;
    osvrQuatSetX(&angularVelocity.incrementalRotation, dx);
    osvrQuatSetY(&angularVelocity.incrementalRotation, dy);
    osvrQuatSetZ(&angularVelocity.incrementalRotation, dz);
    osvrQuatSetW(&angularVelocity.incrementalRotation, dw);

    pushHeadState(state, angularVelocity, true);
}

// Anonymous namespace to avoid symbol collision
namespace {

    class HDK2JniTrackerDevice {
    private:
        osvr::pluginkit::DeviceToken m_dev;
        OSVR_TrackerDeviceInterface m_tracker;
        OSVR_AnalogDeviceInterface m_analog;

    public:
        HDK2JniTrackerDevice(OSVR_PluginRegContext ctx)
        {
            // @todo sanity check for constructor arguments. All ptrs have to
            // be non-null

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

            /// Configure the tracker interface
            osvrDeviceTrackerConfigure(opts, &m_tracker);
            
            // Configure the analog interface
            osvrDeviceAnalogConfigure(opts, &m_analog, OSVR_HDK2JNI_NUM_ANALOG_CHANNELS);

            /// Create the sync device token with the options
            m_dev.initAsync(ctx, "HDK2JniTracker", opts);

            /// Send JSON descriptor
            m_dev.sendJsonDescriptor(org_osvr_android_hdk2jni_json);

            /// Register update callback
            m_dev.registerUpdateCallback(this);
        }

        ~HDK2JniTrackerDevice() {
            
        }

        OSVR_ReturnCode update() {
            OSVR_ReturnCode ret = OSVR_RETURN_SUCCESS;
            std::lock_guard<std::mutex> lock(sStateQueueMutex);
            
            // Head
            while(!sHeadPoseStateQueueSynced.empty()) {
                auto trackerReport = sHeadPoseStateQueueSynced.front();
                sHeadPoseStateQueueSynced.pop();
                if(OSVR_RETURN_SUCCESS !=
                    osvrDeviceTrackerSendPoseTimestamped(
                        m_dev, m_tracker, &trackerReport.state, OSVR_HDK2JNI_HEAD_TRACKER_CHANNEL, &trackerReport.timestamp)) {
                    LOGE("[org_osvr_android_hdk2jni]: Failed to send tracker orientation.");
                    ret = OSVR_RETURN_FAILURE;
                    continue;
                }

                if(trackerReport.hasAngularVelocity) {
                    if(OSVR_RETURN_SUCCESS != osvrDeviceTrackerSendAngularVelocityTimestamped(
                        m_dev, m_tracker, &trackerReport.angularVelocity, OSVR_HDK2JNI_HEAD_TRACKER_CHANNEL, &trackerReport.timestamp)) {

                        LOGE("[org_osvr_android_hdk2jni]: Failed to send tracker angular velocity.");
                        ret = OSVR_RETURN_FAILURE;
                        continue;
                    }
                }
            } // end while

            // analog states
            while(!sHDK2AnalogStateQueueSynced.empty()) {
                auto analogReport = sHDK2AnalogStateQueueSynced.front();
                sHDK2AnalogStateQueueSynced.pop();

                OSVR_AnalogState analogState[OSVR_HDK2JNI_NUM_ANALOG_CHANNELS];
                for(int i = 0; i < OSVR_HDK2JNI_NUM_ANALOG_CHANNELS; i++) {
                    analogState[i] = 0.0;
                }

                analogState[OSVR_HDK2JNI_VERSION_ANALOG_CHANNEL] = analogReport.version;
                analogState[OSVR_HDK2JNI_VIDEO_STATUS_ANALOG_CHANNEL] = analogReport.videoStatus;

                if(OSVR_RETURN_SUCCESS != osvrDeviceAnalogSetValuesTimestamped(
                    m_dev, m_analog, analogState, OSVR_HDK2JNI_NUM_ANALOG_CHANNELS, &analogReport.timestamp)) {
                    LOGE("[org_osvr_android_hdk2jni]: Failed to send HDK2 analog values.");
                    ret = OSVR_RETURN_FAILURE;
                    continue;
                }

            } // end while
            return ret;
        } // end update()
    }; // end class HDK2JniTrackerDevice

    class HardwareDetection {
    private:
        bool mDeviceAdded = false;
    public:
        HardwareDetection() {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
            if(!mDeviceAdded) {
                LOGI("[org_osvr_android_hdk2jni]: Adding device HDK2JniTracker.");
                /// Create our device object
                osvr::pluginkit::registerObjectForDeletion(ctx,
                    new HDK2JniTrackerDevice(ctx));
                mDeviceAdded = true;
            }
            return OSVR_RETURN_SUCCESS;
        }
    };
} // namespace

OSVR_PLUGIN(org_osvr_android_hdk2jni) {
    LOGI("[org_osvr_android_hdk2jni]: Entry point executed!");
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
