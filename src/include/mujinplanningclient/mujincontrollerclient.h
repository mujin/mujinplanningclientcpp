// -*- coding: utf-8 -*-
//
// Copyright (C) 2012-2013 MUJIN Inc. <rosen.diankov@mujin.co.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \file mujinplanningclient.h
    \brief  Defines the public headers of the MUJIN Controller Client
 */
#ifndef MUJIN_PLANNINGCLIENT_H
#define MUJIN_PLANNINGCLIENT_H

#ifdef _MSC_VER

#pragma warning(disable:4251)// needs to have dll-interface to be used by clients of class
#pragma warning(disable:4190)// C-linkage specified, but returns UDT 'boost::shared_ptr<T>' which is incompatible with C
#pragma warning(disable:4819)//The file contains a character that cannot be represented in the current code page (932). Save the file in Unicode format to prevent data loss using native typeof

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif

#else
#endif

#if defined(__GNUC__)
#define MUJINCLIENT_DEPRECATED __attribute__((deprecated))
#else
#define MUJINCLIENT_DEPRECATED
#endif

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <exception>

#include <iomanip>
#include <fstream>
#include <sstream>

#include <boost/version.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>

#include <mujinwebstackclient/webstackclient.h>

#include <mujinplanningclient/config.h>
#include <mujinplanningclient/mujinexceptions.h>
#include <mujinplanningclient/mujinjson.h>
#include <mujinplanningclient/mujindefinitions.h>


namespace mujinclient {

typedef mujin::Transform Transform;

enum TaskResourceOptions
{
    TRO_EnableZMQ=1, ///< create a task resource with zeromq client
};

typedef double Real;

class MUJINCLIENT_API WebResource
{
public:
    WebResource(PlanningClientPtr controller, const std::string& resourcename, const std::string& pk);
    virtual ~WebResource() {
    }

    inline PlanningClientPtr GetController() const {
        return __controller;
    }
    inline const std::string& GetResourceName() const {
        return __resourcename;
    }
    inline const std::string& GetPrimaryKey() const {
        return __pk;
    }

    /// \brief gets an attribute of this web resource
    template<class T>
    inline T Get(const std::string& field, double timeout = 5.0) {
        rapidjson::Document pt(rapidjson::kObjectType);
        GetWrap(pt, field, timeout);
        return mujinjson::GetJsonValueByKey<T>(pt, field.c_str());
    }

    /// \brief sets an attribute of this web resource
    virtual void Set(const std::string& field, const std::string& newvalue, double timeout = 5.0);

    /// \brief sets an attribute of this web resource
    virtual void SetJSON(const std::string& json, double timeout = 5.0);

    /// \brief delete the resource and all its child resources
    virtual void Delete(double timeout = 5.0);

    /// \brief copy the resource and all its child resources to a new name
    virtual void Copy(const std::string& newname, int options, double timeout = 5.0);

private:
    virtual void GetWrap(rapidjson::Document& pt, const std::string& field, double timeout = 5.0);

    PlanningClientPtr __controller;
    std::string __resourcename, __pk;
};

struct SensorData {
public:
    bool operator!=(const SensorData& other) const {
        return std::memcmp(distortion_coeffs, other.distortion_coeffs, 5 * sizeof(Real)) != 0 ||
                distortion_model != other.distortion_model ||
                focal_length != other.focal_length ||
                std::memcmp(image_dimensions, other.image_dimensions, 3 * sizeof(int)) != 0 ||
                std::memcmp(intrinsic, other.intrinsic, 6 * sizeof(Real)) != 0 ||
                measurement_time != other.measurement_time ||
                extra_parameters != other.extra_parameters;
    }
    bool operator==(const SensorData& other) const {
        return !operator!=(other);
    }
    Real distortion_coeffs[5];
    std::string distortion_model;
    Real focal_length;
    int image_dimensions[3];
    Real intrinsic[6];
    Real measurement_time;
    std::vector<Real> extra_parameters;
};

enum MinViableRegionRegistrationMode : uint8_t {
    MVRRM_None = 0, ///< registration without touching
    MVRRM_Lift = 1,
    MVRRM_Drag = 2,
    MVRRM_PerpendicularDrag = 3,
};

class MUJINCLIENT_API BinPickingTaskResource : public WebResource
{
public:
    BinPickingTaskResource(PlanningClientPtr controller, const std::string& pk, const std::string& scenepk, const std::string& tasktype = "binpicking");
    virtual ~BinPickingTaskResource();

    /// \brief Get all the transforms the results are storing. Depending on the optimization, can be more than just the robot
    virtual void GetEnvironmentState(EnvironmentState& envstate);

    struct MUJINCLIENT_API DetectedObject
    {
        std::string name;             // "name": "detectionresutl_1"
        std::string object_uri;       // "object_uri": "mujin:/box0.mujin.dae"
        Transform transform; ///< pose of the object in the world coordinate system
        std::string confidence;
        unsigned long long timestamp; ///< sensor timestamp in ms (from Linux epoch)
        std::string extra;            // (OPTIONAL) "extra": {"type":"randombox", "length":100, "width":100, "height":100}
        bool isPickable; ///< whether the object is pickable
    };

    struct MUJINCLIENT_API ResultBase
    {
        virtual void Parse(const rapidjson::Value& pt) = 0;
    };
    typedef boost::shared_ptr<ResultBase> ResultBasePtr;

    struct MUJINCLIENT_API ResultGetBinpickingState : public ResultBase
    {
        /// \brief holds published occlusion results of camera and container pairs
        struct OcclusionResult
        {
            mujin::SensorSelectionInfo sensorSelectionInfo;
            std::string bodyname;
            int isocclusion;
        };

        ResultGetBinpickingState();
        virtual ~ResultGetBinpickingState();
        void Parse(const rapidjson::Value& pt);
        std::string statusPickPlace;
        std::string statusDescPickPlace;
        std::string statusPhysics;
        bool isDynamicEnvironmentStateEmpty;

        int pickAttemptFromSourceId;
        unsigned long long timestamp;  ///< ms
        unsigned long long lastGrabbedTargetTimeStamp;   ///< ms
        //bool isRobotOccludingSourceContainer; ///?
        std::vector<OcclusionResult> vOcclusionResults;
        std::vector<mujin::LocationTrackingInfo> activeLocationTrackingInfos;
        std::vector<mujin::LocationExecutionInfo> locationExecutionInfos;

        bool isGrabbingTarget;
        bool isGrabbingLastTarget;

        bool hasRobotExecutionStarted=false;
        int orderNumber; ///< -1 if not initiaized
        int numLeftInOrder; ///< -1 if not initiaized
        int numLeftInSupply; ///< -1 if not initiaized
        int placedInDest; ///< -1 if not initiaized
        std::string cycleIndex; ///< index of the published cycle that pickworker is currently executing

        /**
         * Information needed to register a new object using a min viable region
         */
        struct RegisterMinViableRegionInfo
        {
            RegisterMinViableRegionInfo();

            class CopyableRapidJsonDocument : public rapidjson::Document
            {
            public:
                // Since ResultGetBinpickingState needs to be copyable while rapidjson::Document is not, there needs to be a small wrapper
                CopyableRapidJsonDocument& operator=(const CopyableRapidJsonDocument& other) {
                    SetNull();
                    GetAllocator().Clear();
                    CopyFrom(other, GetAllocator());
                    return *this;
                }
            };

            struct MinViableRegionInfo
            {
                MinViableRegionInfo();
                std::array<double, 2> size2D; ///< width and length on the MVR
                std::array<double, 3> maxPossibleSize; ///< the max possible size of actual item
                std::array<double, 3> maxPossibleSizeOriginal; ///< the maxPossibleSize that has originally been given from vision
                uint8_t cornerMaskOriginal; ///< the cornerMask that has originally been given from vision
                uint8_t cornerMask; ///< Represents the corner(s) used for corner based detection. 4 bit. -x-y = 1, +x-y = 2, -x+y = 4, +x+y = 8
            } minViableRegion;

            std::string locationName; ///< The name of the location where the minViableRegion was triggered for
            std::array<double, 3> translation_; ///< Translation of the 2D MVR plane (height = 0)
            std::array<double, 4> quat_; ///< Rotation of the 2D MVR plane (height = 0)
            double objectWeight; ///< If non-zero, use this weight fo registration. unit is kg. zero means unknown.
            uint64_t sensorTimeStampMS; ///< Same as DetectedObject's timestamp sent to planning
            double robotDepartStopTimestamp; ///< Force capture after robot stops
            std::array<double, 3> liftedWorldOffset; ///< [dx, dy, dz], mm in world frame
            std::array<double, 3> maxCandidateSize; ///< the max candidate size expecting
            std::array<double, 3> minCandidateSize; ///< the min candidate size expecting
            double transferSpeedPostMult; ///< transfer speed multiplication factor
            CopyableRapidJsonDocument graspModelInfo; ///< Parameters used for grasping model generation for the object
            double minCornerVisibleDist; ///< how much distance along with uncertain edge from uncertain corner robot exposes to camera
            double minCornerVisibleInsideDist; ///< how much distance inside MVR robot exposes to camera
            double maxCornerAngleDeviation; ///< how much angle deviation around uncertain corner is considered to expose to camera
            uint8_t occlusionFreeCornerMask; ///< mask of corners that robot exposes to camera. 4 bit. -x-y = 1, +x-y = 2, -x+y = 4, +x+y = 8
            MinViableRegionRegistrationMode registrationMode; ///< lift, drag or perpendicular drag
            bool skipAppendingToObjectSet; ///<  if true, skip appending newly created registration data into an active object set
            double maxPossibleSizePadding; ///< how much to additionally expose max possible size region to vision
            std::vector<double> fullDofValues; ///< robot configuration state on capturing
            std::vector<int8_t> connectedBodyActiveStates; ///< robot configuration state on capturing
            bool IsEmpty() const {
                return sensorTimeStampMS == 0;
            }
        } registerMinViableRegionInfo;

        struct RemoveObjectFromObjectListInfo {
            RemoveObjectFromObjectListInfo();
            double timestamp; // timestamp this request was sent. If non-zero, then valid.
            std::string objectPk; // objectPk to remove from the current object set vision is using
            bool IsEmpty() const {
                return objectPk.empty();
            }
        };
        std::vector<RemoveObjectFromObjectListInfo> removeObjectFromObjectListInfos;

        struct TriggerDetectionCaptureInfo {
            TriggerDetectionCaptureInfo();
            double timestamp; ///< timestamp this request was sent. If non-zero, then valid.
            std::string triggerType; ///< The type of trigger this is. For now can be: "phase1Detection", "phase2Detection"
            std::string locationName; ///< The name of the location for this detection trigger.
            std::string targetupdatename; ///< if not empty, use this new targetupdatename for the triggering, otherwise do not change the original targetupdatename
        } triggerDetectionCaptureInfo;

        std::vector<mujin::PickPlaceHistoryItem> pickPlaceHistoryItems; ///< history of pick/place actions that occurred in planning. Events should be sorted in the order they happen, ie event [0] happens before event [1], meaning event[0].eventTimeStampUS is before event[1].eventTimeStampUS
    };

    struct MUJINCLIENT_API ResultOBB : public ResultBase
    {
        void Parse(const rapidjson::Value& pt);
        bool operator!=(const ResultOBB& other) const {
            return !mujin::FuzzyEquals(translation, other.translation) ||
                   !mujin::FuzzyEquals(extents, other.extents) ||
                   !mujin::FuzzyEquals(rotationmat, other.rotationmat) ||
                   !mujin::FuzzyEquals(quaternion, other.quaternion);
        }
        bool operator==(const ResultOBB& other) const {
            return !operator!=(other);
        }
        std::vector<Real> translation;
        std::vector<Real> extents;
        std::vector<Real> rotationmat;  // row major
        std::vector<Real> quaternion; // the quaternion
    };

    struct MUJINCLIENT_API ResultInstObjectInfo : public ResultBase
    {
        virtual ~ResultInstObjectInfo();
        void Parse(const rapidjson::Value& pt);
        Transform instobjecttransform;
        ResultOBB instobjectobb;
        ResultOBB instobjectinnerobb;
        rapidjson::Document rGeometryInfos; ///< for every object, list of all the geometry infos
        rapidjson::Document rIkParams; ///< for every object, dict of serialized ikparams
    };

    struct MUJINCLIENT_API ResultGetInstObjectAndSensorInfo : public ResultBase
    {
        virtual ~ResultGetInstObjectAndSensorInfo();
        void Parse(const rapidjson::Value& pt);
        std::map<std::string, std::string> muri;
        std::map<std::string, Transform> minstobjecttransform;
        std::map<std::string, ResultOBB> minstobjectobb;
        std::map<std::string, ResultOBB> minstobjectinnerobb;
        std::map<mujin::SensorSelectionInfo, Transform> msensortransform;
        std::map<mujin::SensorSelectionInfo, RobotResource::AttachedSensorResource::SensorData> msensordata;
        std::map<std::string, boost::shared_ptr<rapidjson::Document> > mrGeometryInfos; ///< for every object, list of all the geometry infos
    };

    /** \brief Initializes binpicking task.
        \param commandtimeout seconds until this command times out
        \param userinfo json string user info, such as locale
        \param slaverequestid id of mujincontroller planning slave to connect to
     */
    virtual void Initialize(const std::string& defaultTaskParameters, const double commandtimeout=5.0, const std::string& userinfo="{}", const std::string& slaverequestid="");

#ifdef MUJIN_USEZMQ
    /** \brief Initializes binpicking task.
        \param zmqPort port of the binpicking zmq server
        \param heartbeatPort port of the binpicking zmq server's heartbeat publisher
        \param zmqcontext zmq context
        \param initializezmq whether to call InitializeZMQ() in this call
        \param reinitializetimeout seconds until calling InitailizeZMQ() if heartbeat has not been received. If 0, do not reinitialize
        \param commandtimeout seconds until this command times out
        \param userinfo json string user info, such as locale
        \param slaverequestid id of mujincontroller planning slave to connect to
     */
    virtual void Initialize(const std::string& defaultTaskParameters, const int zmqPort, const int heartbeatPort, boost::shared_ptr<zmq::context_t> zmqcontext, const bool initializezmq=false, const double reinitializetimeout=10, const double commandtimeout=0, const std::string& userinfo="{}", const std::string& slaverequestid="");
#endif

    virtual void SetCallerId(const std::string& callerid);

    virtual void ExecuteCommand(const std::string& command, rapidjson::Document&d, const double timeout /* second */=5.0, const bool getresult=true);

    /// \brief executes command directly from rapidjson::Value struct.
    ///
    /// \param rTaskParameters will be destroyed after the call due to r-value moves
    virtual void ExecuteCommand(rapidjson::Value& rTaskParameters, rapidjson::Document& rOutput, const double timeout /* second */=5.0);

    virtual void GetInnerEmptyRegionOBB(ResultOBB& result, const std::string& targetname, const std::string& linkname, const std::string& unit, const double timeout=0);

    /** \brief Update objects in the scene
        \param basename base name of the object. e.g. objects will have name basename_0, basename_1, etc
        \param transformsworld list of transforms in world frame
        \param confidence list of confidence of each detection
        \param state additional information about the objects
        \param unit unit of detectedobject info
        \param timeout seconds until this command times out
     */
    virtual void UpdateObjects(const std::string& objectname, const std::vector<DetectedObject>& detectedobjects, const std::string& resultstate, const std::string& unit, const double timeout /* second */=5.0);

    /** \brief Establish ZMQ connection to the task
        \param reinitializetimeout seconds to wait before re-initializing the ZMQ server after the heartbeat signal is lost
               if reinitializetimeout is 0, then this does not invoke heartbeat monitoring thread
        \param timeout seconds until this command times out
     */
    virtual void InitializeZMQ(const double reinitializetimeout = 0,const double timeout /* second */=5.0);

    /** \brief Add a point cloud collision obstacle with name to the environment.
        \param vpoints list of x,y,z coordinates in meter
        \param state additional information about the objects
        \param pointsize size of each point in meter
        \param name name of the obstacle
        \param isoccluded occlusion status of robot with the container: -1 if unknown, 0 if not occluding, 1 if robot is occluding region in camera
        \param locationName the name of the region for which the point cloud is supposed to be captured of. isoccluded maps to this region
        \param timeout seconds until this command times out
        \param clampToContainer if true, then planning will clamp the points to the container walls specified by locationName. Otherwise, will use all the points
        \param rExtraParameters extra parameters to be inserted
     */
    virtual void AddPointCloudObstacle(const std::vector<float>& vpoints, const Real pointsize, const std::string& name,  const unsigned long long starttimestamp, const unsigned long long endtimestamp, const bool executionverification, const std::string& unit, int isoccluded=-1, const std::string& locationName=std::string(), const double timeout /* second */=5.0, bool clampToContainer=true, CropContainerMarginsXYZXYZPtr pCropContainerMargins=CropContainerMarginsXYZXYZPtr(), AddPointOffsetInfoPtr pAddPointOffsetInfo=AddPointOffsetInfoPtr());

    /** \brief Visualize point cloud on controller
        \param pointslist vector of x,y,z coordinates vector in meter
        \param pointsize size of each point in meter
        \param names vector of names for each point cloud
        \param unit of points
        \param timeout seconds until this command times out
     */
    virtual void VisualizePointCloud(const std::vector<std::vector<float> >& pointslist, const Real pointsize, const std::vector<std::string>& names, const std::string& unit, const double timeout /* second */=5.0);

    /** \brief Clear visualization made by VisualizePointCloud.
     */
    virtual void ClearVisualization(const double timeout /* second */=5.0);

    /** \brief Check if robot is occluding the object in the view of sensor between starttime and endtime
        \param sensorname name of the sensor to be checked, example names: "sensor_kinbodyname/sensor_name" or "sensor_kinbodyname", in the latter case the first attached sensor will be used
        \param timeout seconds until this command times out
     */
    virtual void IsRobotOccludingBody(const std::string& bodyname, const std::string& sensorname, const unsigned long long starttime, const unsigned long long endtime, bool& result, const double timeout /* second */=5.0);

    /** \brief Gets inst object and sensor info of existing objects in the scene
        \param unit input unit
        \param result unit is always in meter
     */
    virtual void GetInstObjectAndSensorInfo(const std::vector<std::string>& instobjectnames, const std::vector<mujin::SensorSelectionInfo>& sensornames, ResultGetInstObjectAndSensorInfo& result, const std::string& unit, const double timeout /* second */=5.0);

    /** \brief Gets inst object info for a particular URI
        \param unit input unit
        \param result unit is always in meter
     */
    virtual void GetInstObjectInfoFromURI(const std::string& objecturi, const Transform& instobjecttransform, ResultInstObjectInfo& result, const std::string& unit, const double timeout /* second */=5.0);

    /// \brief Get state of bin picking
    /// \param result state of bin picking
    /// \param robotname name of robot
    /// \param unit unit to receive values in, either "m" (indicates radian for angle) or "mm" (indicates degree for angle)
    /// \param timeout timeout of communication
    virtual void GetBinpickingState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout /* second */=5.0);

    /// \brief Get state of ITL which includes picking status
    /// \param result state of bin picking
    /// \param robotname name of robot
    /// \param unit unit to receive values in, either "m" (indicates radian for angle) or "mm" (indicates degree for angle)
    /// \param timeout timeout of communication
    virtual void GetITLState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout /* second */=5.0);

    /// \brief Get published state of bin picking
    /// except for initial call, this returns cached value.
    /// \param result state of bin picking
    /// \param robotname name of robot
    /// \param unit unit to receive values in, either "m" (indicates radian for angle) or "mm" (indicates degree for angle)
    /// \param timeout timeout of communication
    virtual void GetPublishedTaskState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout /* second */=5.0);

    virtual void SendMVRRegistrationResult(
        const rapidjson::Document &mvrResultInfo,
        double timeout /* second */=5.0);

    // send result of RemoveObjectsFromObjectList request
    virtual void SendRemoveObjectsFromObjectListResult(const std::vector<ResultGetBinpickingState::RemoveObjectFromObjectListInfo>& removeObjectFromObjectListInfos, const bool success, const double timeout /* second */=5.0);

    // send result of RemoveObjectFromObjectList request
    virtual void SendTriggerDetectionCaptureResult(const std::string& triggerType, const std::string& returnCode, double timeout /* second */=5.0);

protected:
    const std::string& _GetCallerId() const;

    /** \brief Monitors heartbeat signals from a running binpicking ZMQ server, and reinitializes the ZMQ server when heartbeat is lost.
        \param reinitializetimeout seconds to wait before re-initializing the ZMQ server after the heartbeat signal is lost
     */
    virtual void _HeartbeatMonitorThread(const double reinitializetimeout, const double commandtimeout);

    std::stringstream _ss;

    std::map<std::string, std::string> _mapTaskParameters; ///< set of key value pairs that should be included
    std::string _mujinControllerIp;
    boost::mutex _mutexTaskState;
    ResultGetBinpickingState _taskstate;
#ifdef MUJIN_USEZMQ
    boost::shared_ptr<zmq::context_t> _zmqcontext;
#endif
    int _zmqPort;
    int _heartbeatPort;

    rapidjson::Document _rUserInfo;  ///< userinfo json
    rapidjson::Document _rSceneParams; ///\ parameters of the scene to run tasks on the backend zmq slave
    std::string _sceneparams_json, _userinfo_json;
    
    std::string _slaverequestid; ///< to ensure the same slave is used for binpicking task
    std::string _scenepk; ///< scene pk
    std::string _callerid; ///< string identifying the caller
    const std::string _tasktype; ///< the specific task type to create internally. As long as the task supports the binpicking interface, it can be used.
    boost::shared_ptr<boost::thread> _pHeartbeatMonitorThread;

    bool _bIsInitialized;
    bool _bShutdownHeartbeatMonitor;
};
typedef boost::shared_ptr<BinPickingTaskResource> BinPickingTaskResourcePtr;
typedef boost::weak_ptr<BinPickingTaskResource> BinPickingTaskResourceWeakPtr;


namespace utils {
MUJINCLIENT_API std::string GetJsonString(const std::string& string);
MUJINCLIENT_API std::string GetJsonString(const std::vector<float>& vec);
MUJINCLIENT_API std::string GetJsonString(const std::vector<double>& vec);
MUJINCLIENT_API std::string GetJsonString(const std::vector<int>& vec);
MUJINCLIENT_API std::string GetJsonString(const std::vector<std::string>& vec);
MUJINCLIENT_API std::string GetJsonString(const Transform& transform);
MUJINCLIENT_API std::string GetJsonString(const BinPickingTaskResource::DetectedObject& obj);
template <typename T, size_t N>
MUJINCLIENT_API std::string GetJsonString(const std::array<T, N>& a)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<T>::digits10+1);
    ss << "[";
    for (unsigned int i = 0; i < N; ++i) {
        ss << a[i];
        if (i != N - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

MUJINCLIENT_API std::string GetJsonString(const std::string& key, const std::string& value);
MUJINCLIENT_API std::string GetJsonString(const std::string& key, const int value);
MUJINCLIENT_API std::string GetJsonString(const std::string& key, const unsigned long long value);
MUJINCLIENT_API std::string GetJsonString(const std::string& key, const Real value);

#ifdef MUJIN_USEZMQ
/// \brief get heartbeat
/// \param endopoint endpoint to get heartbeat from. looks like protocol://hostname:port (ex. tcp://localhost:11001)
/// \return heartbeat as string
MUJINCLIENT_API std::string GetHeartbeat(const std::string& endpoint);

MUJINCLIENT_API std::string GetScenePkFromHeartbeat(const std::string& heartbeat);
MUJINCLIENT_API std::string GetSlaveRequestIdFromHeartbeat(const std::string& heartbeat);
#endif
} // namespace utils

} // namespace mujinclient

BOOST_STATIC_ASSERT(MUJINCLIENT_VERSION_MAJOR>=0&&MUJINCLIENT_VERSION_MAJOR<=255);
BOOST_STATIC_ASSERT(MUJINCLIENT_VERSION_MINOR>=0&&MUJINCLIENT_VERSION_MINOR<=255);
BOOST_STATIC_ASSERT(MUJINCLIENT_VERSION_PATCH>=0&&MUJINCLIENT_VERSION_PATCH<=255);

#endif