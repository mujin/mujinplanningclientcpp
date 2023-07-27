// -*- coding: utf-8 -*-
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
#include "common.h"
#include <boost/thread.hpp> // for sleep
#include <boost/algorithm/string.hpp>

#if BOOST_VERSION > 104800
#include <boost/algorithm/string/replace.hpp>
#endif
#include <boost/thread.hpp> // for sleep

#include "mujinplanningclient/zmq.hpp"

#ifdef _WIN32
#include <float.h>
#define isnan _isnan
#endif

#include <cmath>

#include "logging.h"

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include "mujinplanningclient/mujinjson.h"

#include "logging.h"
#include "mujinplanningclient/mujinjson.h"

MUJIN_LOGGER("mujin.planningclientcpp");

namespace mujinplanningclient {

namespace {

using namespace mujinjson;
using namespace utils;

void SetMapTaskParameters(std::stringstream &ss, const std::map<std::string, std::string> &params)
{
    ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss.str("");
    ss.clear();
    ss << "{";
    FOREACHC(it, params) {
        ss << "\"" << it->first << "\":" << it->second << ", ";
    }
}

void SerializeGetStateCommand(std::stringstream &ss, const std::map<std::string, std::string> &params,
                              const std::string &functionname, const std::string &tasktype,
                              const std::string &robotname, const std::string &unit, const double timeout) {
    SetMapTaskParameters(ss, params);

    ss << GetJsonString("command", functionname) << ", ";
    ss << GetJsonString("tasktype", tasktype) << ", ";
    if (!robotname.empty()) {
        ss << GetJsonString("robotname", robotname) << ", ";
    }
    ss << GetJsonString("unit", unit);
    ss << "}";
}

void LoadAABBFromJsonValue(const rapidjson::Value& rAABB, mujin::AABB& aabb)
{
    BOOST_ASSERT(rAABB.IsObject());
    BOOST_ASSERT(rAABB.HasMember("pos"));
    BOOST_ASSERT(rAABB.HasMember("extents"));
    const rapidjson::Value& rPos = rAABB["pos"];
    BOOST_ASSERT(rPos.IsArray());
    mujinjson::LoadJsonValue(rPos[0], aabb.pos[0]);
    mujinjson::LoadJsonValue(rPos[1], aabb.pos[1]);
    mujinjson::LoadJsonValue(rPos[2], aabb.pos[2]);
    const rapidjson::Value& rExtents = rAABB["extents"];
    BOOST_ASSERT(rExtents.IsArray());
    mujinjson::LoadJsonValue(rExtents[0], aabb.extents[0]);
    mujinjson::LoadJsonValue(rExtents[1], aabb.extents[1]);
    mujinjson::LoadJsonValue(rExtents[2], aabb.extents[2]);
}

}  // end namespace

MujinPlanningClient::ResultGetBinpickingState::ResultGetBinpickingState() :
    statusPickPlace(""),
    statusDescPickPlace(""),
    statusPhysics(""),
    isDynamicEnvironmentStateEmpty(false),
    pickAttemptFromSourceId(-1),
    timestamp(0),
    lastGrabbedTargetTimeStamp(0),
    isGrabbingTarget(true),
    isGrabbingLastTarget(true),
    hasRobotExecutionStarted(false),
    orderNumber(-1),
    numLeftInOrder(-1),
    numLeftInSupply(-1),
    placedInDest(-1) {}

MujinPlanningClient::ResultGetBinpickingState::~ResultGetBinpickingState() {}

void MujinPlanningClient::ResultGetBinpickingState::Parse(const rapidjson::Value& pt)
{
    BOOST_ASSERT(pt.IsObject() && pt.HasMember("output"));
    const rapidjson::Value& v = pt["output"];

    statusPickPlace = GetJsonValueByKey<std::string>(v, "statusPickPlace", "unknown");
    statusDescPickPlace = GetJsonValueByKey<std::string>(v, "statusDescPickPlace", "unknown");
    cycleIndex = GetJsonValueByKey<std::string>(v, "statusPickPlaceCycleIndex", "");
    statusPhysics = GetJsonValueByKey<std::string>(v, "statusPhysics", "unknown");
    pickAttemptFromSourceId = GetJsonValueByKey<int>(v, "pickAttemptFromSourceId", -1);
    //isContainerEmptyMap.clear();
    //LoadJsonValueByKey(v, "isContainerEmptyMap", isContainerEmptyMap);
    //lastInsideSourceTimeStamp = (unsigned long long)(GetJsonValueByKey<double>(v, "lastInsideSourceTimeStamp", 0) * 1000.0); // s -> ms
    //lastInsideDestTimeStamp = (unsigned long long)(GetJsonValueByKey<double>(v, "lastInsideDestTimeStamp", 0) * 1000.0); // s -> ms
    timestamp = (unsigned long long)(GetJsonValueByKey<double>(v, "timestamp", 0) * 1000.0); // s -> ms
    lastGrabbedTargetTimeStamp = (unsigned long long)(GetJsonValueByKey<double>(v, "lastGrabbedTargetTimeStamp", 0) * 1000.0); // s -> ms

    vOcclusionResults.clear();
    const rapidjson::Value::ConstMemberIterator itOcclusionResults = v.FindMember("occlusionResults");
    if( itOcclusionResults != v.MemberEnd() && itOcclusionResults->value.IsArray() ) {
        vOcclusionResults.resize(itOcclusionResults->value.Size());
        for(int iocc = 0; iocc < (int)vOcclusionResults.size(); ++iocc) {
            const rapidjson::Value& result = itOcclusionResults->value[iocc];
            vOcclusionResults[iocc].sensorSelectionInfo = GetJsonValueByKey<mujin::SensorSelectionInfo,mujin::SensorSelectionInfo>(result, "sensorSelectionInfo", mujin::SensorSelectionInfo());
            vOcclusionResults[iocc].bodyname = GetJsonValueByKey<std::string,std::string>(result, "bodyname", std::string());
            vOcclusionResults[iocc].isocclusion = GetJsonValueByKey<int,int>(result, "isocclusion", -1);
        }
    }

    isGrabbingTarget = GetJsonValueByKey<bool>(v, "isGrabbingTarget", true);
    isGrabbingLastTarget = GetJsonValueByKey<bool>(v, "isGrabbingLastTarget", true);
    hasRobotExecutionStarted = GetJsonValueByKey<bool>(v, "hasRobotExecutionStarted", false);
    orderNumber = GetJsonValueByPath<int>(v, "/orderstate/orderNumber", -1);
    numLeftInOrder = GetJsonValueByPath<int>(v, "/orderstate/numLeftInOrder", -1);
    numLeftInSupply = GetJsonValueByPath<int>(v, "/orderstate/numLeftInSupply", -1);
    placedInDest = GetJsonValueByPath<int>(v, "/orderstate/placedInDest", -1);

    registerMinViableRegionInfo.locationName = GetJsonValueByPath<std::string>(v, "/registerMinViableRegionInfo/locationName", std::string());
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/translation_", registerMinViableRegionInfo.translation_);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/quat_", registerMinViableRegionInfo.quat_);
    registerMinViableRegionInfo.objectWeight = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/objectWeight", 0);
    registerMinViableRegionInfo.sensorTimeStampMS = GetJsonValueByPath<uint64_t>(v, "/registerMinViableRegionInfo/sensorTimeStampMS", 0);
    registerMinViableRegionInfo.robotDepartStopTimestamp = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/robotDepartStopTimestamp", 0);
    registerMinViableRegionInfo.transferSpeedPostMult = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/transferSpeedPostMult", 1.0);
    {
        registerMinViableRegionInfo.graspModelInfo.SetNull();
        registerMinViableRegionInfo.graspModelInfo.GetAllocator().Clear();
        const rapidjson::Value* graspModelInfoJson = rapidjson::Pointer("/registerMinViableRegionInfo/graspModelInfo").Get(v);
        if( !!graspModelInfoJson && graspModelInfoJson->IsObject() ) {
            registerMinViableRegionInfo.graspModelInfo.CopyFrom(*graspModelInfoJson, registerMinViableRegionInfo.graspModelInfo.GetAllocator());
        }
    }
    registerMinViableRegionInfo.minCornerVisibleDist = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/minCornerVisibleDist", 30);
    registerMinViableRegionInfo.minCornerVisibleInsideDist = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/minCornerVisibleInsideDist", 0);
    registerMinViableRegionInfo.maxCornerAngleDeviation = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/maxCornerAngleDeviation", 0);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/minViableRegion/size2D", registerMinViableRegionInfo.minViableRegion.size2D);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/minViableRegion/maxPossibleSize", registerMinViableRegionInfo.minViableRegion.maxPossibleSize);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/minViableRegion/maxPossibleSizeOriginal", registerMinViableRegionInfo.minViableRegion.maxPossibleSizeOriginal, registerMinViableRegionInfo.minViableRegion.maxPossibleSize);
    registerMinViableRegionInfo.minViableRegion.cornerMask = GetJsonValueByPath<uint8_t>(v, "/registerMinViableRegionInfo/minViableRegion/cornerMask", 0);
    registerMinViableRegionInfo.minViableRegion.cornerMaskOriginal = GetJsonValueByPath<uint8_t>(v, "/registerMinViableRegionInfo/minViableRegion/cornerMaskOriginal", 0);
    registerMinViableRegionInfo.occlusionFreeCornerMask = GetJsonValueByPath<uint8_t>(v, "/registerMinViableRegionInfo/occlusionFreeCornerMask", 0);
    const uint8_t registrationMode = GetJsonValueByPath<uint8_t>(v, "/registerMinViableRegionInfo/registrationMode", MVRRM_Drag);
    if( registrationMode > MVRRM_PerpendicularDrag ) {
        throw MujinException(str(boost::format("got unexpected value %d when receiving /registerMinViableRegionInfo/registrationMode. assuming 'drag'")%(int)registrationMode), MEC_InvalidArguments);
    }
    else {
        registerMinViableRegionInfo.registrationMode = static_cast<MinViableRegionRegistrationMode>(registrationMode);
    }
    registerMinViableRegionInfo.maxPossibleSizePadding = GetJsonValueByPath<double>(v, "/registerMinViableRegionInfo/maxPossibleSizePadding", 30);
    registerMinViableRegionInfo.skipAppendingToObjectSet = GetJsonValueByPath<bool>(v, "/registerMinViableRegionInfo/skipAppendingToObjectSet", false);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/liftedWorldOffset", registerMinViableRegionInfo.liftedWorldOffset);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/minCandidateSize", registerMinViableRegionInfo.minCandidateSize);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/maxCandidateSize", registerMinViableRegionInfo.maxCandidateSize);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/fullDofValues", registerMinViableRegionInfo.fullDofValues);
    LoadJsonValueByPath(v, "/registerMinViableRegionInfo/connectedBodyActiveStates", registerMinViableRegionInfo.connectedBodyActiveStates);

    removeObjectFromObjectListInfos.clear();
    if( v.HasMember("removeObjectsFromObjectList") && v["removeObjectsFromObjectList"].IsArray()) {
        const rapidjson::Value& rRemoveObjectFromObjectList = v["removeObjectsFromObjectList"];
        removeObjectFromObjectListInfos.resize(rRemoveObjectFromObjectList.Size());
        for(int iitem = 0; iitem < (int)removeObjectFromObjectListInfos.size(); ++iitem) {
            const rapidjson::Value& rInfo = rRemoveObjectFromObjectList[iitem];
            removeObjectFromObjectListInfos[iitem].timestamp = GetJsonValueByKey<double, double>(rInfo, "timestamp", 0);
            removeObjectFromObjectListInfos[iitem].objectPk = GetJsonValueByKey<std::string, std::string>(rInfo, "objectPk", std::string());
        }
    }

    triggerDetectionCaptureInfo.timestamp = GetJsonValueByPath<double>(v, "/triggerDetectionCaptureInfo/timestamp", 0);
    triggerDetectionCaptureInfo.triggerType = GetJsonValueByPath<std::string>(v, "/triggerDetectionCaptureInfo/triggerType", "");
    triggerDetectionCaptureInfo.locationName = GetJsonValueByPath<std::string>(v, "/triggerDetectionCaptureInfo/locationName", "");
    triggerDetectionCaptureInfo.targetupdatename = GetJsonValueByPath<std::string>(v, "/triggerDetectionCaptureInfo/targetupdatename", "");

    activeLocationTrackingInfos.clear();
    if( v.HasMember("activeLocationTrackingInfos") && v["activeLocationTrackingInfos"].IsArray()) {
        const rapidjson::Value& rLocationTrackingInfo = v["activeLocationTrackingInfos"];
        activeLocationTrackingInfos.resize(rLocationTrackingInfo.Size());
        for(int iitem = 0; iitem < (int)rLocationTrackingInfo.Size(); ++iitem) {
            const rapidjson::Value& rInfo = rLocationTrackingInfo[iitem];
            activeLocationTrackingInfos[iitem].locationName = GetJsonValueByKey<std::string,std::string>(rInfo, "locationName", std::string());
            activeLocationTrackingInfos[iitem].containerId = GetJsonValueByKey<std::string,std::string>(rInfo, "containerId", std::string());
            activeLocationTrackingInfos[iitem].containerName = GetJsonValueByKey<std::string,std::string>(rInfo, "containerName", std::string());
            activeLocationTrackingInfos[iitem].containerUsage = GetJsonValueByKey<std::string,std::string>(rInfo, "containerUsage", std::string());
            activeLocationTrackingInfos[iitem].cycleIndex = GetJsonValueByKey<std::string,std::string>(rInfo, "cycleIndex", std::string());
        }
    }

    locationExecutionInfos.clear();
    if( v.HasMember("locationExecutionInfos") && v["locationExecutionInfos"].IsArray()) {
        const rapidjson::Value& rLocationTrackingInfo = v["locationExecutionInfos"];
        locationExecutionInfos.resize(rLocationTrackingInfo.Size());
        for(int iitem = 0; iitem < (int)rLocationTrackingInfo.Size(); ++iitem) {
            const rapidjson::Value& rInfo = rLocationTrackingInfo[iitem];
            locationExecutionInfos[iitem].locationName = GetJsonValueByKey<std::string,std::string>(rInfo, "locationName", std::string());
            locationExecutionInfos[iitem].containerId = GetJsonValueByKey<std::string,std::string>(rInfo, "containerId", std::string());
            locationExecutionInfos[iitem].forceRequestStampMS = GetJsonValueByKey<uint64_t, uint64_t>(rInfo, "forceRequestStampMS", 0);
            locationExecutionInfos[iitem].lastInsideContainerStampMS = GetJsonValueByKey<uint64_t, uint64_t>(rInfo, "lastInsideContainerStampMS", 0);
            locationExecutionInfos[iitem].needContainerState = GetJsonValueByKey<std::string,std::string>(rInfo, "needContainerState", std::string());
        }
    }

    pickPlaceHistoryItems.clear();
    if( v.HasMember("pickPlaceHistoryItems") && v["pickPlaceHistoryItems"].IsArray() ) {
        pickPlaceHistoryItems.resize(v["pickPlaceHistoryItems"].Size());
        for(int iitem = 0; iitem < (int)pickPlaceHistoryItems.size(); ++iitem) {
            const rapidjson::Value& rItem = v["pickPlaceHistoryItems"][iitem];
            pickPlaceHistoryItems[iitem].pickPlaceType = GetJsonValueByKey<std::string,std::string>(rItem, "pickPlaceType", std::string());
            pickPlaceHistoryItems[iitem].locationName = GetJsonValueByKey<std::string,std::string>(rItem, "locationName", std::string());
            pickPlaceHistoryItems[iitem].eventTimeStampUS = GetJsonValueByKey<unsigned long long>(rItem, "eventTimeStampUS", 0);
            pickPlaceHistoryItems[iitem].object_uri = GetJsonValueByKey<std::string,std::string>(rItem, "object_uri", std::string());

            pickPlaceHistoryItems[iitem].objectpose = Transform();
            const rapidjson::Value::ConstMemberIterator itPose = rItem.FindMember("objectpose");
            if( itPose != rItem.MemberEnd() ) {
                const rapidjson::Value& rObjectPose = itPose->value;;
                if( rObjectPose.IsArray() && rObjectPose.Size() == 7 ) {
                    LoadJsonValue(rObjectPose[0], pickPlaceHistoryItems[iitem].objectpose.quaternion[0]);
                    LoadJsonValue(rObjectPose[1], pickPlaceHistoryItems[iitem].objectpose.quaternion[1]);
                    LoadJsonValue(rObjectPose[2], pickPlaceHistoryItems[iitem].objectpose.quaternion[2]);
                    LoadJsonValue(rObjectPose[3], pickPlaceHistoryItems[iitem].objectpose.quaternion[3]);
                    LoadJsonValue(rObjectPose[4], pickPlaceHistoryItems[iitem].objectpose.translate[0]);
                    LoadJsonValue(rObjectPose[5], pickPlaceHistoryItems[iitem].objectpose.translate[1]);
                    LoadJsonValue(rObjectPose[6], pickPlaceHistoryItems[iitem].objectpose.translate[2]);
                }
            }

            pickPlaceHistoryItems[iitem].localaabb = mujin::AABB();
            const rapidjson::Value::ConstMemberIterator itLocalAABB = rItem.FindMember("localaabb");
            if( itLocalAABB != rItem.MemberEnd() ) {
                const rapidjson::Value& rLocalAABB = itLocalAABB->value;
                LoadAABBFromJsonValue(rLocalAABB, pickPlaceHistoryItems[iitem].localaabb);
            }

            pickPlaceHistoryItems[iitem].sensorTimeStampUS = GetJsonValueByKey<unsigned long long>(rItem, "sensorTimeStampUS", 0);
        }
    }
}

MujinPlanningClient::ResultGetBinpickingState::RegisterMinViableRegionInfo::RegisterMinViableRegionInfo() :
    objectWeight(0.0),
    sensorTimeStampMS(0),
    robotDepartStopTimestamp(0),
    transferSpeedPostMult(1.0),
    minCornerVisibleDist(30),
    minCornerVisibleInsideDist(0),
    occlusionFreeCornerMask(0),
    skipAppendingToObjectSet(false),
    maxPossibleSizePadding(30)
{
    translation_.fill(0);
    quat_.fill(0);
    liftedWorldOffset.fill(0);
    maxCandidateSize.fill(0);
    minCandidateSize.fill(0);
}

MujinPlanningClient::ResultGetBinpickingState::RegisterMinViableRegionInfo::MinViableRegionInfo::MinViableRegionInfo() :
    cornerMask(0)
{
    size2D.fill(0);
    maxPossibleSize.fill(0);
    maxPossibleSizeOriginal.fill(0);
}

MujinPlanningClient::ResultGetBinpickingState::RemoveObjectFromObjectListInfo::RemoveObjectFromObjectListInfo() :
    timestamp(0)
{
}

MujinPlanningClient::ResultGetBinpickingState::TriggerDetectionCaptureInfo::TriggerDetectionCaptureInfo() :
    timestamp(0)
{
}

void MujinPlanningClient::ResultOBB::Parse(const rapidjson::Value& pt)
{
    const rapidjson::Value&  v = (pt.IsObject()&&pt.HasMember("output") ? pt["output"] : pt);

    LoadJsonValueByKey(v, "translation", translation);
    LoadJsonValueByKey(v, "extents", extents);
    std::vector<std::vector<Real> > rotationmatrix2d;
    LoadJsonValueByKey(v, "rotationmat", rotationmatrix2d);
    if (translation.size() != 3) {
        throw MujinException("The length of translation is invalid.", MEC_Failed);
    }
    if (extents.size() != 3) {
        throw MujinException("The length of extents is invalid.", MEC_Failed);
    }
    if (rotationmatrix2d.size() != 3 || rotationmatrix2d[0].size() != 3 || rotationmatrix2d[1].size() != 3 || rotationmatrix2d[2].size() != 3) {
        throw MujinException("The row number of rotationmat is invalid.", MEC_Failed);
    }

    rotationmat.resize(9);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotationmat[i*3+j] = rotationmatrix2d[i][j];
        }
    }

    LoadJsonValueByKey(v, "quaternion", quaternion);
}

MujinPlanningClient::ResultInstObjectInfo::~ResultInstObjectInfo()
{
}

void MujinPlanningClient::ResultInstObjectInfo::Parse(const rapidjson::Value& pt)
{
    BOOST_ASSERT(pt.IsObject() && pt.HasMember("output"));
    const rapidjson::Value& rOutput = pt["output"];

    LoadJsonValueByKey(rOutput, "translation", instobjecttransform.translate);
    LoadJsonValueByKey(rOutput, "quaternion", instobjecttransform.quaternion);
    instobjectobb.Parse(rOutput["obb"]);
    instobjectinnerobb.Parse(rOutput["innerobb"]);

    if( rOutput.HasMember("geometryInfos") ) {
        mujinjson::SaveJsonValue(rGeometryInfos, rOutput["geometryInfos"]);
    }

    if( rOutput.HasMember("ikparams") ) {
        mujinjson::SaveJsonValue(rIkParams, rOutput["ikparams"]);
    }
}

MujinPlanningClient::ResultGetInstObjectAndSensorInfo::~ResultGetInstObjectAndSensorInfo()
{
}

void MujinPlanningClient::ResultGetInstObjectAndSensorInfo::Parse(const rapidjson::Value& pt)
{
    BOOST_ASSERT(pt.IsObject() && pt.HasMember("output"));
    const rapidjson::Value& output = pt["output"];

    mrGeometryInfos.clear();

    const rapidjson::Value& instobjects = output["instobjects"];
    for (rapidjson::Document::ConstMemberIterator it = instobjects.MemberBegin(); it != instobjects.MemberEnd(); it++) {
        std::string objname = it->name.GetString();
        Transform transform;
        ResultOBB resultobb, resultinnerobb;
        LoadJsonValueByKey(it->value, "translation", transform.translate);
        LoadJsonValueByKey(it->value, "quaternion", transform.quaternion);
        resultobb.Parse(it->value["obb"]);
        resultinnerobb.Parse(it->value["innerobb"]);

        minstobjecttransform[objname] = transform;
        minstobjectobb[objname] = resultobb;
        minstobjectinnerobb[objname] = resultinnerobb;

        if( it->value.HasMember("geometryInfos") ) {
            boost::shared_ptr<rapidjson::Document> pr(new rapidjson::Document());;
            mujinjson::SaveJsonValue(*pr, it->value["geometryInfos"]);
            mrGeometryInfos[objname] = pr;
        }

        LoadJsonValueByKey(it->value, "uri", muri[objname]);
    }

    const rapidjson::Value& sensors = output["sensors"];
    for (rapidjson::Document::ConstMemberIterator it = sensors.MemberBegin(); it != sensors.MemberEnd(); it++) {
        mujin::SensorSelectionInfo sensorSelectionInfo;
        LoadJsonValue(it->name, sensorSelectionInfo.sensorName);
        for (rapidjson::Document::ConstMemberIterator itlink = it->value.MemberBegin(); itlink != it->value.MemberEnd(); itlink++) {
            LoadJsonValue(itlink->name, sensorSelectionInfo.sensorLinkName);
            Transform transform;
            SensorData sensordata;
            LoadJsonValueByKey(itlink->value, "translation", transform.translate);
            LoadJsonValueByKey(itlink->value, "quaternion", transform.quaternion);

            const rapidjson::Value &sensor = itlink->value["sensordata"];
            LoadJsonValueByKey(sensor, "distortion_coeffs", sensordata.distortion_coeffs);
            LoadJsonValueByKey(sensor, "intrinsic", sensordata.intrinsic);

            std::vector<int> imagedimensions;
            LoadJsonValueByKey(sensor, "image_dimensions", imagedimensions);
            if (imagedimensions.size() == 2) {
                imagedimensions.push_back(1);
            }
            if (imagedimensions.size() != 3) {
                throw MujinException("the length of image_dimensions is invalid", MEC_Failed);
            }
            for (int i = 0; i < 3; i++) {
                sensordata.image_dimensions[i] = imagedimensions[i];
            }

            LoadJsonValueByKey(sensor, "extra_parameters", sensordata.extra_parameters);
            LoadJsonValueByKey(sensor, "distortion_model", sensordata.distortion_model);
            LoadJsonValueByKey(sensor, "focal_length", sensordata.focal_length);
            LoadJsonValueByKey(sensor, "measurement_time", sensordata.measurement_time);

            msensortransform[sensorSelectionInfo] = transform;
            msensordata[sensorSelectionInfo] = sensordata;
        }
    }
}

MujinPlanningClient::MujinPlanningClient(
    /// HACK until can think of proper way to send sceneparams
    const std::string& scenebasename,
    const std::string& tasktype,
    const std::string& baseuri,
    const std::string& userName)
    : _zmqPort(-1),
      _heartbeatPort(-1),
      _tasktype(tasktype),
      _bIsInitialized(false)
{
    _callerid = str(boost::format("planningclientcpp%s_zmq")%MUJINPLANNINGCLIENT_VERSION_STRING);
    // get hostname from uri
    std::string::const_iterator uriend = baseuri.end();
    // query start
    std::string::const_iterator querystart = std::find(baseuri.begin(), uriend, '?');
    // protocol
    std::string protocol;
    std::string::const_iterator protocolstart = baseuri.begin();
    std::string::const_iterator protocolend = std::find(protocolstart, uriend, ':');
    if (protocolend != uriend) {
        std::string p = &*(protocolend);
        if ((p.length() > 3) & (p.substr(0,3) == "://")) {
            protocol = std::string(protocolstart, protocolend);
            protocolend +=3;
        } else {
            protocolend = baseuri.begin();
        }
    } else {
        protocolend = baseuri.begin();
    }
    // host
    std::string::const_iterator hoststart = protocolend;
    std::string::const_iterator pathstart = std::find(hoststart, uriend, '/');
    std::string::const_iterator hostend = std::find(protocolend, (pathstart != uriend) ? pathstart : querystart, ':');
    _mujinControllerIp = std::string(hoststart, hostend);

    {

        _rSceneParams.SetObject();
        mujinjson::SetJsonValueByKey(_rSceneParams, "scenetype", "mujin");
        mujinjson::SetJsonValueByKey(_rSceneParams, "sceneuri", std::string("mujin:/")+scenebasename);

        // should stop sending scenefilename since it is a hack!
        std::string MUJIN_MEDIA_ROOT_DIR = "/var/www/media/u";
        char* pMUJIN_MEDIA_ROOT_DIR = getenv("MUJIN_MEDIA_ROOT_DIR");
        if( !!pMUJIN_MEDIA_ROOT_DIR ) {
            MUJIN_MEDIA_ROOT_DIR = pMUJIN_MEDIA_ROOT_DIR;
        }

        std::string scenefilename = MUJIN_MEDIA_ROOT_DIR + std::string("/") + userName + std::string("/") + scenebasename;
        mujinjson::SetJsonValueByKey(_rSceneParams, "scenefilename", scenefilename);
        _sceneparams_json = mujinjson::DumpJson(_rSceneParams);
    }
}

MujinPlanningClient::~MujinPlanningClient()
{
    _bShutdownHeartbeatMonitor = true;
    if (!!_pHeartbeatMonitorThread) {
        _pHeartbeatMonitorThread->join();
    }
}

void MujinPlanningClient::Initialize(const std::string& defaultTaskParameters, const int zmqPort, const int heartbeatPort, boost::shared_ptr<zmq::context_t> zmqcontext, const bool initializezmq, const double reinitializetimeout, const double timeout, const std::string& userinfo, const std::string& slaverequestid)
{

    if( defaultTaskParameters.size() > 0 ) {
        _mapTaskParameters.clear();
        rapidjson::Document d;
        d.Parse(defaultTaskParameters.c_str());
        for (rapidjson::Value::ConstMemberIterator it = d.MemberBegin(); it != d.MemberEnd(); ++it) {
            rapidjson::StringBuffer stringbuffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(stringbuffer);
            it->value.Accept(writer);
            _mapTaskParameters[it->name.GetString()] = std::string(stringbuffer.GetString(), stringbuffer.GetSize());
        }
    }

    _zmqPort = zmqPort;
    _heartbeatPort = heartbeatPort;
    _bIsInitialized = true;
    _zmqcontext = zmqcontext;
    ParseJson(_rUserInfo, userinfo);
    _userinfo_json = userinfo;
    _slaverequestid = slaverequestid;

    InitializeZMQ(reinitializetimeout, timeout);
}

void MujinPlanningClient::SetCallerId(const std::string& callerid)
{
    _callerid = callerid;
}

void MujinPlanningClient::ExecuteCommand(rapidjson::Value& rTaskParameters, rapidjson::Document& rOutput, const double timeout)
{
    rapidjson::Document rCommand; rCommand.SetObject();
    mujinjson::SetJsonValueByKey(rCommand, "fnname", _tasktype == "binpicking" ? "binpicking.RunCommand" : "RunCommand");
    mujinjson::SetJsonValueByKey(rCommand, "stamp", GetMilliTime()*1e-3);
    mujinjson::SetJsonValueByKey(rCommand, "callerid", _GetCallerId());

    {
        rapidjson::Value rTaskParams; rTaskParams.SetObject();
        mujinjson::SetJsonValueByKey(rTaskParams, "tasktype", _tasktype, rCommand.GetAllocator());
        rTaskParams.AddMember(rapidjson::Document::StringRefType("taskparameters"), rTaskParameters, rCommand.GetAllocator());

        {
            rapidjson::Value rSceneParams;
            rSceneParams.CopyFrom(_rSceneParams, rCommand.GetAllocator());
            rTaskParams.AddMember(rapidjson::Document::StringRefType("sceneparams"), rSceneParams, rCommand.GetAllocator());
        }
        rCommand.AddMember(rapidjson::Document::StringRefType("taskparams"), rTaskParams, rCommand.GetAllocator());
    }

    {
        rapidjson::Value rUserInfo;
        rUserInfo.CopyFrom(_rUserInfo, rCommand.GetAllocator());
        rCommand.AddMember(rapidjson::Document::StringRefType("userinfo"), rUserInfo, rCommand.GetAllocator());
    }

    if (!_slaverequestid.empty()) {
        mujinjson::SetJsonValueByKey(rCommand, "slaverequestid", _slaverequestid);
    }

    try {
        _ExecuteCommandZMQ(mujinjson::DumpJson(rCommand), rOutput, timeout);
    }
    catch (const MujinException& e) {
        MUJIN_LOG_ERROR(e.what());
        if (e.GetCode() == MEC_Timeout) {
            _LogTaskParametersAndThrow(mujinjson::DumpJson(rCommand["taskparams"]));
        }
        else {
            throw;
        }
    }
}

void MujinPlanningClient::ExecuteCommand(const std::string& taskparameters, rapidjson::Document& rResult, const double timeout, const bool getresult)
{
    std::stringstream ss; ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss << "{\"fnname\": \"";
    ss << (_tasktype == "binpicking" ? "binpicking.RunCommand\", " : "RunCommand\", ");

    ss << "\"stamp\": " << (GetMilliTime()*1e-3) << ", ";
    ss << "\"callerid\": \"" << _GetCallerId() << "\", ";
    ss << "\"taskparams\": {\"tasktype\": \"" << _tasktype << "\", ";

    ss << "\"taskparameters\": " << taskparameters << ", ";
    ss << "\"sceneparams\": " << _sceneparams_json << "}, ";
    ss << "\"userinfo\": " << _userinfo_json;
    if (_slaverequestid != "") {
        ss << ", " << GetJsonString("slaverequestid", _slaverequestid);
    }
    ss << "}";
    std::string result_ss;

    try{
        _ExecuteCommandZMQ(ss.str(), rResult, timeout, getresult);
    }
    catch (const MujinException& e) {
        MUJIN_LOG_ERROR(e.what());
        if (e.GetCode() == MEC_Timeout) {
            _LogTaskParametersAndThrow(taskparameters);
        }
        else {
            throw;
        }
    }
}

void MujinPlanningClient::_ExecuteCommandZMQ(const std::string& command, rapidjson::Document& rOutput, const double timeout, const bool getresult)
{
    if (!_bIsInitialized) {
        throw MujinException("BinPicking task is not initialized, please call Initialzie() first.", MEC_Failed);
    }

    std::string result_ss;
    try {
        result_ss = _CallZMQ(command, timeout);
    }
    catch (const MujinException& e) {
        MUJIN_LOG_ERROR(e.what());
        if (e.GetCode() == MEC_ZMQNoResponse) {
            MUJIN_LOG_INFO("reinitializing zmq connection with the slave");
        } else if (e.GetCode() == MEC_Timeout) {
            throw MujinException("");  // Filled by `ExecuteCommand` callers who can access taskparameters more easily
        }
        else {
            throw;
        }
    }

    try {
        ParseJson(rOutput, result_ss);
    }
    catch(const std::exception& ex) {
        MUJIN_LOG_ERROR(str(boost::format("Could not parse result %s")%result_ss));
        throw;
    }
    if( rOutput.IsObject() && rOutput.HasMember("error")) {
        std::string error = GetJsonValueByKey<std::string>(rOutput["error"], "errorcode");
        std::string description = GetJsonValueByKey<std::string>(rOutput["error"], "description");
        if ( error.size() > 0 ) {
            std::string serror;
            if ( description.size() > 0 ) {
                serror = description;
            }
            else {
                serror = error;
            }
            if( serror.size() > 1000 ) {
                MUJIN_LOG_ERROR(str(boost::format("truncated original error message from %d")%serror.size()));
                serror = serror.substr(0,1000);
            }
            throw MujinException(str(boost::format("Error when calling binpicking.RunCommand: %s")%serror), MEC_BinPickingError);
        }
    }
}

void MujinPlanningClient::GetInnerEmptyRegionOBB(ResultOBB& result, const std::string& targetname, const std::string& linkname, const std::string& unit, const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "GetInnerEmptyRegionOBB";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("targetname", targetname) << ", ";
    if (linkname != "") {
        _ss << GetJsonString("linkname", linkname) << ", ";
    }
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    result.Parse(pt);
}

void MujinPlanningClient::UpdateObjects(const std::string& objectname, const std::vector<DetectedObject>& detectedobjects, const std::string& resultstate, const std::string& unit, const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    static const std::string command = "UpdateObjects";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("objectname", objectname) << ", ";
    _ss << GetJsonString("envstate") << ": [";
    for (unsigned int i=0; i<detectedobjects.size(); i++) {
        _ss << GetJsonString(detectedobjects[i]);
        if (i!=detectedobjects.size()-1) {
            _ss << ", ";
        }
    }
    _ss << "], ";
    if (resultstate.size() == 0) {
        _ss << GetJsonString("detectionResultState") << ": {}, ";
    }
    else {
        _ss << GetJsonString("detectionResultState") << ": " << resultstate << ", ";
    }
    _ss << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document d;
    ExecuteCommand(_ss.str(), d, timeout); // need to check return code
}

void MujinPlanningClient::InitializeZMQ(const double reinitializetimeout, const double timeout)
{
    if (!_pHeartbeatMonitorThread) {
        _bShutdownHeartbeatMonitor = false;
        if ( reinitializetimeout > 0) {
            _pHeartbeatMonitorThread.reset(new boost::thread(boost::bind(&MujinPlanningClient::_HeartbeatMonitorThread, this, reinitializetimeout, timeout)));
        }
    }
}

void MujinPlanningClient::GetTransform(const std::string& targetname, Transform& transform, const std::string& unit, const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "GetTransform";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("targetname", targetname) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    BOOST_ASSERT(pt.IsObject() && pt.HasMember("output"));
    const rapidjson::Value& v = pt["output"];
    LoadJsonValueByKey(v, "translation", transform.translate);
    LoadJsonValueByKey(v, "quaternion", transform.quaternion);
}


void MujinPlanningClient::AddPointCloudObstacle(const std::vector<float>&vpoints, const Real pointsize, const std::string& name,  const unsigned long long starttimestamp, const unsigned long long endtimestamp, const bool executionverification, const std::string& unit, int isoccluded, const std::string& locationName, const double timeout, bool clampToContainer, CropContainerMarginsXYZXYZPtr pCropContainerMargins, AddPointOffsetInfoPtr pAddPointOffsetInfo)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "AddPointCloudObstacle";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("isoccluded", isoccluded) << ", ";
    _ss << GetJsonString("locationName", locationName) << ", ";
    _ss << GetJsonString("clampToContainer", clampToContainer) << ", ";
    if( !!pCropContainerMargins ) {
        _ss << "\"cropContainerMarginsXYZXYZ\":[" << pCropContainerMargins->minMargins[0] << ", " << pCropContainerMargins->minMargins[1] << ", " << pCropContainerMargins->minMargins[2] << ", " << pCropContainerMargins->maxMargins[0] << ", " << pCropContainerMargins->maxMargins[1] << ", " << pCropContainerMargins->maxMargins[2] << "], ";
    }
    if( !!pAddPointOffsetInfo ) {
        _ss << "\"addPointOffsetInfo\":{";
        _ss << "\"zOffsetAtBottom\": " << pAddPointOffsetInfo->zOffsetAtBottom << ", ";
        _ss << "\"zOffsetAtTop\": " << pAddPointOffsetInfo->zOffsetAtTop << ", ";
        _ss << "\"use\": true";
        _ss << "}, ";
    }
    _ss << std::setprecision(std::numeric_limits<float>::digits10+1); // want to control the size of the JSON file outputted
    // "\"name\": __dynamicobstacle__, \"pointsize\": 0.005, \"points\": []
    _ss << GetJsonString("pointcloudid") << ": " << GetJsonString(name) << ", ";
    _ss << GetJsonString("pointsize") << ": " << pointsize <<", ";

    _ss << GetJsonString("points") << ": " << "[";
    bool bwrite = false;
    for (unsigned int i = 0; i < vpoints.size(); i+=3) {
        // sometimes point clouds can have NaNs, although it's a bug on detectors sending bad point clouds, these points can usually be ignored.
        if (!isnan(vpoints[i])
            && !isnan(vpoints[i+1])
            && !isnan(vpoints[i+2]) ) {
            if( bwrite ) {
                _ss << ",";
            }
            _ss << vpoints[i] << "," << vpoints[i+1] << "," << vpoints[i+2];
            bwrite = true;
        }
    }
    _ss << "]";

    // send timestamp regardless of the pointcloud definition
    _ss << ", \"starttimestamp\": " << starttimestamp;
    _ss << ", \"endtimestamp\": " << endtimestamp;
    _ss << ", \"executionverification\": " << (int) executionverification;

    _ss << ", " << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document rResult;
    ExecuteCommand(_ss.str(), rResult, timeout); // need to check return code
}

void MujinPlanningClient::VisualizePointCloud(const std::vector<std::vector<float> >&pointslist, const Real pointsize, const std::vector<std::string>&names, const std::string& unit, const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "VisualizePointCloud";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("pointslist") << ": [";
    for (unsigned int i=0; i< pointslist.size(); i++) {
        _ss << GetJsonString(pointslist[i]);
        if (i<pointslist.size()-1) {
            _ss << ", ";
        }
    }
    _ss << "], ";
    _ss << GetJsonString("pointsize") << ": " << pointsize << ", ";
    _ss << GetJsonString("names") << ": [";
    for (unsigned int i=0; i< names.size(); i++) {
        _ss << GetJsonString(names[i]);
        if (i<names.size()-1) {
            _ss << ", ";
        }
    }
    _ss << "]" << ", ";
    _ss << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document d(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(),d, timeout); // need to check return code
}

void MujinPlanningClient::ClearVisualization(const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "ClearVisualization";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << "}";
    rapidjson::Document d;
    ExecuteCommand(_ss.str(), d, timeout); // need to check return code
}

void MujinPlanningClient::IsRobotOccludingBody(const std::string& bodyname, const std::string& cameraname, const unsigned long long starttime, const unsigned long long endtime, bool& r, const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "IsRobotOccludingBody";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("bodyname") << ": " << GetJsonString(bodyname) << ", ";
    _ss << GetJsonString("cameraname") << ": " << GetJsonString(cameraname) << ", ";
    _ss << GetJsonString("starttime") << ": " << starttime <<", ";
    _ss << GetJsonString("endtime") << ": " << endtime;
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    BOOST_ASSERT(pt.IsObject() && pt.HasMember("output"));
    const rapidjson::Value& v = pt["output"];
    if (!v.IsObject() || !v.HasMember("occluded")) {
        throw MujinException("Output does not have \"occluded\" attribute!", MEC_Failed);
    }
    r = GetJsonValueByKey<int>(v, "occluded", 1) == 1;
}

void MujinPlanningClient::GetInstObjectAndSensorInfo(
    const std::vector<std::string>& instobjectnames,
    const std::vector<mujin::SensorSelectionInfo>& sensorSelectionInfos,
    ResultGetInstObjectAndSensorInfo& result,
    const std::string& unit,
    const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "GetInstObjectAndSensorInfo";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("tasktype", _tasktype) << ", ";
    _ss << GetJsonString("instobjectnames") << ": " << GetJsonString(instobjectnames) << ", ";
    _ss << GetJsonString("sensorSelectionInfos") << ": " << GetJsonString(sensorSelectionInfos) << ", ";
    _ss << GetJsonString("unit", unit);
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    try {
        result.Parse(pt);
    }
    catch(const std::exception& ex) {
        MUJIN_LOG_ERROR(str(boost::format("got error when parsing result. exception: %s result: %s")%ex.what()%DumpJson(pt)));
        throw;
    }
}

void MujinPlanningClient::GetInstObjectInfoFromURI(
    const std::string& objecturi,
    const Transform& instobjecttransform,
    ResultInstObjectInfo& result,
    const std::string& unit,
    const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    std::string command = "GetInstObjectInfoFromURI";
    _ss << GetJsonString("command", command) << ", ";
    _ss << GetJsonString("unit", unit) << ", ";
    _ss << "\"objecturi\":" << GetJsonString(objecturi) << ", ";
    _ss << "\"instobjectpose\":[";
    _ss << instobjecttransform.quaternion[0] << ", " << instobjecttransform.quaternion[1] << ", " << instobjecttransform.quaternion[2] << ", " << instobjecttransform.quaternion[3] << ", ";
    _ss << instobjecttransform.translate[0] << ", " << instobjecttransform.translate[1] << ", " << instobjecttransform.translate[2];
    _ss << "]";
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    try {
        result.Parse(pt);
    }
    catch(const std::exception& ex) {
        MUJIN_LOG_ERROR(str(boost::format("got error when parsing result: %s result: %s")%ex.what()%DumpJson(pt)));
        throw;
    }
}

void MujinPlanningClient::GetBinpickingState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout)
{
    SerializeGetStateCommand(_ss, _mapTaskParameters, "GetState", _tasktype, robotname, unit, timeout);
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    result.Parse(pt);
}

void MujinPlanningClient::GetITLState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout)
{
    SerializeGetStateCommand(_ss, _mapTaskParameters, "GetState", _tasktype, robotname, unit, timeout);
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
    result.Parse(pt);
}

void MujinPlanningClient::GetPublishedTaskState(ResultGetBinpickingState& result, const std::string& robotname, const std::string& unit, const double timeout)
{
    ResultGetBinpickingState taskstate;
    {
        boost::mutex::scoped_lock lock(_mutexTaskState);
        taskstate =_taskstate;
    }

    if (taskstate.timestamp == 0) {
        if (_tasktype == "binpicking") {
            MUJIN_LOG_INFO("Have not received published message yet, getting published state from GetBinpickingState()");
            GetBinpickingState(result, robotname, unit, timeout);
        }
        else {
            MUJIN_LOG_INFO("Have not received published message yet, getting published state from GetITLState()");
            GetITLState(result, robotname, unit, timeout);
        }
        {
            boost::mutex::scoped_lock lock(_mutexTaskState);
            _taskstate = result;
        }
    } else {
        result = taskstate;
    }
}

void MujinPlanningClient::SendMVRRegistrationResult(
    const rapidjson::Document &mvrResultInfo,
    double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    _ss << GetJsonString("command", "SendMVRRegistrationResult") << ", ";
    _ss << GetJsonString("mvrResultInfo", DumpJson(mvrResultInfo)) << ", ";
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
}

void MujinPlanningClient::SendRemoveObjectsFromObjectListResult(
    const std::vector<ResultGetBinpickingState::RemoveObjectFromObjectListInfo>& removeObjectFromObjectListInfos,
    const bool success,
    const double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    _ss << GetJsonString("command", "SendRemoveObjectsFromObjectListResult") << ", ";
    _ss << GetJsonString("objectPks") << ": [";
    for (size_t iInfo = 0; iInfo < removeObjectFromObjectListInfos.size(); ++iInfo) {
        _ss << GetJsonString(removeObjectFromObjectListInfos[iInfo].objectPk);
        if (iInfo != removeObjectFromObjectListInfos.size() - 1) {
            _ss << ", ";
        }
    }
    _ss << "], ";
    _ss << GetJsonString("success", success) << ", ";
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
}

void MujinPlanningClient::SendTriggerDetectionCaptureResult(const std::string& triggerType, const std::string& returnCode, double timeout)
{
    SetMapTaskParameters(_ss, _mapTaskParameters);
    _ss << GetJsonString("command", "SendTriggerDetectionCaptureResult") << ", ";
    _ss << GetJsonString("triggerType", triggerType) << ", ";
    _ss << GetJsonString("returnCode", returnCode) << ", ";
    _ss << "}";
    rapidjson::Document pt(rapidjson::kObjectType);
    ExecuteCommand(_ss.str(), pt, timeout);
}

const std::string& MujinPlanningClient::_GetCallerId() const
{
    return _callerid;
}

void MujinPlanningClient::_HeartbeatMonitorThread(const double reinitializetimeout, const double commandtimeout)
{
    MUJIN_LOG_DEBUG(str(boost::format("starting controller %s monitoring thread on port %d for slaverequestid=%s.")%_mujinControllerIp%_heartbeatPort%_slaverequestid));
    boost::shared_ptr<zmq::socket_t> socket;
    while (!_bShutdownHeartbeatMonitor) {
        if (!!socket) {
            socket->close();
            socket.reset();
        }
        socket = _CreateZMQSocket();

        zmq::pollitem_t pollitem;
        memset(&pollitem, 0, sizeof(zmq::pollitem_t));
        pollitem.socket = socket->operator void*();
        pollitem.events = ZMQ_POLLIN;
        unsigned long long lastheartbeat = GetMilliTime();
        while (!_bShutdownHeartbeatMonitor && (GetMilliTime() - lastheartbeat) / 1000.0f < reinitializetimeout) {
            zmq::poll(&pollitem,1, 50); // wait 50 ms for message
            if (pollitem.revents & ZMQ_POLLIN) {
                zmq::message_t reply;
                socket->recv(&reply);
                std::string replystring((char *)reply.data (), (size_t)reply.size());
                rapidjson::Document pt(rapidjson::kObjectType);
                try{
                    std::stringstream replystring_ss(replystring);
                    ParseJson(pt, replystring_ss.str());

                    std::string status;
                    LoadJsonValueByKey(pt, "status", status);
                    if (status != "lost" && status.size() > 1) {
                        lastheartbeat = GetMilliTime();
                    }
                    if (pt.HasMember("slavestates") && _slaverequestid.size() > 0) {
                        std::string key = "slaverequestid-" + _slaverequestid;
                        if (pt["slavestates"].HasMember(key.c_str())
                            && pt["slavestates"][key.c_str()].HasMember("taskstate")) {
                            MujinPlanningClient::ResultGetBinpickingState taskstate; 
                            taskstate.Parse(pt["slavestates"][key.c_str()]["taskstate"]);
                            {
                                boost::mutex::scoped_lock lock(_mutexTaskState);
                                _taskstate = taskstate;
                            }
                        }
                    }
                }
                catch (std::exception const &e) {
                    MUJIN_LOG_ERROR("HeartBeat reply is not JSON");
                    MUJIN_LOG_ERROR(replystring);
                    MUJIN_LOG_ERROR(e.what());
                    continue;
                }
            }
        }
        if (!_bShutdownHeartbeatMonitor) {
            std::stringstream sss; sss << std::setprecision(std::numeric_limits<double>::digits10+1);
            sss << (double)((GetMilliTime() - lastheartbeat)/1000.0f) << " seconds passed since last heartbeat signal, re-intializing ZMQ server.";
            MUJIN_LOG_INFO(sss.str());
        }
    }
    MUJIN_LOG_DEBUG(str(boost::format("Stopped controller %s monitoring thread on port %d for slaverequestid=%s.")%_mujinControllerIp%_heartbeatPort%_slaverequestid));
}

void MujinPlanningClient::_LogTaskParametersAndThrow(const std::string& taskparameters) {
    std::string errstr;
    if (taskparameters.size()>1000) {
        errstr = taskparameters.substr(0, 1000);
    } else {
        errstr = taskparameters;
    }
    throw MujinException(boost::str(boost::format("Timed out receiving response of command with taskparameters=%s...")%errstr));
}

boost::shared_ptr<zmq::socket_t> MujinPlanningClient::_CreateZMQSocket()
{
    auto socket = boost::make_shared<zmq::socket_t>((*_zmqcontext.get()),ZMQ_SUB);
    socket->setsockopt(ZMQ_TCP_KEEPALIVE, 1); // turn on tcp keepalive, do these configuration before connect
    socket->setsockopt(ZMQ_TCP_KEEPALIVE_IDLE, 2); // the interval between the last data packet sent (simple ACKs are not considered data) and the first keepalive probe; after the connection is marked to need keepalive, this counter is not used any further
    socket->setsockopt(ZMQ_TCP_KEEPALIVE_INTVL, 2); // the interval between subsequential keepalive probes, regardless of what the connection has exchanged in the meantime
    socket->setsockopt(ZMQ_TCP_KEEPALIVE_CNT, 2); // the number of unacknowledged probes to send before considering the connection dead and notifying the application layer
    std::stringstream ss; ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss << _heartbeatPort;
    socket->connect (("tcp://"+ _mujinControllerIp+":"+ss.str()).c_str());
    socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    return socket;
}

std::string MujinPlanningClient::_CallZMQ(const std::string& msg, const double timeout)
{
    boost::shared_ptr<zmq::socket_t> socket = _CreateZMQSocket();
    //send
    zmq::message_t request (msg.size());
    // std::cout << msg.size() << std::endl;
    // std::cout << msg << std::endl;
    memcpy ((void *) request.data (), msg.c_str(), msg.size());

    uint64_t starttime = GetMilliTime();
    bool recreatedonce = false;
    while (GetMilliTime() - starttime < timeout*1000.0) {
        try {
            socket->send(request);
            break;
        } catch (const zmq::error_t& e) {
            if (e.num() == EAGAIN) {
                MUJIN_LOG_ERROR("failed to send request, try again");
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                continue;
            } else {
                std::stringstream errss;
                errss << "failed to send msg: ";
                if (msg.length() > 1000) {
                    errss << msg.substr(0, 1000) << "...";
                } else {
                    errss << msg;
                }
                MUJIN_LOG_ERROR(errss.str());
            }
            if (!recreatedonce) {
                MUJIN_LOG_INFO("re-creating zmq socket and trying again");
                if (!!socket) {
                    socket->close();
                    socket.reset();
                }
                socket = _CreateZMQSocket();
                recreatedonce = true;
            } else{
                std::stringstream ss;
                ss << "Failed to send request after re-creating socket.";
                MUJIN_LOG_ERROR(ss.str());
                throw MujinException(ss.str(), MEC_Failed);
            }
        }
    }
    if (GetMilliTime() - starttime > timeout*1000.0) {
        std::stringstream ss;
        ss << "Timed out trying to send request.";
        MUJIN_LOG_ERROR(ss.str());
        if (msg.length() > 1000) {
            MUJIN_LOG_INFO(msg.substr(0,1000) << "...");
        } else {
            MUJIN_LOG_INFO(msg);
        }
        throw MujinException(ss.str(), MEC_Timeout);
    }
    //recv
    recreatedonce = false;
    zmq::message_t reply;
    bool receivedonce = false; // receive at least once
    while (!receivedonce || (GetMilliTime() - starttime < timeout * 1000.0)) {
        try {

            zmq::pollitem_t pollitem;
            memset(&pollitem, 0, sizeof(zmq::pollitem_t));
            pollitem.socket = socket->operator void*();
            pollitem.events = ZMQ_POLLIN;

            // if timeout param is 0, caller means infinite
            long timeoutms = -1;
            if (timeout > 0) {
                timeoutms = timeout * 1000.0;
            }

            zmq::poll(&pollitem, 1, timeoutms);
            receivedonce = true;
            if (pollitem.revents & ZMQ_POLLIN) {
                socket->recv(&reply);
                std::string replystring((char *) reply.data (), (size_t) reply.size());
                return replystring;
            } else{
                std::stringstream ss;
                if (msg.length() > 1000) {
                    ss << "Timed out receiving response of command " << msg.substr(0, 1000) << "... after " << timeout << " seconds";
                } else {
                    ss << "Timed out receiving response of command " << msg << " after " << timeout << " seconds";
                }
                MUJIN_LOG_ERROR(ss.str());
#if BOOST_VERSION > 104800
                std::string errstr = ss.str();
                boost::replace_all(errstr, "\"", ""); // need to remove " in the message so that json parser works
                boost::replace_all(errstr, "\\", ""); // need to remove \ in the message so that json parser works
#else
                std::vector<std::pair<std::string, std::string>> searchpairs(2);
                searchpairs[0].first = "\""; searchpairs[0].second = "";
                searchpairs[1].first = "\\"; searchpairs[1].second = "";
                std::string errstr;
                mujinclient::SearchAndReplace(errstr, ss.str(), searchpairs);
#endif
                throw MujinException(errstr, MEC_Timeout);
            }

        } catch (const zmq::error_t& e) {
            if (e.num() == EAGAIN) {
                MUJIN_LOG_ERROR("failed to receive reply, zmq::EAGAIN");
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                continue;
            } else {
                MUJIN_LOG_INFO("failed to send");
                if (msg.length() > 1000) {
                    MUJIN_LOG_INFO(msg.substr(0,1000) << "...");
                } else {
                    MUJIN_LOG_INFO(msg);
                }
            }
            if (!recreatedonce) {
                MUJIN_LOG_INFO("re-creating zmq socket and trying again");
                if (!!socket) {
                    socket->close();
                    socket.reset();
                }
                socket = _CreateZMQSocket();
                recreatedonce = true;
            } else{
                std::string errstr = "Failed to receive response after re-creating socket.";
                MUJIN_LOG_ERROR(errstr);
                throw MujinException(errstr, MEC_Failed);
            }
        }
    }
    if (GetMilliTime() - starttime > timeout*1000.0) {
        std::stringstream ss;
        ss << "timed out trying to receive request";
        MUJIN_LOG_ERROR(ss.str());
        if (msg.length() > 1000) {
            MUJIN_LOG_INFO(msg.substr(0,1000) << "...");
        } else {
            MUJIN_LOG_INFO(msg);
        }
        throw MujinException(ss.str(), MEC_Failed);
    }

    return "";
}

std::string utils::GetJsonString(const std::string& str)
{
    std::string newstr = str;
#if BOOST_VERSION > 104800
    boost::replace_all(newstr, "\"", "\\\"");
#else
    std::vector<std::pair<std::string, std::string>> searchpairs(1);
    searchpairs[0].first = "\"";
    searchpairs[0].second = "\\\"";
    SearchAndReplace(newstr, str, searchpairs);
#endif
    return "\""+newstr+"\"";
}

std::string utils::GetJsonString(const std::vector<float>& vec)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<float>::digits10+1);
    ss << "[";
    for (unsigned int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

std::string utils::GetJsonString(const std::vector<double>& vec)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss << "[";
    for (unsigned int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

std::string utils::GetJsonString(const std::vector<int>& vec)
{
    std::stringstream ss;
    ss << "[";
    for (unsigned int i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

std::string utils::GetJsonString(const std::vector<std::string>& vec)
{
    std::stringstream ss;
    ss << "[";
    for (unsigned int i = 0; i < vec.size(); ++i) {
        ss << GetJsonString(vec[i]);
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

std::string utils::GetJsonString(const Transform& transform)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<Real>::digits10+1);
    // \"translation\":[%.15f, %.15f, %.15f], \"quaternion\":[%.15f, %.15f, %.15f, %.15f]
    ss << GetJsonString("translation") << ": [";
    for (unsigned int i=0; i<3; i++) {
        ss << transform.translate[i];
        if (i!=3-1) {
            ss << ", ";
        }
    }
    ss << "], ";
    ss << GetJsonString("quaternion") << ": [";
    for (unsigned int i=0; i<4; i++) {
        ss << transform.quaternion[i];
        if (i!=4-1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

std::string utils::GetJsonString(const MujinPlanningClient::DetectedObject& obj)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<Real>::digits10+1);
    //"{\"name\": \"obj\",\"translation_\":[100,200,300],\"quat_\":[1,0,0,0],\"confidence\":0.5}"
    ss << "{";
    ss << GetJsonString("name") << ": " << GetJsonString(obj.name) << ", ";
    ss << GetJsonString("object_uri") << ": " << GetJsonString(obj.object_uri) << ", ";
    ss << GetJsonString("translation_") << ": [";
    for (unsigned int i=0; i<3; i++) {
        ss << obj.transform.translate[i];
        if (i!=3-1) {
            ss << ", ";
        }
    }
    ss << "], ";
    ss << GetJsonString("quat_") << ": [";
    for (unsigned int i=0; i<4; i++) {
        ss << obj.transform.quaternion[i];
        if (i!=4-1) {
            ss << ", ";
        }
    }
    ss << "], ";
    ss << GetJsonString("confidence") << ": " << obj.confidence;
    ss << ", " << GetJsonString("sensortimestamp") << ": " << obj.timestamp;
    ss << ", " << GetJsonString("isPickable") << ": " << obj.isPickable;
    if( obj.extra.size() > 0 ) {
        ss << ", " << GetJsonString("extra") << ": " << obj.extra;
    }
    ss << "}";
    return ss.str();
}

std::string utils::GetJsonString(const std::string& key, const std::string& value)
{
    std::stringstream ss;
    ss << GetJsonString(key) << ": " << GetJsonString(value);
    return ss.str();
}

std::string utils::GetJsonString(const std::string& key, const int value)
{
    std::stringstream ss;
    ss << GetJsonString(key) << ": " << value;
    return ss.str();
}

std::string utils::GetJsonString(const std::string& key, const unsigned long long value)
{
    std::stringstream ss;
    ss << GetJsonString(key) << ": " << value;
    return ss.str();
}

std::string utils::GetJsonString(const std::string& key, const Real value)
{
    std::stringstream ss;
    ss << GetJsonString(key) << ": " << value;
    return ss.str();
}


std::string utils::GetHeartbeat(const std::string& endpoint) {
    zmq::context_t zmqcontext(1);
    zmq::socket_t socket(zmqcontext, ZMQ_SUB);
    socket.connect(endpoint.c_str());
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    zmq::pollitem_t pollitem;
    memset(&pollitem, 0, sizeof(zmq::pollitem_t));
    pollitem.socket = socket;
    pollitem.events = ZMQ_POLLIN;

    zmq::poll(&pollitem,1, 50); // wait 50 ms for message
    if (!(pollitem.revents & ZMQ_POLLIN)) {
        return "";
    }

    zmq::message_t reply;
    socket.recv(&reply);
    const std::string received((char *)reply.data (), (size_t)reply.size());
#ifndef _WIN32
    return received;
#else
    // sometimes buffer can container \n or \\, which windows does not like
    std::string newbuffer;
    std::vector< std::pair<std::string, std::string> > serachpairs(2);
    serachpairs[0].first = "\n"; serachpairs[0].second = "";
    serachpairs[1].first = "\\"; serachpairs[1].second = "";
    SearchAndReplace(newbuffer, received, serachpairs);
    return newbuffer;
#endif
}


namespace {
std::string FindSmallestSlaveRequestId(const rapidjson::Value& pt) {
    // get all slave request ids
    std::vector<std::string> slavereqids;
    if (pt.IsObject() && pt.HasMember("slavestates")) {
        const rapidjson::Value& slavestates = pt["slavestates"];
        for (rapidjson::Document::ConstMemberIterator it = slavestates.MemberBegin(); it != slavestates.MemberEnd(); ++it) {
            static const std::string prefix("slaverequestid-");
            if (std::string(it->name.GetString()).substr(0,prefix.length()) == prefix) {
                // GetValueForSmallestSlaveRequestId uses slavereqid directly, so cannot substr here
                slavereqids.push_back(it->name.GetString());
            }
        }
    }

    // find numerically smallest suffix (find one with smallest ### in slave request id of form hostname_slave###)
    int smallest_suffix_index = -1;
    int smallest_suffix = INT_MAX;
    static const char searchstr('_');
    for (std::vector<std::string>::const_iterator it = slavereqids.begin();
         it != slavereqids.end(); ++it) {
        const size_t foundindex = it->rfind(searchstr);
        if (foundindex == std::string::npos) {
            continue;
        }

        if ((*it)[it->length()-1] < '0' || '9' < (*it)[it->length()-1]) {
            continue;
        }
        int suffix = 0;
        int po = 1;
        for (int i = it->length()-1; i >= 0 && '0' <= (*it)[i] && (*it)[i] <= '9'; i--, po *= 10) {
            suffix = suffix + ((*it)[i] - '0')*po;
        }

        if (smallest_suffix > suffix) {
            smallest_suffix = suffix;
            smallest_suffix_index = std::distance<std::vector<std::string>::const_iterator>(slavereqids.begin(), it);
        }
    }

    if (smallest_suffix_index == -1) {
        throw MujinException("Failed to find slave request id like hostname_slave### where ### is a number");
    }
    return slavereqids[smallest_suffix_index];
}

std::string GetValueForSmallestSlaveRequestId(const std::string& heartbeat,
                                              const std::string& key)
{

    rapidjson::Document pt(rapidjson::kObjectType);
    std::stringstream ss(heartbeat);
    ParseJson(pt, ss.str());
    try {
        const std::string slavereqid = FindSmallestSlaveRequestId(pt);
        std::string result;
        LoadJsonValue(pt["slavestates"][slavereqid.c_str()][key.c_str()], result);
        return result;
    }
    catch (const MujinException& ex) {
        throw MujinException(boost::str(boost::format("%s from heartbeat:\n%s")%ex.what()%heartbeat));
    }

}
}


std::string mujinplanningclient::utils::GetScenePkFromHeartbeat(const std::string& heartbeat) {
    static const std::string prefix("mujin:/");
    return GetValueForSmallestSlaveRequestId(heartbeat, "currentsceneuri").substr(prefix.length());
}

std::string utils::GetSlaveRequestIdFromHeartbeat(const std::string& heartbeat) {
    rapidjson::Document pt;
    std::stringstream ss(heartbeat);
    ParseJson(pt, ss.str());
    try {
        static const std::string prefix("slaverequestid-");
        return FindSmallestSlaveRequestId(pt).substr(prefix.length());
    }
    catch (const MujinException& ex) {
        throw MujinException(boost::str(boost::format("%s from heartbeat:\n%s")%ex.what()%heartbeat));
    }
}

} // end namespace mujinplanningclient
