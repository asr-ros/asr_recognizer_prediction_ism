/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Global includes
#include <Eigen/Geometry>

//ISM includes
#include "ISM/utility/GeometryHelper.hpp"

//local includes
#include "asr_recognizer_prediction_ism/ism_helper.h"

IH::ObjectConverter::ObjectConverter(const std::string& pBaseFrame, const bool pIgnoreTypes, const bool pIgnoreIds, const int pEnableRotationMode, const std::string pRotationFrame, const std::string pRotationObjType, const std::string pRotationObjId) : mBaseFrame(pBaseFrame),
    mIgnoreTypes(pIgnoreTypes), mIgnoreIds(pIgnoreIds), mEnableRotationMode(pEnableRotationMode), mRotationFrame(pRotationFrame), mRotationObjType(pRotationObjType), mRotationObjId(pRotationObjId), mUseConfidenceFromMsg(false),
    nodehandle(ros::this_node::getName())
{
    objectMetaDataClient = nodehandle.serviceClient<asr_object_database::ObjectMetaData>("/asr_object_database/object_meta_data");
}

IH::ObjectConverter::ObjectConverter(const std::string& pBaseFrame, const bool pIgnoreTypes, const bool pIgnoreIds, const int pEnableRotationMode, const std::string pRotationFrame, const std::string pRotationObjType, const std::string pRotationObjId, const bool pUseConfidenceFromMsg)
    : mBaseFrame(pBaseFrame), mIgnoreTypes(pIgnoreTypes), mIgnoreIds(pIgnoreIds), mEnableRotationMode(pEnableRotationMode), mRotationFrame(pRotationFrame), mRotationObjType(pRotationObjType), mRotationObjId(pRotationObjId), mUseConfidenceFromMsg(pUseConfidenceFromMsg),
      nodehandle(ros::this_node::getName())
{
    objectMetaDataClient = nodehandle.serviceClient<asr_object_database::ObjectMetaData>("/asr_object_database/object_meta_data");
}

void IH::ObjectConverter::normalizeRotationInvariantObjects(ISM::ObjectSetPtr objectSet) {
    if (mEnableRotationMode == ROTATION_MODE_DEACTIVATED) return;
    Eigen::Quaterniond orientationRef;
    Eigen::Vector3d yUnitVector; //
    // Search reference object in set
    if (mEnableRotationMode == ROTATION_MODE_OBJECT) {
     for (ISM::ObjectPtr object : objectSet->objects) {
        if (object->type == mRotationObjType && object->observedId == mRotationObjId) {
            mRotationRefObject = object;
            break;
        }
     }
     // return if object not found
     if (!mRotationRefObject) {
        ROS_INFO_STREAM("Orientation reference object: [" << mRotationObjType << ", " << mRotationObjId << "] is not set.");
        return;
     }
     orientationRef = ISM::GeometryHelper::quatToEigenQuat(mRotationRefObject->pose->quat);
     yUnitVector = Eigen::Vector3d::UnitY();
    } else if (mEnableRotationMode == ROTATION_MODE_FRAME) {
        if (mRotationFrame != mBaseFrame) {
            ROS_INFO_STREAM("Base frame: " << mBaseFrame << " is not equal to rotation frame: " << mRotationFrame << ". Normalization not possible.");
            return;
        }
        orientationRef = Eigen::Quaterniond(1.0, 0, 0, 0);
        yUnitVector = -Eigen::Vector3d::UnitZ(); // NOTE: in world coordinates system is the -Z-Axis equivalent to the Y-Axis in objects coordinates system.
    }
    orientationRef.normalize();

    for (ISM::ObjectPtr object : objectSet->objects) {
        asr_object_database::ObjectMetaData objMetaData;
        objMetaData.request.object_type = object->type;
        objMetaData.request.recognizer = object->providedBy == "fake_data_publisher" || "fake_object_recognition" ? "segmentable" : object->providedBy;
        if (objectMetaDataClient.call(objMetaData)) {
            if (objMetaData.response.is_rotation_invariant)
            {
                Eigen::Quaterniond orientation = ISM::GeometryHelper::quatToEigenQuat(object->pose->quat);
                orientation.normalize();
                Eigen::Vector3d objYAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientation), Eigen::Vector3d::UnitY());
                Eigen::Vector3d refYAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientationRef), yUnitVector);
                Eigen::AngleAxisd yTransform = calculateTransform(objYAxis, refYAxis, Eigen::Vector3d::UnitX());
                Eigen::Quaterniond yAlignedOrientation = (yTransform * orientation).normalized();
                Eigen::Vector3d objXAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(yAlignedOrientation), Eigen::Vector3d::UnitX());
                Eigen::Vector3d refXAxis = ISM::GeometryHelper::getAxisFromQuat(ISM::GeometryHelper::eigenQuatToQuat(orientationRef), Eigen::Vector3d::UnitX());
                object->pose->quat->eigen = (yTransform.inverse() * calculateTransform(objXAxis, refXAxis, Eigen::Vector3d::UnitY())).normalized() * yAlignedOrientation;
            }
        } else {
            ROS_ERROR("Could not call the asr_object_database::ObjectMetaData service call");
        }
    }
}

Eigen::AngleAxisd IH::ObjectConverter::calculateTransform(const Eigen::Vector3d& objectAxis, const Eigen::Vector3d& referenceAxis, const Eigen::Vector3d& unitAxis) {
    double angle = ISM::GeometryHelper::getAngleBetweenAxes(referenceAxis, objectAxis);
    Eigen::Vector3d rotationAxis = objectAxis.cross(referenceAxis);
    rotationAxis.normalize();

    angle *= M_PI/180; // Convert to radians
    if (angle < 0.001) {
      return Eigen::AngleAxisd(0, unitAxis);
    } else if (fabs(angle - M_PI) < 0.001) {
      return Eigen::AngleAxisd(M_PI, unitAxis);
    } else {
      return Eigen::AngleAxisd(angle, rotationAxis);
    }
}
