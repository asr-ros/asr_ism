/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _ISM_HELPER_HPP_
#define _ISM_HELPER_HPP_

//Pkg includes
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <asr_msgs/AsrObject.h>
#include <asr_object_database/ObjectMetaData.h>

//Local includes
#include <ISM/recognizer/Recognizer.hpp>

/**
 * Namespace ism helper. Some functions needed in ros nodes interfacing libism.
 * 
 * @author Pascal Meissner
 * @version See SVN.
 */
namespace IH {
  
  /**
   * Converts of 7D position + quaternion from ros msg to ism internal datastructure.
   *
   * @param p 7D pose as ros msg.
   * @return 7D pose as ism datastructure.
   */
  ISM::PosePtr rosPoseToISMPose(const geometry_msgs::Pose p);

  /**
   * Namespace ism helper. Some functions needed in ros nodes interfacing libism.
   *
   * @author Pascal Meissner
   * @version See SVN
   */
  class ObjectConverter
  {
  public:

    /**
     * Constructor setting up transform_listener environment for transforming object messages to world coordinate frame. 
     *
     * @param pBaseFrame Frame to which incoming object messages are transformed.
     * @param pIgnoreTypes Create ism objects that have no type. 
     * @param pIgnoreIds Create ism objects that have no id.
     */
    ObjectConverter(const std::string& pBaseFrame, const bool pIgnoreTypes, const bool pIgnoreIds, const int pEnableRotationMode, const std::string pRotationFrame, const std::string pRotationObjType, const std::string pRotationObjId);
    ObjectConverter(const std::string& pBaseFrame, const bool pIgnoreTypes, const bool pIgnoreIds, const int pEnableRotationMode, const std::string pRotationFrame, const std::string pRotationObjType, const std::string pRotationObjId, const bool pUseConfidenceFromMsg);

    virtual ~ObjectConverter() {};

    /**
     * Extract class and identifier (within class) from estimation given by object localization system. Object 7D pose is extracted as well and transformed into world coordinate frame fixed during app startup.
     *
     * @param o ROS object msgs from which information is extracted.
     * @return ISM representation of class, id and pose being extracted.
     */
    ISM::ObjectPtr pbdObjectToISMObject(const asr_msgs::AsrObject& o);
    /**
     * @brief normalizeOrientation Normalizes the orientation of rotation invariance objects accroding to:
     *     - RotationFrame
     *     - or rotationObject
     * params in capturing.yaml.
     * @param objectSet accumlated objects to be processed.
     */
    void normalizeRotationInvariantObjects(ISM::ObjectSetPtr objectSet);
    
  private:
    //Some consts for orientation normalization.
    const int ROTATION_MODE_DEACTIVATED = 0;
    const int ROTATION_MODE_FRAME = 1;
    const int ROTATION_MODE_OBJECT = 2;

    ///Needed for transform to base_frame coordinate system.
    tf::TransformListener mTfListener;

    ///Frame to which incoming object messages are transformed.
    const std::string mBaseFrame;

    ///Whether to take over object types from AsrObjects.
    const bool mIgnoreTypes;
    ///Whether to take over object ids from AsrObjects.
    const bool mIgnoreIds;

    ///Normalize orientations of rotation invariant objects to either a coordinate frame or an not-rotation invariant object.
    const int mEnableRotationMode;

    ///If normalizing to frame: Which frame to normalize to.
    const std::string mRotationFrame;
    ///If normalizing to object: Which type of object to normalize to.
    const std::string mRotationObjType;
    ///If normalizing to object: Which id of object to normalize to.
    const std::string mRotationObjId;
    ///Current object pose, to which rotation invariant objects are normalized.
    ISM::ObjectPtr mRotationRefObject;

    ///Whether to take over recognition confidences (into ism rating) from AsrObjects.
    const bool mUseConfidenceFromMsg;

    ///Needed to store poses which have not yet been transformed
    std::vector<asr_msgs::AsrObject> transformBuffer;

    ros::NodeHandle nodehandle;

    ros::ServiceClient objectMetaDataClient;

    /**
     * @brief calculateTransform Caculate the transformation between two vectors.
     * @param objectAxis Object vector.
     * @param referenceAxis Reference vector.
     * @return
     */
    Eigen::AngleAxisd calculateTransform(const Eigen::Vector3d& objectAxis, const Eigen::Vector3d& referenceAxis, const Eigen::Vector3d& unitAxis);
  }; 

  typedef boost::shared_ptr<ObjectConverter> ObjectConverterPtr;

}

#endif /* _ISM_HELPER_HPP_ */
