/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Global includes
#include <iostream>
#include <string>
#include <set>
#include <vector>
#include <cmath>

//Pkg includes
#include <ros/ros.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <asr_msgs/AsrObject.h>

#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>

//ISM includes
#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/ObjectSet.hpp>
#include <ISM/common_type/RecordedPattern.hpp>

//Local includes
#include "ism_helper.hpp"

/**
 * Fake_data_publisher. Publishs data from dbfile so Recognizer has some data even when no cameras are available.
 * Wip.
 * @author Fabian Hanselmann
 * @version See SVN
 */
class Fake_data_publisher {
public:

    /**
     * Constructor processing parameters of scene recognition and setting up ros node.
     */
  Fake_data_publisher() : mNh("~") 
  {    
 
    std::string objectTopic;
    std::string dbfilename;
    std::string visualizationTopic;
    patternIter = 0;
    setIter = 0;
    usedPattern = -1;
    
    //Extract cli parameters of ros node for being used in this program.
    getNodeParameters(objectTopic, dbfilename, visualizationTopic);
    mTableHelper = ISM::TableHelperPtr (new ISM::TableHelper (dbfilename));
    sleep(1);
    mObjectSets = getDbEntries();

    int numberOfPatterns = mObjectSets.size();
    if (numberOfPatterns <= 0)
    {
        ROS_INFO_STREAM("Database doesn't contain any pattern!");
        exit(0);
    }

    if (mNh.hasParam("usedPattern")) {
        mNh.param<int>("usedPattern", usedPattern, 1);
        if (usedPattern < 1)
        {
            usedPattern = 1;
            ROS_INFO_STREAM("usedPattern is smaller than 1, setting it to 1 now.");
        }
        else if (usedPattern > numberOfPatterns)
        {
            usedPattern = numberOfPatterns;
            ROS_INFO_STREAM("usedPattern is greater than " << (numberOfPatterns) << ", setting it to " << numberOfPatterns << " now.");
        }
        int numberOfSets = mObjectSets[usedPattern-1].size();
        if (numberOfSets <= 0)
        {
            ROS_INFO_STREAM("Pattern doesn't contain any sets!");
            exit(0);
        }

        mNh.param<int>("firstUsedSet", firstUsedSet, 1);
        if (firstUsedSet < 1)
        {
            firstUsedSet = 1;
            ROS_INFO_STREAM("firstUsedSet is smaller than 1, setting it to 1 now.");
        }
        else if (firstUsedSet > numberOfSets)
        {
            firstUsedSet = numberOfSets ;
            ROS_INFO_STREAM("firstUsedSet is greater than numberOfSets(" << (numberOfSets) << "), setting it to " << firstUsedSet << " now.");
        }

        mNh.param<int>("lastUsedSet", lastUsedSet, numberOfSets);
        if (lastUsedSet < 1)
        {
            lastUsedSet = 1;
            ROS_INFO_STREAM("lastUsedSet is smaller than 1, setting it to 1 now.");
        }
        else if (lastUsedSet > numberOfSets)
        {
            lastUsedSet = numberOfSets;
            ROS_INFO_STREAM("lastUsedSet is greater than numberOfSets(" << numberOfSets << "), setting it to " << lastUsedSet << " now.");
        }

        if (firstUsedSet > lastUsedSet)
        {
            ROS_INFO_STREAM("firstUsedSet is greater than lastUsedSet!");
            exit(0);
        }

        std::cout << std::endl; //just for seperation of the terminal output
        ROS_INFO_STREAM("using pattern " << (usedPattern));
        ROS_INFO_STREAM("and sets from " << (firstUsedSet) << " to " << lastUsedSet << " (including last)");

        // sql tables start with 1 while std::vector starts with 0, so we reduce it by 1 and say it is a std::vector index/pos
        patternIter = --usedPattern;
        setIter = --firstUsedSet;
        lastUsedSet--;
    }

    mDataPub = mNh.advertise<asr_msgs::AsrObject> (objectTopic, 5, this);
    mVisualizationPub = mNh.advertise<visualization_msgs::MarkerArray>("fake_data_publisher_visualization", 100);
    mTimer = mNh.createTimer(ros::Duration(mPublishInterval), &Fake_data_publisher::timerCallback, this, false, true);
    mObjectModelVisualizer = new VIZ::ObjectModelVisualizerRVIZ(mVisualizationPub, mBaseFrame, "", 0.0);
  }
  virtual ~Fake_data_publisher(){};
  
  void getNodeParameters(std::string& pObjectTopic, std::string& pDbfilename,
			 std::string& pVisualizationTopic) 
  {
    if (!mNh.getParam("objectTopic", pObjectTopic))
    {
      pObjectTopic = "/objects";
    }
    ROS_INFO_STREAM("objectTopic: " << pObjectTopic);

    if (!mNh.getParam("capture_interval", mPublishInterval))
    {
      mPublishInterval = 1;
    }
    ROS_INFO_STREAM("PublishInterval (capture_interval): " << mPublishInterval);

    if (!mNh.getParam("dbfilename", pDbfilename))
    {
	  throw std::runtime_error("dbfilename not set");
    }
    ROS_INFO_STREAM("dbfilename: " << pDbfilename);

    if (!mNh.getParam("baseFrame", mBaseFrame))
    {
      mBaseFrame = "/map";
    }
    ROS_INFO_STREAM("baseFrame: " << mBaseFrame);

    if (!mNh.getParam("step", mStep))
    {
      mStep = 0.1;
    }
    ROS_INFO_STREAM("step: " << mStep);

    if (!mNh.getParam("visualization_topic", pVisualizationTopic)) 
    {
      pVisualizationTopic = "visualization_marker";
    }
    ROS_INFO_STREAM("visualization_topic: " << pVisualizationTopic);
  }
  
private:

  // type and id in an incoming object estimation.
  typedef std::pair<std::string, std::string> RecordKey;
  typedef std::map<RecordKey, const asr_msgs::AsrObject> RecordMapType;
  //Object estimations received within current time interval.
  RecordMapType mObjectRecord;

  IH::ObjectConverter* mConverter;

  //See launch file documentation
  double mPublishInterval;
  double mStep;
  std::string mBaseFrame;

  //Core functionality
  std::vector<std::vector<ISM::ObjectSetPtr>> mObjectSets;
  ISM::TableHelperPtr mTableHelper;
  ISM::Recognizer* mRecognizer;
  unsigned int patternIter;
  unsigned int setIter;
  // first and last set used
  int firstUsedSet;
  int lastUsedSet;
  int usedPattern;

  //Ros Node stuff
  ros::Publisher mVisualizationPub;
  ros::Timer mTimer;
  ros::Publisher mDataPub;
  ros::NodeHandle mNh;
  ros::Subscriber mSub;

  //Visualization stuff
  VIZ::ObjectModelVisualizerRVIZ* mObjectModelVisualizer;


  /*A method to get all "dbfilename" recorded_sets
   *
   *return value is a 3D-Vector with entries database[pattern][set]->objects[object]
   * (database [pattern][set] returns an object set, which can be accessed like a vector)
   */
  std::vector<std::vector<ISM::ObjectSetPtr>> getDbEntries ()
   {  
     ROS_INFO_STREAM("Extracting Patterns");
     std::vector<std::string> patternNames = mTableHelper->getRecordedPatternNames();
     ROS_INFO_STREAM("There are/is " << patternNames.size() << " pattern(s)");
     std::vector<ISM::RecordedPatternPtr> recordedPatterns (patternNames.size());
     std::vector<std::vector<ISM::ObjectSetPtr>> recordedSets (recordedPatterns.size());     
     for (unsigned int i = 0; i < patternNames.size(); ++i)
      {
	ROS_INFO_STREAM("Extracting pattern " << i + 1);
	recordedPatterns[i] = mTableHelper->getRecordedPattern(patternNames[i]);
      }
     for (unsigned int pattern = 0; pattern < recordedPatterns.size(); ++pattern)
      {
	recordedSets[pattern] = std::vector<ISM::ObjectSetPtr> (recordedPatterns[pattern]->objectSets.size());
	ROS_INFO_STREAM("Pattern " << pattern + 1 << " consists of " << recordedPatterns[pattern]->objectSets.size() 
			<< " sets ");
	for (unsigned int set = 0; set < recordedPatterns[pattern]->objectSets.size(); ++set)
	  {
	    recordedSets[pattern][set] = recordedPatterns[pattern]->objectSets[set];
	    ROS_INFO_STREAM("Set " << set + 1 << " of pattern " << pattern + 1 << " extracted"); 
	  }
      }
  return recordedSets;
  }

  asr_msgs::AsrObjectPtr objectToMessage(ISM::ObjectPtr object)
  {
    asr_msgs::AsrObjectPtr msg;
    msg.reset(new asr_msgs::AsrObject());

    //msg->header.stamp.secs = ros::Time::now().toSec();
    //msg->header.stamp.nsecs = ros::Time::now().toNSec();
    msg->header.frame_id = mBaseFrame;

    msg->providedBy = "fake_data_publisher";
    geometry_msgs::Pose fake_pose;
    fake_pose.position.x = object->pose->point->eigen.x();
    fake_pose.position.y = object->pose->point->eigen.y();
    fake_pose.position.z = object->pose->point->eigen.z();

    fake_pose.orientation.w = object->pose->quat->eigen.w();
    fake_pose.orientation.x = object->pose->quat->eigen.x();
    fake_pose.orientation.y = object->pose->quat->eigen.y();
    fake_pose.orientation.z = object->pose->quat->eigen.z();
    
    geometry_msgs::PoseWithCovariance fake_pose_with_c;
    fake_pose_with_c.pose = fake_pose;
    msg->sampledPoses.push_back(fake_pose_with_c);
    msg->type = object->type;
    msg->typeConfidence = object->confidence;
    msg->sizeConfidence = object->confidence;
    msg->identifier = object->observedId;
    msg->meshResourcePath = object->ressourcePath.string();

    /*for(unsigned int i = 0; i < msg->poseEstimation.covariance.size(); i++) 
      {
	msg->poseEstimation.covariance.at(i) = 0.0f;
      }
    */
    
    //object->boundingBox = get_bounding_box(entry.sOivFilePath);

	//object->type = entry.sName;
	//object->meshResourcePath = entry.sOivFilePath;
	/*
	// Segmentable Recognition:
	// quality: Verhältnis zwischen gemessener und simulierter Größe (unter Verwendung der berechneten Objektlage)
	// quality2: Korrelation der binären Objektsilhouetten (real, simuliert)
	// Beides sind Werte zwischen 0 und 1, wobei 1 maximale Übereinstimmung bedeutet.
	// Textured Recognition:
	// entry.quality: Mean Error after Homography estimation
	// entry.quality2: Number of Features after Homography estimation
	object->typeConfidence = entry.quality2;
	object->sizeConfidence = entry.quality;*/
    return msg;
  }

void timerCallback(const ros::TimerEvent& timerEvent)
{
    asr_msgs::AsrObjectPtr object;
    unsigned int setSize = mObjectSets[patternIter][setIter]->objects.size();
    unsigned int patterns = mObjectSets.size();
    unsigned int patternSize = mObjectSets[patternIter].size();
    ROS_INFO_STREAM("Publishing set " << setIter + 1 << " / " << patternSize << " of pattern "
                    << patternIter + 1 << " / " << patterns);
    ROS_INFO_STREAM("There are  " << setSize << " objects in this set");
    for (unsigned int objectIter = 0; objectIter < setSize; ++objectIter)
    {
        ROS_INFO_STREAM("Publishing object " << objectIter + 1 << " / " << setSize << " of set "
                        << setIter + 1);
        std::cout<<"info\n"<<std::endl;
        object = objectToMessage(mObjectSets[patternIter][setIter]->objects[objectIter]);
        mDataPub.publish(object);
    }

    mObjectModelVisualizer->drawObjectModels(mObjectSets[patternIter][setIter]->objects);

    setIter++;
    // we only use a part of all set
    if (setIter > (unsigned int) lastUsedSet)
    {
        setIter = firstUsedSet;
        ROS_INFO_STREAM("Pattern " << patternIter << " completed\n");
        ROS_INFO_STREAM("Done. Restarting.\n\n");
    }
}
};


int main(int argc, char **argv) 
{

  //Usual ros node stuff
  ros::init(argc, argv, "fake_data_publisher");

  Fake_data_publisher* publisher = new Fake_data_publisher();

  //Let this node run as long as we want objects to be published.
  ros::spin();

  delete publisher;
};





