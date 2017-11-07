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
#include <fstream>
#include <string>
#include <set>
#include <vector>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <sstream>

//Pkg includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <asr_msgs/AsrObject.h>

//ISM includes
#include <ISM/heuristic_trainer/Trainer.hpp>
#include <ISM/heuristic_trainer/DataCollector.hpp>
#include <ISM/heuristic_trainer/ManuallyDefPseudoHeuristic.hpp>
#include <ISM/utility/Util.hpp>

//Local includes
#include "ism_helper.hpp"

#define LINEWIDTH 0.008

bool changes;

/**
 * Trainer class. ROS Node wrapper for Trainer from libism.
 *
 * @author Reno Reckling, Pascal Meissner
 * @version See SVN
 */
class Trainer {
	typedef boost::filesystem::path path;
public:

    /**
     * Constructor processing parameters of training and setting up ros node.
     */
  Trainer() :
    mNh("~") {

    std::string dbfilename;
    bool useUserDefCluster;
    bool usePredefinedRefs;
    path preDefRefListFile;
    path clusterListFile;
    bool dropOldModelTables;
    
    //Extract cli parameters of ros node for being used in this program.
    getNodeParameters(dbfilename, useUserDefCluster, clusterListFile, dropOldModelTables,
    		usePredefinedRefs, preDefRefListFile);

    ISM::DataCollector::setCollect(true);
    if (dbfilename != "")
    {
        ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(dbfilename));
        if(!table_helper->recordDataExists())
        {
            ISM::printRed("The database \"" + dbfilename + "\" doesn't contain any recordings!\n");
            exit(0);
        }
        mTrainer = boost::shared_ptr<ISM::Trainer>(new ISM::Trainer(dbfilename, dropOldModelTables));
    }
    else
    {
        throw std::runtime_error("No db specified");
    }

    if(useUserDefCluster)
    {
        std::ifstream manualList(clusterListFile.string());
    	std::string line;
    	std::vector<std::pair<std::vector<ISM::ManuallyDefPseudoHeuristic::ClusterObject>,
		    uint16_t>> cluster;
    	while(std::getline(manualList, line))
    	{
            std::vector<ISM::ManuallyDefPseudoHeuristic::ClusterObject> clusterObjects;
    		std::vector<std::string> objAndId;
    		boost::split(objAndId, line, boost::is_any_of(";"));
    		uint16_t clusterId = boost::lexical_cast<uint16_t>(objAndId[1]);
    		std::vector<std::string> objSs;
    		boost::split(objSs, objAndId[0], boost::is_any_of(","));
    		for(std::string& objS : objSs)
    		{
    			uint16_t subId;
    			bool isSub = true;
    			try
    			{
    			  subId = boost::lexical_cast<int>(objS);
    			}
    			catch(boost::bad_lexical_cast& e)
    			{
    				isSub = false;
    			}
    			if(isSub)
    			{
    				clusterObjects.push_back(ISM::ManuallyDefPseudoHeuristic::ClusterObject(subId));
    			} else
    			{
    				clusterObjects.push_back(ISM::ManuallyDefPseudoHeuristic::ClusterObject(objS));
    			}
    		}
    		cluster.push_back(std::make_pair(clusterObjects, clusterId));
    	}
    	mTrainer->setClusterForManualDefHeuristic(cluster);
    }

    if(usePredefinedRefs)
    {
    	std::string line;
    	std::ifstream preDefRefS (preDefRefListFile.string());
    	std::map<std::string, std::string> preDefRefList;
    	while(std::getline(preDefRefS, line))
    	{
    		std::vector<std::string> patternType;
    		boost::split(patternType, line, boost::is_any_of(","));
    		preDefRefList[patternType[0]] = patternType[1];
    	}
    	mTrainer->setPredefinedRefs(preDefRefList);
    }


    mTrainer->staticBreakRatio = mStaticBreakRatio;
    mTrainer->togetherRatio = mTogetherRatio;
    mTrainer->maxAngleDeviation = mMaxAngleDeviation;

    mTrainer->setUseClustering(mUseClustering);
    
  }

  /**
   * Extracts parameters of already launched ros node and prints them on cli.
   *
   * @param pDbfilename Contains the ism table (i.e. scene models) as well as the table with the training data.   
   */
  void getNodeParameters(std::string& pDbfilename, bool& useUserDefCluster, path& clusterListFile,
		  bool& dropOldModelTables, bool& usePredefinedRefs, path& preDefRefListFile) {

    if (!mNh.getParam("dbfilename", pDbfilename)) {
      pDbfilename = "";
    }
    ROS_INFO_STREAM("dbfilename: " << pDbfilename);

    if (!mNh.getParam("baseFrame", mBaseFrame)) {
      mBaseFrame = "/map";
    }
    ROS_INFO_STREAM("baseFrame: " << mBaseFrame);

    if (!mNh.getParam("useClustering", mUseClustering)) {
      mUseClustering = true;
    }
    ROS_INFO_STREAM("useClustering: " << mUseClustering);

    if (!mNh.getParam("staticBreakRatio", mStaticBreakRatio)) {
      mStaticBreakRatio = 0.01;
    }
    ROS_INFO_STREAM("staticBreakRatio: " << mStaticBreakRatio);

    if (!mNh.getParam("togetherRatio", mTogetherRatio)) {
      mTogetherRatio = 0.90;
    }
    ROS_INFO_STREAM("togetherRatio: " << mTogetherRatio);

    if (!mNh.getParam("maxAngleDeviation", mMaxAngleDeviation)) {
      mMaxAngleDeviation = 45;
    }
    ROS_INFO_STREAM("maxAngleDeviation: " << mMaxAngleDeviation);

    if (!mNh.getParam("useUserDefCluster", useUserDefCluster)) {
        useUserDefCluster = false;
    }
    ROS_INFO_STREAM("useUserDefCluster: " << useUserDefCluster);
    if(useUserDefCluster)
    {
        std::string temp;
        if (!mNh.getParam("manualHeuristicListFile", temp)) {
          temp = "";
        } else
        {
                clusterListFile = temp;
        }
        if(!boost::filesystem::is_regular_file(clusterListFile))
        {
                std::stringstream ss;
                ss << clusterListFile << " does not exist or is not a file";
                ROS_ERROR_STREAM(ss.str());
                throw std::runtime_error(ss.str());
        }
        ROS_INFO_STREAM("clusterListFile: " << clusterListFile);
    }

    std::string temp;
    if (mNh.getParam("preDefRefListFile", temp))
    {
	    preDefRefListFile = temp;
	    if(boost::filesystem::is_regular_file(preDefRefListFile))
	    {
	    	usePredefinedRefs = true;
		    ROS_INFO_STREAM("preDefRefListFile" << preDefRefListFile);
	    }
    }
    else
      ROS_INFO_STREAM("preDefRefListFile: No file.");

    if (!mNh.getParam("dropOldModelTables", dropOldModelTables)) {
    	dropOldModelTables = false;
    }
    ROS_INFO_STREAM("dropOldModelTables: " << dropOldModelTables);
  
  }

  /**
   * Here isms are generated, based on recorded object configurations loaded 
   */
  void train() {
    this->mTrainer->trainPattern();
  }

private:

  //See launch file documentation
  std::string mBaseFrame;
  bool mUseClustering;
  double mStaticBreakRatio;
  double mTogetherRatio;
  double mMaxAngleDeviation;

  //Core functionality
  boost::shared_ptr<ISM::Trainer> mTrainer;

  //Ros Node Stuff  
  ros::NodeHandle mNh;

};

int main(int argc, char **argv) {

  //Usual ros node stuff
  ros::init(argc, argv, "trainer");

  Trainer* tr = new Trainer();
  tr->train();

  //Let this node run as long as we want training results being published to RViz.
  ros::spin();

};
