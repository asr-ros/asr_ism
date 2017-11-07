/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#define BOOST_NO_SCOPED_ENUMS

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <chrono>

#include <ISM/combinatorial_trainer/CombinatorialTrainer.hpp>
#include <ISM/combinatorial_trainer/CombinatorialTrainerParameters.hpp>
#include <ISM/common_type/RecognitionResult.hpp>
#include <ISM/utility/Util.hpp>

using boost::filesystem::path;

class CombinatorialTrainer
{

private:
	ros::NodeHandle mNh;

public:
	CombinatorialTrainer() : mNh("~")
	{
		ISM::CombinatorialTrainerParameters params;
		getCombinatorialTrainerParameters(mNh, &params);

        ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(params.general.dbfilename));
        if(!table_helper->recordDataExists())
        {
            ISM::printRed("The database \"" + params.general.dbfilename + "\" doesn't contain any recordings!\n");
            exit(0);
        }


		ISM::CombinatorialTrainerPtr combinatorialTrainer = ISM::CombinatorialTrainerPtr(new ISM::CombinatorialTrainer(params));
		combinatorialTrainer->learn();
	}

	void getCombinatorialTrainerParameters(const ros::NodeHandle& node_handle, ISM::CombinatorialTrainerParameters* params)
	{
		readParameter(node_handle, "dbfilename", &params->general.dbfilename, "");
		readParameter(node_handle, "outputDataPath", &params->general.outputDataPath, "");
		readParameter(node_handle, "testSetCount", &params->general.testSetCount, 0u);
		readParameter(node_handle, "useClassifier", &params->general.useClassifier, false);
		readParameter(node_handle, "loadValidTestSetsFrom", &params->general.loadValidTestSetsFrom, "");
		readParameter(node_handle, "loadInvalidTestSetsFrom", &params->general.loadInvalidTestSetsFrom, "");
		readParameter(node_handle, "storeValidTestSetsTo", &params->general.storeValidTestSetsTo, "");
		readParameter(node_handle, "storeInvalidTestSetsTo", &params->general.storeInvalidTestSetsTo, "");
		readParameter(node_handle, "loadStartTopologiesFrom", &params->general.loadStartTopologiesFrom, "");
		readParameter(node_handle, "storeFullyMeshedISM", &params->general.storeFullyMeshedISM, false);
		readParameter(node_handle, "storeStartTopologyISM", &params->general.storeStartTopologyISM, false);

		readParameter(node_handle, "evaluatorId", &params->evaluator.evaluatorId, 0);
		readParameter(node_handle, "bin_size", &params->evaluator.binSize, 0.08);
		readParameter(node_handle, "maxAngleDeviation", &params->evaluator.maxAngleDeviation, 10);
		readParameter(node_handle, "confidenceThreshold", &params->evaluator.confidenceThreshold, 0.99);
		readParameter(node_handle, "testForFalseNegatives", &params->evaluator.testForFalseNegatives, false);

		readParameter(node_handle, "topologyGeneratorId", &params->topologyGenerator.topologyGeneratorId, 0);
		readParameter(node_handle, "swapRelations", &params->topologyGenerator.swapRelations, true);
		readParameter(node_handle, "removeRelations", &params->topologyGenerator.removeRelations, true);
		readParameter(node_handle, "maxNeighbourCount", &params->topologyGenerator.maxNeighbourCount, -1);

		readParameter(node_handle, "treeValidatorId", &params->treeValidator.treeValidatorId, 0);
		readParameter(node_handle, "maxTreeHeight", &params->treeValidator.maxTreeHeight, 2);

		readParameter(node_handle, "optimizationAlgorithmId", &params->optimizationAlgorithm.optimizationAlgorithmId, 0);
		readParameter(node_handle, "randomRestartProbability", &params->optimizationAlgorithm.randomRestartProbability, 0);
		readParameter(node_handle, "randomWalkProbability", &params->optimizationAlgorithm.randomWalkProbability, 0);
		readParameter(node_handle, "startFromRandomTopology", &params->optimizationAlgorithm.startFromRandomTopology, false);

		readParameter(node_handle, "startTemperature", &params->optimizationAlgorithm.startTemperature, 0);
		readParameter(node_handle, "endTemperature", &params->optimizationAlgorithm.endTemperature, 0);
		readParameter(node_handle, "temperatureFactor", &params->optimizationAlgorithm.temperatureFactor, 0);
		readParameter(node_handle, "repetitionsBeforeUpdated", &params->optimizationAlgorithm.repetitionsBeforeUpdated, 0);

		readParameter(node_handle, "initialAcceptableCostDelta", &params->optimizationAlgorithm.initialAcceptableCostDelta, 0);
		readParameter(node_handle, "costDeltaDecreaseFactor", &params->optimizationAlgorithm.costDeltaDecreaseFactor , 0);
		readParameter(node_handle, "costFunctionId", &params->costFunction.costFunctionId, 0);
		readParameter(node_handle, "alpha", &params->costFunction.alpha, 1);
        readParameter(node_handle, "beta", &params->costFunction.beta, 1);
	}

	void readParameter(const ros::NodeHandle& node_handle, const std::string& parameterName, double* d, const double defaultValue)
	{
		node_handle.param<double>(parameterName, *d, defaultValue);
		ROS_INFO("%s = %.4lf", parameterName.c_str(), *d);
	}

	void readParameter(const ros::NodeHandle& node_handle, const std::string& parameterName, std::string* s, const std::string defaultValue)
	{
		node_handle.param<std::string>(parameterName, *s, defaultValue);
		ROS_INFO("%s = %s", parameterName.c_str(), (*s).c_str());
	}

	void readParameter(const ros::NodeHandle& node_handle, const std::string& parameterName, int *i, const int defaultValue)
	{
		node_handle.param<int>(parameterName, *i, defaultValue);
		ROS_INFO("%s = %d", parameterName.c_str(), *i);
	}

	void readParameter(const ros::NodeHandle& node_handle, const std::string& parameterName, bool *b, const bool defaultValue)
	{
		node_handle.param<bool>(parameterName, *b, defaultValue);
		ROS_INFO("%s = %s", parameterName.c_str(), *b ? "true" : "false");
	}

	void readParameter(const ros::NodeHandle& node_handle, const std::string& parameterName, unsigned int *ui, const unsigned int defaultValue)
	{
		int tmp;
		node_handle.param<int>(parameterName, tmp, defaultValue);
		*ui = boost::lexical_cast<unsigned int>(tmp);
		ROS_INFO("%s = %u", parameterName.c_str(), *ui);
	}
};

int main(int argc, char **argv)
{
	auto unix_timestamp = std::chrono::seconds(std::time(NULL));
	std::cout << "Start Time: " << unix_timestamp.count() << std::endl;

	boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d-%b-%Y %H:%M:%S");
	std::cout.imbue(std::locale(std::cout.getloc(), facet));
	std::cout << boost::posix_time::second_clock::local_time() << std::endl;

	auto t1 = std::chrono::high_resolution_clock::now();

	ros::init(argc, argv, "combinatorialTrainer");
	CombinatorialTrainer* otr = new CombinatorialTrainer();

	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
	std::cout << "Duration: " << duration << std::endl;

	delete otr;
	delete facet;
}
