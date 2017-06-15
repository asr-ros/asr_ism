/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <thread>
#include "optedTrainerEvaluator.hpp"
#include "worker.hpp"
#include "boost/filesystem/path.hpp"

using path;
unsigned AMOUNT_OF_THREADS = 4;

#define ISM_DATA "/media/share/data"


void addVizData (map<string, map<string, unsigned> > & target, map<string, map<string, unsigned> >& source)
  {
    for (auto it : source)
      {
	for (auto voteIt : it.second)
	  {
	    target[it.first][voteIt.first] += voteIt.second;
	  }
      }    
  }
/*
void addVizData(map<string, vector<VotedPosePtr> >& target, map<string, vector<VotedPosePtr> >& source)
  {
    for (auto it : source)
      {
	target[it.first].insert(target[it.first].end(), it.second.begin(), it.second.end());
      }    
  }
*/
void drawISM(string model, string patternName, map<string, map<string, unsigned> >graphVizData, unsigned testsPerformed, unsigned timeSteps)
{
  if (testsPerformed == 0)
    {
      testsPerformed = 1;
    }
  ofstream file;
  stringstream filePath;
  filePath<<ISM_DATA<<patternName<<"_"<<model<<"_ISMWithVotes.dot";
  string filename = filePath.str();
  filePath.str("");
  vector<pair<string, string> > alreadyTakenCombinations;
  ios_base::iostate exceptionMask = file.exceptions() | ios::failbit | ios::badbit;
  file.exceptions(exceptionMask);
  //map<string, set<string> > referencesPerObject;
  try
    {
      file.open(filename);
      file<<"digraph "<<model<<" {\n";
      for (auto type : graphVizData)
	{
	  for (auto voteIt : type.second)
	    {
	      file<<voteIt.first<<"[shape=\"box\"];\n";
	      //referencesPerObject[type.first].insert(voteIt.first);
	    }
	}
      for (auto typeIt : graphVizData)
	{
	  for (auto voteIt : typeIt.second)
	    {
	      if (find(alreadyTakenCombinations.begin(), alreadyTakenCombinations.end(), make_pair(voteIt.first, typeIt.first)) == alreadyTakenCombinations.end())
		{
		  //		  cout<<"from "<<typeIt.first<<" to "<<voteIt.first<<": "<<voteIt.second<<endl;exit(0);
		  file<<voteIt.first<<" -> "<<typeIt.first<<"[ label=\" &#8746;"
		      <<(unsigned)((((double)voteIt.second) / testsPerformed) / timeSteps)<<"&#124;&#8594;"
		    //<<(unsigned)((((double)typeIt.second.size() / testsPerformed) / timeSteps) / referencesPerObject.at(typeIt.first).size())<<"&#124;&#8594;"
		      <<(unsigned)(((double)voteIt.second) / testsPerformed)
		    //<<(unsigned)(((double)typeIt.second.size() / testsPerformed) / referencesPerObject.at(typeIt.first).size())
		      <<"\", decorate=\"true\", labelfoat=\"true\", labelfontcolor=\"red\", dir=\"back\"];\n";
		  alreadyTakenCombinations.push_back(make_pair(voteIt.first, typeIt.first));
		}
	    }	  
	}
      file<<"}\n\n";
      file.flush();
      file.close();
    }
  catch (ios_base::failure& e)
    {
      cerr<<e.what()<<"\n";
      //exit(1);
    }
}


TestResult testModel(TestSpec testSpec, string modelDB)
{
  unsigned objectCount = testSpec.objectCount;
  unsigned timesteps = testSpec.timesteps;
  TestResult currentResult = {testSpec, 0, 0, 0, 0, 0, 0, 0};
  //map<string, vector<VotedPosePtr> > graphVizData;
  map<string, map<string, unsigned> > graphVizData;
  //struct timeval start, end;	  
  auto validSetsPath = std::string(ISM_DATA) + "/testDBs/testSets/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_validTestSets\n";
  TableHelperPtr validSetsTable(new TableHelper(validSetsPath));
  auto validPatterns = validSetsTable->getRecordedPattern("demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps));
  vector<ObjectSetPtr> validSets;
  if (validPatterns != 0)
    {
      validSets = validPatterns->objectSets;
    }
	  
  auto invalidSetsPath = std::string(ISM_DATA) + "/testDBs/testSets/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_invalidTestSets";
  TableHelperPtr invalidSetsTable(new TableHelper(invalidSetsPath));
  auto invalidPatterns = invalidSetsTable->getRecordedPattern("demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps));
  vector<ObjectSetPtr> invalidSets;
  if (invalidPatterns != 0)
    {
      invalidSets = invalidPatterns->objectSets;
    }
  vector<Worker*> workers;
  vector<shared_ptr<thread> > threads;
  //Recognizer is not thread safe
  const bool onlyPerfectResults = false;
  RecognizerPtr prototype(new Recognizer(modelDB, 0.03, 10, false));
  for (unsigned t_id = 0; t_id < AMOUNT_OF_THREADS; ++t_id)
    {
      Worker* worker = new Worker();
      RecognizerPtr threadRecognizer(new Recognizer(*prototype));
      shared_ptr<thread> oneThread(new thread(&Worker::test, worker, threadRecognizer, t_id, modelDB, validSets, invalidSets, testSpec));
      //We need the memory
      threadRecognizer.reset();
      threads.push_back(oneThread);
      workers.push_back(worker);
    }
  //We need the memory
  prototype.reset();
  for (unsigned t_id = 0; t_id < AMOUNT_OF_THREADS; ++t_id)
    {
      threads[t_id]->join();
      TestResult workerTestResult = workers[t_id]->getTestResult();
      currentResult.avgRuntimeForInvalidSets += workerTestResult.avgRuntimeForInvalidSets;
      currentResult.avgRuntimeForValidSets += workerTestResult.avgRuntimeForValidSets;
      currentResult.avgConfidenceForInvalidSets += workerTestResult.avgConfidenceForInvalidSets;
      currentResult.avgConfidenceForValidSets += workerTestResult.avgConfidenceForValidSets;
      currentResult.falsePositivesWithConfidence_1 += workerTestResult.falsePositivesWithConfidence_1;
      currentResult.falseNegativesWithConfidenceSmaller_1 += workerTestResult.falseNegativesWithConfidenceSmaller_1;
      auto workerVizData =  workers[t_id]->getVizData();
      addVizData(graphVizData, workerVizData);
      //Enjoy garbage collection, IF YOU HAVE ONE
      delete workers[t_id];
    }
  currentResult.avgRuntimeForInvalidSets /= (max((int)invalidSets.size(), 1));
  currentResult.avgRuntimeForValidSets /= (max((int)validSets.size(), 1));
  currentResult.avgConfidenceForInvalidSets /= (max((int)invalidSets.size(), 1));
  currentResult.avgConfidenceForValidSets /= (max((int)validSets.size(), 1));
  currentResult.falsePositivesWithConfidence_1 /= (max((int)invalidSets.size(), 1));
  currentResult.falseNegativesWithConfidenceSmaller_1 /= (max((int)validSets.size(), 1));
  drawISM(testSpec.usedModel, "demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps), graphVizData, validSets.size() + invalidSets.size(), timesteps);
  return currentResult;
}

void writeTestResult(const TestResult& r, ofstream& csvFile) {
  csvFile << r.testSpec.timesteps << "," << r.testSpec.objectCount << "," << r.testSpec.usedModel << "," << r.testSpec.useGreedyConstruction
	  << "," << r.avgRuntimeForValidSets << "," << r.avgConfidenceForValidSets << "," << r.avgRuntimeForInvalidSets << "," << r.avgConfidenceForInvalidSets
	  << ","<< r.falsePositivesWithConfidence_1 << "," << r.falseNegativesWithConfidenceSmaller_1 << "," << r.learnTime << endl;
}
void writeError(string error, ofstream& csvFile)
{
  csvFile<<error<<endl;
}

int main (int argc, char** argv)
{
  vector<unsigned> objectCounts = {3, 4, 5, 6, 7, 10, 15, 20};
  vector<unsigned> timestepCounts = {10, 15, 20, 30, 50, 100, 200};
  vector<string> models = 
    {
      "OptimizedModel", 
      //"OpimizedModel_naive", 
      "Heuristic", 
      "StarTopology", 
      //"FullyMeshed", 
      //"FullyMeshed_naive"
    };
  RandomDemoRecorder demoRec = *(new RandomDemoRecorder());
  double bin_size = 0.03;
  double maxAngleDeviation = 10;
  ofstream csvFile;
  csvFile.open(std::string(ISM_DATA) + "/evaluation/result-csvs/result.csv");
  csvFile << "timesteps,objectCount,usedModel,useGreedyConstruction,avgRuntimeForValidSets,avgConfideneForValidSets,avgRuntimForInvalidSets,avgConfidenceForInvalidSets,falsePositivesWithConfidence_1_in_%,falseNegativesWithConfidenceSmaller_1_in_%,learnTime" << endl;
  for (auto objectCount : objectCounts)
    {
      for (auto timesteps : timestepCounts)
	{
	  bool useGreedyConstruction = false;
	  if (objectCount > 5)
	    {
	      useGreedyConstruction = true;
	    }
	  auto demoRecordingPath = std::string(ISM_DATA) + "/testDBs/demo_recordings/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps);
	  TableHelperPtr recordingsTable(new TableHelper(demoRecordingPath));
	  recordingsTable->dropTables();
	  cout<<"inserting demo recording"<<endl;
	  demoRec.generateDemoRecording(demoRecordingPath, objectCount, timesteps);
	  cout<<"inserted demo recording"<<endl;
	  /*
    OptedTrainer(std::string dbfilename, path outputDataPath, bool useClassifier, double bin_size,
    		double maxAngleDeviation, unsigned falsePositiveTolerance, unsigned maxRelations,
    		bool useGreedyConstruction, bool useThreads,
    		bool usePredefinedSeedGenTestSets = false, unsigned int genTestSetsSeed = 0);
    		*/

      OptedTrainerPtr optimizedTrainer(new OptedTrainer(demoRecordingPath, path("."), false, bin_size, maxAngleDeviation, 0, 0, useGreedyConstruction, true));
	  struct timeval start, end;	  
	  cout<<"training for "<<objectCount<<" objects recorded over "<<timesteps<<" started"<<endl;
	  gettimeofday(&start, NULL);
	  optimizedTrainer->learn();
	  gettimeofday(&end, NULL);
	  cout<<"training for "<<objectCount<<" objects recorded over "<<timesteps<<" ended"<<endl;
	  recordingsTable->dropModelTables();
	  //We need the memory
	  optimizedTrainer.reset();
	  long learnTime = end.tv_sec - start.tv_sec;
	  for (auto model : models)
	    {
	      TestSpec currentTest = {useGreedyConstruction, objectCount, timesteps, model};
	      string modelDB;
	      if(model == "OptimizedModel")
		{ 
		  modelDB = std::string(ISM_DATA) + "/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_best"; 
		}
	      else if (model == "OpimizedModel_naive")
		{
		  if (timesteps > 100)
		    {
		      continue;
		    }
		  else if (objectCount > 5)
		    {
		      if (objectCount > 6 || timesteps > 50)
			{
			  continue;
			}
		    }
		  modelDB = std::string(ISM_DATA) + "/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_best_naive"; 
		}
	      else if (model == "Heuristic")
		{
		  TrainerPtr regularTrainer(new Trainer(demoRecordingPath));
		  regularTrainer->trainPattern();
		  modelDB = demoRecordingPath; 
		  //We need the memory
		  regularTrainer.reset();
		} 
	      else if (model == "StarTopology")
		{
		  modelDB = std::string(ISM_DATA) + "/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_star"; 
		}
	      else if (model == "FullyMeshed")
		{ 
		  modelDB = std::string(ISM_DATA) + "/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_fullyMeshed";
		}
	      else 
		{
		  if (timesteps > 100)
		    {
		      continue;
		    }
		  else if (objectCount > 5)
		    {
		      if (objectCount > 6 || timesteps > 50)
			{
			  continue;
			}
		    }
		  modelDB = std::string(ISM_DATA) + "/testDBs/topologies/record.sqlite.demo_recording_" + to_string(objectCount) + "_" + to_string(timesteps) + "_fullyMeshed_naive"; 
		}	
	      try
		{
		  TestResult currentResult = testModel(currentTest, modelDB);
		  currentResult.learnTime = learnTime;
		  writeTestResult(currentResult, csvFile);
		} 
	      catch (bad_alloc& ba)
		{
		  cerr<<ba.what();
		  writeError(to_string(timesteps) + "," + to_string(objectCount) + "," + model + "," + ba.what(), csvFile);
		}
	    }
	}
    }
  csvFile.close();
}
