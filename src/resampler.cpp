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
#include <list>
#include <map>
#include <utility>
#include <algorithm>

//Pkg includes
#include <ros/ros.h>

//Local includes
#include "asr_recognizer_prediction_ism/resampler.h"

using namespace std;
using namespace ISM;

namespace recognizer_prediction_ism
{

Resampler::Resampler(bool equal_results_number, unsigned int importance_resampled_size, SharedRecognitionResultsManagerPtr shared_recognition_results_ptr):
    EQUAL_RESULTS_NUMBER_(equal_results_number),
    IMPORTANCE_RESAMPLED_SIZE_(importance_resampled_size),
    shared_recognition_results_manager_ptr_(shared_recognition_results_ptr)
{
    ROS_DEBUG_STREAM_NAMED("Resampler", "Resampler::Resampler() is called.");
}

vector<RecognitionResult> Resampler::drawSamples()
{
    
    ROS_INFO("\n--------------------------------------------------\n");
    ROS_INFO("Resampler::drawSamples() is called.");

    const unsigned int SCENE_RECOGNITION_RESULT_NUMBER = shared_recognition_results_manager_ptr_->getResultsNumber();
    ROS_INFO_STREAM("Recognition results in shared memory before sampling: " << SCENE_RECOGNITION_RESULT_NUMBER);

    vector<RecognitionResultPtr> allResults = shared_recognition_results_manager_ptr_->getResults();

    map<string, unsigned int>::iterator resultIt;

    //TODO: Extract to own method

    //Count number of results in shared memory per scene
    map<string, unsigned int> patternToResultsNumber;

    for(RecognitionResultPtr result : allResults) {

        resultIt = patternToResultsNumber.find(result->patternName);

        if(resultIt != patternToResultsNumber.end())
            resultIt->second++;
        else
            patternToResultsNumber.insert(make_pair(result->patternName, 1));

    }

    for(pair<const string, unsigned int>& pair : patternToResultsNumber)
        ROS_INFO_STREAM(pair.second << " recognition results in shared memory for scene: " << pair.first);

    //TODO - End: Extract to own method

    //TODO: Extract to own helper method (without sum_of_confidence)

    //We assume that recognition results are sorted by their confidence in shared memory.
    list<RecognitionResult> results_for_sampling;
    list<RecognitionResult> surplus_results;
    double sum_of_confidence = 0.0;
    
    if(EQUAL_RESULTS_NUMBER_) {
        //Set mapping from present scenes to number of their results in shared memory to minimum number of results per scene.
        auto minIt = min_element(patternToResultsNumber.begin(), patternToResultsNumber.end(),
                                 [](decltype(patternToResultsNumber)::value_type& l, decltype(patternToResultsNumber)::value_type& r)
                -> bool { return l.second < r.second; });

        ROS_INFO_STREAM(minIt->second << " results per scene are considered as input for importance sampling.");
        ROS_INFO("The following results are considered as input for importance sampling: ");

        for(resultIt = patternToResultsNumber.begin(); resultIt != patternToResultsNumber.end(); resultIt++)
            resultIt->second = minIt->second;

        unsigned int num_results_for_sampling = minIt->second * patternToResultsNumber.size();

        while(results_for_sampling.size() < num_results_for_sampling) {

            RecognitionResult recognition_result = shared_recognition_results_manager_ptr_->getLastResult();

            if(patternToResultsNumber.at(recognition_result.patternName)){
                ROS_DEBUG_STREAM(recognition_result);
                sum_of_confidence += recognition_result.confidence;
                results_for_sampling.push_back(recognition_result);
                patternToResultsNumber.at(recognition_result.patternName)--;
            }
            else{
                surplus_results.push_back(recognition_result);
            }
            shared_recognition_results_manager_ptr_->popLastResult();

        }

    }
    //Otherwise all results from shared mem are taken at once.
    else {
        for (unsigned int i = 0; i < SCENE_RECOGNITION_RESULT_NUMBER; ++i)
        {
            RecognitionResult recognition_result = shared_recognition_results_manager_ptr_->getLastResult();
            shared_recognition_results_manager_ptr_->popLastResult();
            sum_of_confidence += recognition_result.confidence;
            results_for_sampling.push_back(recognition_result);
        }
        ROS_INFO("All results are considered as input for importance sampling.");
    }
    
    const int ACTUAL_SAMPLE_SIZE = results_for_sampling.size();

    //TODO - End: Extract to own method

    ROS_ASSERT(SCENE_RECOGNITION_RESULT_NUMBER == results_for_sampling.size() + surplus_results.size() + shared_recognition_results_manager_ptr_->getResultsNumber());

    //TODO: Extract to own helper method
    double accumulated_confidence_per_result[ACTUAL_SAMPLE_SIZE];
    list<RecognitionResult>::iterator results_for_sampling_it = results_for_sampling.begin();
    //Calculate accumulated confidences for following importance sampling.
    accumulated_confidence_per_result[0] = results_for_sampling_it->confidence / sum_of_confidence;
    for (int i = 1; i < ACTUAL_SAMPLE_SIZE; ++i)
    {
        results_for_sampling_it++;
        RecognitionResult recognition_result = (*results_for_sampling_it);
        accumulated_confidence_per_result[i] =
                (recognition_result.confidence / sum_of_confidence) +
                accumulated_confidence_per_result[i - 1];
    }
    if (accumulated_confidence_per_result[ACTUAL_SAMPLE_SIZE - 1] != 1.0)
        accumulated_confidence_per_result[ACTUAL_SAMPLE_SIZE - 1] = 1.0;

    ROS_INFO_STREAM("These are the recognition results from importance sampling.");

    vector<RecognitionResult> resampled_results;
    vector<list<RecognitionResult>::iterator> resultsToBeDeleted;

    //Perform the importance sampling
    for (unsigned int i = 0; i < IMPORTANCE_RESAMPLED_SIZE_; ++i)
    {
        double slot = rand() / double(RAND_MAX);
        int index = 0;
        results_for_sampling_it = results_for_sampling.begin();
        while (slot > accumulated_confidence_per_result[index])
        {
            index++;
            results_for_sampling_it++;
        }

        RecognitionResult recognition_result = *results_for_sampling_it;
        ROS_DEBUG_STREAM(recognition_result);
        resampled_results.push_back(recognition_result);

        if (std::find(resultsToBeDeleted.begin(), resultsToBeDeleted.end(), results_for_sampling_it) == resultsToBeDeleted.end()) {
            resultsToBeDeleted.push_back(results_for_sampling_it);
        }
    }

    ROS_INFO_STREAM("We sampled " << resampled_results.size() << " results.");

    //TODO: End

    //Remove all sampled recognition results from list of extracted best results.
    for (list<RecognitionResult>::iterator delIt : resultsToBeDeleted)
        results_for_sampling.erase(delIt);

    results_for_sampling.insert(results_for_sampling.end(), surplus_results.begin(), surplus_results.end());
    results_for_sampling.sort([](const RecognitionResult & a, const RecognitionResult & b) -> bool
    {
        return a.confidence > b.confidence;
    });
    vector<RecognitionResultPtr> remainingResults;

    //Append all remaining recognition results to shared memory.
    list<RecognitionResult>::reverse_iterator reverseResultIt;

    for(reverseResultIt = results_for_sampling.rbegin(); reverseResultIt != results_for_sampling.rend();
        reverseResultIt++)
        remainingResults.push_back(RecognitionResultPtr(new RecognitionResult(*reverseResultIt)));

    shared_recognition_results_manager_ptr_->addResults(remainingResults);

    ROS_INFO_STREAM(shared_recognition_results_manager_ptr_->getResultsNumber() << " in shared memory after resampling.");

    ROS_INFO("\n--------------------------------------------------\n");
    return resampled_results;

}

bool Resampler::recognitionResultsAvailable()
{
    return shared_recognition_results_manager_ptr_->recognitionResultsAvailable();
}

unsigned int Resampler::getImportanceResampledSize() const
{
    return IMPORTANCE_RESAMPLED_SIZE_;
}

}
