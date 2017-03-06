/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Local includes
#include "asr_recognizer_prediction_ism/shared_recognition_results_manager.h"
#include <ros/ros.h>

using namespace std;
using namespace ISM;

namespace recognizer_prediction_ism
{

bool SharedRecognitionResultsManager::addResults(std::vector<ISM::RecognitionResultPtr> &result_ptrs)
{
    mutex_.lock();
    shared_memory_vector_.insert(shared_memory_vector_.end(), result_ptrs.begin(), result_ptrs.end());
    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Added: " << result_ptrs.size() << " recognitionresults to shared memory, now containing "  << shared_memory_vector_.size() << " recognitionresults");
    mutex_.unlock();
    return true;
}

RecognitionResult SharedRecognitionResultsManager::getLastResult()
{
    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Returning last recognitionresults");
    return *shared_memory_vector_.back();
}

std::vector<RecognitionResultPtr> SharedRecognitionResultsManager::getResults()
{

    std::vector<RecognitionResultPtr> results;

    mutex_.lock();
    //This should may copy all Recognition Results!!!
    for (std::vector<RecognitionResultPtr>::iterator it = shared_memory_vector_.begin(); it != shared_memory_vector_.end(); it++)
        results.push_back(*it);

    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Returning all (" << shared_memory_vector_.size() << ") recognitionresults");

    mutex_.unlock();

    return results;

}

unsigned int SharedRecognitionResultsManager::getResultsNumber()
{
    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Returning size of shared memory");
    return shared_memory_vector_.size();
}

void SharedRecognitionResultsManager::clearResults()
{
    mutex_.lock();
    shared_memory_vector_.clear();
    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Cleared shared memory, now containing "  << shared_memory_vector_.size() << " recognitionresults");
    mutex_.unlock();
}

bool SharedRecognitionResultsManager::recognitionResultsAvailable()
{
    return !shared_memory_vector_.empty();
}

bool SharedRecognitionResultsManager::popLastResult()
{
    mutex_.lock();

    if (!shared_memory_vector_.empty())
        shared_memory_vector_.pop_back();
    else
        return false;
    mutex_.unlock();

    ROS_DEBUG_STREAM_NAMED("SharedMemory", "Popped last recognitionresult, now containing "  << shared_memory_vector_.size() << " recognitionresults");
    return true;
}
std::ostream& operator<<(std::ostream &strm, SharedRecognitionResultsManager &p){
    stringstream s;
    std::vector<RecognitionResultPtr> rr_vec = p.getResults();
    s << "[SharedMemory:size=" << rr_vec.size() <<";vector:[";
    for(RecognitionResultPtr r : rr_vec){
       s << r << ",";
    }
    s << "]]";
    return strm << s.str();
}

std::ostream& operator<<(std::ostream &strm, const SharedRecognitionResultsManagerPtr &pPtr){
    return strm << (*pPtr);
}
}
