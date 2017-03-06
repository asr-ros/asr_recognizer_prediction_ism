/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ros/ros.h"
#include "asr_recognizer_prediction_ism/pose_prediction.h"
#include "asr_recognizer_prediction_ism/scene_recognition.h"
#include "asr_recognizer_prediction_ism/shared_recognition_results_manager.h"
#include <asr_recognizer_prediction_ism/rp_ism_nodeConfig.h>
#include <thread>
#include <signal.h>

using namespace std;
using namespace recognizer_prediction_ism;

void mySigintHandler(int sig)
{
    ROS_DEBUG_NAMED("rp_ism_node", "rp_ism_node: SHUTDOWNED");
    ros::shutdown();
}

//Here we create scene recognition and pose prediction, each in different threads and interacting via shared memory.
int main(int argc, char **argv)
{

    //*** ROS stuff ***
    ROS_INFO("Initializing ROS node for rp_ism_node");
    ros::init(argc, argv, "rp_ism_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n("~");
    signal(SIGINT, mySigintHandler);
    unsigned concurent_threads_supported = std::thread::hardware_concurrency();
    ROS_INFO_STREAM("RP_ISM: Number of Threads: " << concurent_threads_supported);
    ros::AsyncSpinner spinner(concurent_threads_supported); // Use dynamic # of threads

    //Buffer all scene recognition results that have been used to predict object poses. Prevents using them twice.
    std::vector<ISM::RecognitionResultPtr> resultsAlreadyInSharedMem;
    SharedRecognitionResultsManagerPtr shared_memory = SharedRecognitionResultsManagerPtr(new SharedRecognitionResultsManager());

    //*** Scene recognition and pose prediction stuff ***

    ROS_DEBUG_NAMED("rp_ism_node", "rp_ism_node: Initializing ism scene recognition.");
    SceneRecognition scene_recognition(resultsAlreadyInSharedMem, shared_memory);

    ROS_DEBUG_NAMED("rp_ism_node", "rp_ism_node: Initializing ism pose prediction.");
    PosePrediction pose_prediction(resultsAlreadyInSharedMem, shared_memory);

    //Please do not move that call from here and especially not to the loop. This is the random Seed for PosePrediction.
    srand (time(NULL));

    spinner.start();
    ros::waitForShutdown();
    return 0;

}
