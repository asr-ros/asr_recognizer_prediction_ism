/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RECOGNIZER_PREDICTION_ISM_POSE_PREDICTION_NODE_H
#define RECOGNIZER_PREDICTION_ISM_POSE_PREDICTION_NODE_H

//ROS Includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <asr_recognizer_prediction_ism/pose_predictionConfig.h>

/* ----------------- Package messages/services  ------------------  */
#include "asr_recognizer_prediction_ism/GetPointCloud.h"

/* ----------------- Foreign messages/services  ------------------  */
#include <std_srvs/Empty.h>
#include <asr_msgs/AsrAttributedPointCloud.h>
#include <asr_msgs/AsrObject.h>
#include <asr_ism_visualizations/PosePredictionVisualizerRVIZ.hpp>
#include <asr_ism_visualizations/ism_result_visualizer_rviz.hpp>
#include <asr_world_model/GetFoundObjectList.h>
#include <pose_prediction_ism/pose_predictor.h>

/* ----------------- Local includes ------------------  */
#include "asr_recognizer_prediction_ism/resampler.h"

namespace recognizer_prediction_ism
{
class PosePrediction
{

public:

    //TODO: Comment
    PosePrediction(std::vector<ISM::RecognitionResultPtr>& results_already_in_shared, SharedRecognitionResultsManagerPtr shared_recognition_results_ptr);

    ~PosePrediction() { }

    /**
     * @brief RecognizerPredictionISM::processGetPointCloudServiceCall Callback function for a GetPointCloud Request
     *  The point cloud is empty if there is no result.
     *  In the debug mode this method publishes a point cloud of the object hypotheses
     * @param req Given GetPointCloud request
     * @param res The GetPointCloud response
     * @return Returns false, if the found object call is not available
     */
    bool processGetPointCloudServiceCall(asr_recognizer_prediction_ism::GetPointCloud::Request &req,
                                         asr_recognizer_prediction_ism::GetPointCloud::Response &res);
private:

    /* ----------------- Attributes for pose prediction ------------------  */
    pose_prediction_ism::PosePredictorPtr pose_predictor_ptr_;
    ResamplerPtr resampler_ptr_;
    double percentage_of_records_for_prediction_;
    double prediction_generation_factor_;
    std::string database_filename_;

    //Buffer shared with scene recognition that is filled every time we go back to direct search.
    std::vector<ISM::RecognitionResultPtr>& results_already_shared_;
    //Temporary buffer for scene recognition results until we fill mResultsAlreadyInSharedMem.
    std::vector<ISM::RecognitionResultPtr> results_buffer_;

    /* ----------------- Attributes for ROS ------------------  */
    ros::NodeHandle node_handle_;
    ros::ServiceServer point_cloud_server_;
    ros::ServiceServer toggle_visualization_server_;
    ros::ServiceServer toggle_pose_predictor_server_;

    ros::ServiceClient found_object_client_;

    ros::Publisher scene_markers_publisher_;
    ros::Publisher pose_prediction_markers_publisher_;

    dynamic_reconfigure::Server<asr_recognizer_prediction_ism::pose_predictionConfig>* reconfigure_server_;
    void dynamicReconfCallback(asr_recognizer_prediction_ism::pose_predictionConfig &config, uint32_t level);

    bool processToggleVisualizationServiceCall(std_srvs::Empty::Request &req, std_srvs::Empty::Response & res);
    bool processTogglePosePredictorServiceCall(std_srvs::Empty::Request &req, std_srvs::Empty::Response & res);

    /* ----------------- Visualization attributes------------------  */
    bool is_visualization_active_;
    VIZ::PosePredictionVisualizerPtr pose_prediction_visualizer_ptr_;
    VIZ::ISMResultVisualizerRVIZPtr res_visualizer_;

    void visualize(ISM::RecognitionResult recognition_result, char *result_specifier, int *i, unsigned int sampled_results_size, asr_msgs::AsrAttributedPointCloud *union_attributed_point_cloud, std::set<std::pair<std::string, std::string>> *predicted_objects);

    /* ----------------- Debug functions ------------------  */
    void checkPointCloud(asr_msgs::AsrAttributedPointCloud attributed_point_cloud);

};

typedef boost::shared_ptr<PosePrediction> PosePredictionPtr;

}
#endif
