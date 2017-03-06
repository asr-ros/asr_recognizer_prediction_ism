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
#include <map>
#include <string>

//Pkg includes
#include "pose_prediction_ism/shortest_path.h"
#include "pose_prediction_ism/old_prediction_non_normalized.h"
#include "pose_prediction_ism/old_prediction_normalized.h"
#include "pose_prediction_ism/best_path.h"
#include "pose_prediction_ism/random_path.h"

//Local includes
#include "asr_recognizer_prediction_ism/pose_prediction.h"
#include "asr_recognizer_prediction_ism/param_helper.h"

using namespace std;
using namespace ros;
using namespace asr_msgs;
using namespace std_srvs;
using namespace ISM;
using namespace VIZ;

namespace recognizer_prediction_ism
{

PosePrediction::PosePrediction(std::vector<ISM::RecognitionResultPtr>& results_already_shared, SharedRecognitionResultsManagerPtr shared_recognition_results_ptr):
    results_already_shared_(results_already_shared),
    node_handle_(ros::this_node::getName())
{

    ParamHelper ph("Pose Prediction", node_handle_);
    /* ----------------- Names ------------------  */
    string point_cloud_server_name;
    string pose_prediction_markers_publisher_name;
    string scene_markers_pose_prediction_publisher_name;
    string found_object_client_name;
    bool latched;

    ph.getParam<string>("foundObjectClientName", &found_object_client_name, "");
    ph.getParam<string>("sceneMarkersPosePredictionPublisherName", &scene_markers_pose_prediction_publisher_name, "");
    ph.getParam<string>("posePredictionMarkersPublisherName", &pose_prediction_markers_publisher_name, "");
    ph.getParam<string>("PointCloudServerName", &point_cloud_server_name, "");
    ph.getParam<bool>("latched", &latched, false);

    found_object_client_ = node_handle_.serviceClient<asr_world_model::GetFoundObjectList>(found_object_client_name);
    ROS_ASSERT_MSG(found_object_client_.waitForExistence(), "Please start the world model");

    point_cloud_server_ = node_handle_.advertiseService(point_cloud_server_name, &PosePrediction::processGetPointCloudServiceCall, this);

    pose_prediction_markers_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(pose_prediction_markers_publisher_name, 1000, latched);
    pose_prediction_visualizer_ptr_ = PosePredictionVisualizerPtr(new PosePredictionVisualizer(pose_prediction_markers_publisher_, ros::NodeHandle(node_handle_.getNamespace() + "/pose_prediction_visualizer/")));

    scene_markers_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(scene_markers_pose_prediction_publisher_name, 1000, latched);
    res_visualizer_ = ISMResultVisualizerRVIZPtr(new ISMResultVisualizerRVIZ(scene_markers_publisher_, ros::NodeHandle(node_handle_.getNamespace() + "/pose_prediction_result_visualizer/")));

    ROS_INFO("PosePrediction: Visualization initialized.");


    /* ----------------- Pose prediction attributes------------------  */
    ph.getParam<string>("dbfilename", &database_filename_, "");
    ph.getParam<double>("predictionGenerationFactor", &prediction_generation_factor_, 1);

    /* ----------------- Resampler attributes------------------  */
    bool equal_results_number;
    unsigned int importance_resampled_size;

    ph.getParam<bool>("equal_results_number", &equal_results_number, true);
    ph.getParam<unsigned int>("importance_resampled_size", &importance_resampled_size, 10);
    resampler_ptr_ = ResamplerPtr(new Resampler(equal_results_number, importance_resampled_size, shared_recognition_results_ptr));

    ROS_INFO("PosePrediction: Resampler initialized.");

    /* ----------------- ROS & Viz initialization ------------------  */

    reconfigure_server_ = new dynamic_reconfigure::Server<asr_recognizer_prediction_ism::pose_predictionConfig>(ros::NodeHandle(node_handle_.getNamespace() + "/pose_prediction/"));
    dynamic_reconfigure::Server<asr_recognizer_prediction_ism::pose_predictionConfig>::CallbackType reconf_callback = boost::bind(&PosePrediction::dynamicReconfCallback, this, _1, _2);
    reconfigure_server_->setCallback(reconf_callback);

}


void PosePrediction::dynamicReconfCallback(asr_recognizer_prediction_ism::pose_predictionConfig &config, uint32_t level){
    ROS_DEBUG_NAMED("PosePrediction", "Callback PosePrediction::dynamicReconfCallback() is called.");

    switch(config.posePredictor){
    case 0:
        pose_predictor_ptr_ = pose_prediction_ism::BestPathPtr(new pose_prediction_ism::BestPath(database_filename_));
        break;
    case 1:
        pose_predictor_ptr_ = pose_prediction_ism::PaperPredictionNonNormalizedPtr(new pose_prediction_ism::PaperPredictionNonNormalized(database_filename_));
        break;
    case 2:
        pose_predictor_ptr_ = pose_prediction_ism::PaperPredictionNormalizedPtr(new pose_prediction_ism::PaperPredictionNormalized(database_filename_));
        break;
    case 3:
        pose_predictor_ptr_ = pose_prediction_ism::RandomPathPtr(new pose_prediction_ism::RandomPath(database_filename_));
        break;
    case 4:
        pose_predictor_ptr_ = pose_prediction_ism::ShortestPathPtr(new pose_prediction_ism::ShortestPath(database_filename_));
        break;
    default:
        pose_predictor_ptr_ = pose_prediction_ism::ShortestPathPtr(new pose_prediction_ism::ShortestPath(database_filename_));
        break;
    }
    pose_predictor_ptr_->setPredictionGenerationFactor(prediction_generation_factor_);
    ROS_INFO_STREAM("Using " << pose_predictor_ptr_ << " for pose prediction.");

    is_visualization_active_ = config.enableVisualization;
    ROS_INFO_COND(is_visualization_active_, "Visualization is active");
    ROS_WARN_COND(!is_visualization_active_, "Visualization is not active");

}

bool PosePrediction::processGetPointCloudServiceCall(asr_recognizer_prediction_ism::GetPointCloud::Request &req,
                                                     asr_recognizer_prediction_ism::GetPointCloud::Response &res)
{
    ROS_INFO("\n==================================================\n");

    ROS_INFO("Callback PosePrediction::processGetPointCloudServiceCall() is called.");

    vector<AsrObject> found_objects;
    for (asr_msgs::AsrObject asr_object : req.objects)
    {
        ROS_ASSERT_MSG(!asr_object.sampledPoses.empty(),"Object estimate with no samples not allowed.");
        found_objects.push_back(asr_object);
    }


    //Ask world model if we have found any objects during asr => Missing objects for prediction != objects not in scene recognition results.
    if (!found_objects.empty())
    {
        stringstream output;
        output << "Summary of results from PosePrediction::processGetPointCloudServiceCall() in rp_ism_node." << endl;

        pose_predictor_ptr_->setFoundObjects(found_objects);
        output << "These results come from importance (re)sampling and are used for pose prediction:";
        //Ask if there are any scene recognition results sampled from shared memory to process. Need reference of scene recognition result for pose prediction.
        if (resampler_ptr_->recognitionResultsAvailable())
        {

            pose_prediction_visualizer_ptr_->clearAllMarkerOfTopic();
            res_visualizer_->clearAllMarkerOfTopic();

            ROS_DEBUG_NAMED("PosePrediction", "Flushed visualization for pose prediction.");

            //Here actual resampling of recognition results for pose prediction takes place.
            std::vector<RecognitionResult> sampled_results = resampler_ptr_->drawSamples();
            res_visualizer_->setSceneCount(sampled_results.size());

            AsrAttributedPointCloud union_attributed_point_cloud;
            set<pair<string,string > > predicted_objects;

            char result_specifier = 'a';

            ROS_INFO_STREAM("List of recognition results used to predict poses with their relative summarized votes (percentages):");
            for (unsigned int i = 0; i < sampled_results.size(); i ++) {
                ISM::RecognitionResult result = sampled_results[i];
                ROS_INFO_STREAM("i: " << i << " for patternName: " << result.patternName << " with %: " << 1.0 / sampled_results.size());
            }

            int i = 0;
            for (RecognitionResult recognition_result : sampled_results)
            {

                //Save recognition result just extracted by importance sampling to temporary buffer until initial search reached again.
                results_buffer_.push_back(RecognitionResultPtr(new RecognitionResult(recognition_result)));
                output << endl << recognition_result << endl;

                //May add reference
                visualize(recognition_result, &result_specifier, &i, sampled_results.size(), &union_attributed_point_cloud, &predicted_objects);
            }

            output << "Poses have been predicted for the following objects: " << endl;

            for (pair<string, string> predicted_object : predicted_objects)
                output << "[" << predicted_object.first << ", " << predicted_object.second << "], ";
            output << endl;

            pose_prediction_visualizer_ptr_->publishCollectedMarkers();

            res_visualizer_->publishCollectedMarkers();

            checkPointCloud(union_attributed_point_cloud);
            //Return merged pointcloud in service call response.y
            res.point_cloud = union_attributed_point_cloud;
            res.output = output.str();

            ROS_INFO("Successfully finished pose prediction for given recognition results.");

            ROS_INFO("\n==================================================\n");
            return true;
        }
        else
        {
            ROS_ERROR("No recognition results available for pose predicition. Returning empty point cloud.");
            AsrAttributedPointCloud cloud;
            res.point_cloud = cloud;
            res.output = "No recognition results available for pose predicition. Returning empty point cloud.";

            ROS_INFO_STREAM("Pushing already used results from this indirect-based run into buffer: " << results_buffer_.size());
            //Save buffered recognition results from current indirect-based object search run to prevent using them again.
            results_already_shared_.insert(results_already_shared_.end(), results_buffer_.begin(), results_buffer_.end());
            //Reset buffer for next indirect-based object search execution.
            results_buffer_.clear();

            ROS_INFO("\n==================================================\n");
            return true;
        }
    }
    else
    {
        ROS_ERROR("Failed to call the service get_found_objects");

        ROS_INFO("\n==================================================\n");
        return false;
    }
}

void PosePrediction::visualize(RecognitionResult recognition_result, char *result_specifier, int *i, unsigned int sampled_results_size, AsrAttributedPointCloud *union_attributed_point_cloud, set<pair<string, string>> *predicted_objects){
    //Get reference pose of recognition result as starting point (or better pose) for pose prediction.
    PosePtr reference_pose_ptr = recognition_result.referencePose;
    //This is the actual pose prediction call to asr_lib_pose_prediction_ism.
    AsrAttributedPointCloud current_point_cloud =
            pose_predictor_ptr_->predictUnfoundPoses(reference_pose_ptr, recognition_result.patternName, 1.0 / sampled_results_size);
    (*i)++;

    //Merge pose prediction results for current recognition result with results from other recognition results, used too.
    for (AsrAttributedPoint attributed_point : current_point_cloud.elements)
    {
        union_attributed_point_cloud->elements.push_back(attributed_point);
        predicted_objects->insert(make_pair(attributed_point.type, attributed_point.identifier));
    }

    if (is_visualization_active_)
    {
        res_visualizer_->addVisualization(RecognitionResultPtr(new RecognitionResult(recognition_result)));
        string pose_prediction_name_space = pose_predictor_ptr_->getMarkerNameSpace() + "_" + (*result_specifier);
        pose_prediction_visualizer_ptr_->addPosePredictionVisualization(PosePtr(reference_pose_ptr), current_point_cloud, pose_prediction_name_space);
        (*result_specifier)++;
    }
}

void PosePrediction::checkPointCloud(AsrAttributedPointCloud attributed_point_cloud)
{
    map<string, map<string, unsigned int> > currentMap;
    for (AsrAttributedPoint attributed_point : attributed_point_cloud.elements)
        currentMap[attributed_point.type][attributed_point.identifier]++;

    ROS_INFO_STREAM("Attributed pointcloud from pose prediction contains:" << endl);
    for (map<string, map<string,unsigned int> >::iterator typeIt = currentMap.begin(); typeIt != currentMap.end(); ++typeIt)
        for (map<string, unsigned int>::iterator idIt = typeIt->second.begin(); idIt != typeIt->second.end(); ++idIt)
            ROS_INFO_STREAM(idIt->second << " predicted poses for object "<< "(" << typeIt->first << "," << idIt->first << ").");

}
}
