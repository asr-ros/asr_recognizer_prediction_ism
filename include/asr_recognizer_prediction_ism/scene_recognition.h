/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RECOGNIZER_PREDICTION_ISM_SCENE_RECOGNITION_NODE_H
#define RECOGNIZER_PREDICTION_ISM_SCENE_RECOGNITION_NODE_H

#include <iostream>
#include <fstream>
#include <cmath>

#include <ISM/recognizer/Recognizer.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/ObjectSet.hpp>

/* ----------------- Foreign messages/services  ------------------  */
#include <std_srvs/Empty.h>

#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <asr_ism_visualizations/ism_result_visualizer_rviz.hpp>
#include <dynamic_reconfigure/server.h>
#include <asr_recognizer_prediction_ism/rp_ism_nodeConfig.h>

/* ----------------- Package messages/services  ------------------  */
#include <asr_world_model/CompletePattern.h>
#include <asr_world_model/VisualizeSampledPoses.h>
#include <asr_world_model/PushCompletePatterns.h>


#include "asr_recognizer_prediction_ism/FindScenes.h"
#include "asr_recognizer_prediction_ism/SetLogDir.h"
#include "asr_recognizer_prediction_ism/ToggleVisualization.h"

/* ----------------- Local includes ------------------  */
#include "shared_recognition_results_manager.h"
#include "ism_helper.h"

namespace recognizer_prediction_ism
{
  class SceneRecognition
  {
    /* ----------------- ROS ------------------  */
    ros::NodeHandle node_handle_;
    ros::ServiceServer find_scenes_server_;
    ros::ServiceServer set_log_dir_server_;
    ros::ServiceServer toggle_visualization_server_;
    ros::ServiceServer reset_server_;

    ros::ServiceClient viz_samples_client_;
    ros::ServiceClient push_complete_patterns_server_;

    /* ----------------- ISM ------------------  */
    ISM::RecognizerPtr recognizer_;
    ISM::TableHelperPtr table_helper_;
    double bin_size_;
    double max_projection_angle_deviation_;
    std::string database_filename_;
    unsigned int rater_type_;

    std::string object_constellation_folder_path_;
    int object_constellation_file_counter_;

    unsigned int object_set_max_count_;

    IH::ObjectConverterPtr converter;

    //Settings for handling of rotation invariant objects
    int enable_rotation_mode_;
    std::string rotation_frame_;
    std::string rotation_object_type_;
    std::string rotation_object_id_;

    //Size of last object set, on which find_scenes() has been performed.
    unsigned int last_object_set_size_;
    //Buffer shared with pose prediction.
    std::vector<ISM::RecognitionResultPtr>& results_already_shared_;

    //Mapping from patterns to objects from loaded sqlite file.
    void constructPatternObjectMapping();

    std::map<std::string, std::set<std::pair<std::string, std::string> > > pattern_object_map_;
    std::set<std::string> unique_pattern_names_;

    /* ----------------- Visualization ------------------  */
    std::string scene_markers_publisher_name_;
    std::string viz_samples_client_name_;
    std::string push_complete_patterns_name_;

    double marker_life_time_;
    double step_;
    std::string base_frame_;
    bool is_visualization_active_;
    VIZ::ISMResultVisualizerRVIZPtr res_visualizer_;
    VIZ::ObjectModelVisualizerRVIZ* object_model_visualizer_;
    ros::Publisher visualization_publisher_;
    ros::Publisher my_publisher_;
    SharedRecognitionResultsManagerPtr shared_recognition_results_ptr_;

    std::vector<ISM::RecognitionResultPtr> incomplete_recognition_results_;
    std::vector<ISM::RecognitionResultPtr> results_for_visualization_;
    std::set<std::pair<std::string, std::string> > object_types_and_ids_set_;

    /* ----------------- Help functions ------------------  */
    std::vector<ISM::ObjectPtr> extractRealObjects(ISM::RecognitionResultPtr result_ptr);

    static bool compareConfidence(ISM::RecognitionResultPtr i,ISM::RecognitionResultPtr j);
    static bool isCompleteSubset(std::set<std::pair<std::string, std::string> > set_a,
                 std::set<std::pair<std::string, std::string> > set_b);

  public:

    /**
     * @brief SceneRecognitionNode Constructor
     */
    //TODO: Comment.
    SceneRecognition(std::vector<ISM::RecognitionResultPtr>& results_already_shared, SharedRecognitionResultsManagerPtr shared_recognition_results_ptr);

    ~SceneRecognition() { }

    /**
     * @brief processFindScenesServiceCall Receives service call with detected AsrObjects.
     * @param request
     * @param response Returns whether scenes are found.
     * @return
     */
    bool processFindScenesServiceCall(FindScenes::Request &request, FindScenes::Response &response);

    /**
     * @brief processSetLogDirServiceCall Receives service call where to write object constellations, used for scene recognition.
     * @param request The path to the folder, where xml files are written.
     * @param response
     * @return
     */
    bool setLogFilePathServiceCall(SetLogDir::Request &request, SetLogDir::Response &response);

    /**
     * @brief processToggleVisualizationServiceCall Toggle the visualization via service call.
     * @param request
     * @param response
     * @return
     */
    bool processToggleVisualizationServiceCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    /**
     * @brief processResetServiceCall Clears the old recognition results.
     * @param request
     * @param response
     * @return
     */
    bool processResetServiceCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

  private:
    void createObjectSetsFromPoseSamples(std::vector<asr_msgs::AsrObject>::iterator object_it, std::vector<asr_msgs::AsrObject>::iterator end_it,
                     ISM::ObjectSetPtr temp_set_ptr, std::vector<ISM::ObjectSetPtr>& object_sets, bool measured_value_only);
    void writeObjectConstellationToXML(std::vector<ISM::ObjectPtr> &objects, int sceneRecogCount, int constellation_count);

    void filterIncompleteRR(std::vector<ISM::RecognitionResultPtr> &recognition_results);
    void sortBestRRperScene();
    void deleteAllNewRR();

  };

  typedef boost::shared_ptr<SceneRecognition> SceneRecognitionPtr;

}
#endif
