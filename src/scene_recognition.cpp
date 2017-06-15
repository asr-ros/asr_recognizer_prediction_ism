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
#include <stdio.h>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include<fstream>

//Pkg includes
#include <ros/ros.h>
#include <asr_msgs/AsrObject.h>
#include <geometry_msgs/Pose.h>
#include <ISM/recognizer/Recognizer.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/GeometryHelper.hpp>
#include <ISM/common_type/Pose.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

//Local includes
#include "asr_recognizer_prediction_ism/scene_recognition.h"
#include "asr_recognizer_prediction_ism/param_helper.h"
namespace ser = ros::serialization;

using namespace std_srvs;
using namespace std;
using namespace ros;
using namespace ISM;
using namespace IH;
using namespace VIZ;

namespace recognizer_prediction_ism
{

SceneRecognition::SceneRecognition(std::vector<ISM::RecognitionResultPtr>& results_already_shared, SharedRecognitionResultsManagerPtr shared_recognition_results_ptr) :
    node_handle_(ros::this_node::getName()),
    //Used for writing input configurations to scene recognition onto filesystem.
    object_constellation_folder_path_(""),
    object_constellation_file_counter_(0),
    last_object_set_size_(0),
    results_already_shared_(results_already_shared),
    is_visualization_active_(true),
    shared_recognition_results_ptr_(shared_recognition_results_ptr)

{
    ParamHelper ph("Scene Recognition", node_handle_);
    ph.getParam<double>("bin_size", &bin_size_, 0.1);
    ph.getParam<double>("maxProjectionAngleDeviation", &max_projection_angle_deviation_, 10);

    ph.getParam<string>("dbfilename", &database_filename_, "");
    if(database_filename_.empty()){
        ROS_FATAL("DB Filename empty. Shutting down node!");
        ros::shutdown();
    }

    ph.getParam<int unsigned>("raterType", &rater_type_, 0);
    ph.getParam<int unsigned>("objectSetMaxCount", &object_set_max_count_, 676);

    ph.getParam<int>("enableRotationMode", &enable_rotation_mode_, 0);
    ph.getParam<string>("rotationFrame", &rotation_frame_, "/map");
    ph.getParam<string>("rotationObjectType", &rotation_object_type_, "");
    ph.getParam<string>("rotationObjectId", &rotation_object_id_, "");

    ph.getParam<string>("baseFrame", &base_frame_, "/map");
    ph.getParam<string>("sceneMarkersSceneRecognitionPublisherName", &scene_markers_publisher_name_, "");
    ph.getParam<string>("vizSamplesClientName", &viz_samples_client_name_, "");
    ph.getParam<string>("pushCompletePatternsName", &push_complete_patterns_name_, "");
    ph.getParam<double>("markerLifetime", &marker_life_time_, 0);


    /* ----------------- ROS & Viz initialization ------------------  */
    viz_samples_client_ = node_handle_.serviceClient<asr_world_model::VisualizeSampledPoses>(viz_samples_client_name_);
    push_complete_patterns_server_ = node_handle_.serviceClient<asr_world_model::PushCompletePatterns>(push_complete_patterns_name_);

    find_scenes_server_ = node_handle_.advertiseService("find_scenes", &SceneRecognition::processFindScenesServiceCall, this);
    set_log_dir_server_ = node_handle_.advertiseService("set_log_dir", &SceneRecognition::setLogFilePathServiceCall, this);
    toggle_visualization_server_ = node_handle_.advertiseService("toggle_visualization", &SceneRecognition::processToggleVisualizationServiceCall, this);
    reset_server_ = node_handle_.advertiseService("reset", &SceneRecognition::processResetServiceCall, this);

    /* ----------------- ism initialization ------------------  */

    table_helper_ = TableHelperPtr(new TableHelper(database_filename_));
    recognizer_ = RecognizerPtr(new Recognizer(database_filename_, bin_size_, max_projection_angle_deviation_, true, rater_type_));
    converter = ObjectConverterPtr(new ObjectConverter(base_frame_, false, false, enable_rotation_mode_, rotation_frame_, rotation_object_type_, rotation_object_id_, false));

    ROS_INFO("SceneRecognition: Initialized recognizer from ism_lib.");

    /* ----------------- viz initialization ------------------  */

    visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(scene_markers_publisher_name_, 1000);
    res_visualizer_ = ISMResultVisualizerRVIZPtr(new ISMResultVisualizerRVIZ(visualization_publisher_, ros::NodeHandle(node_handle_.getNamespace() + "/result_visualizer/")));
    object_model_visualizer_ = new ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame_, "", marker_life_time_);

    ROS_INFO("SceneRecognition: Initialized ism visualizations.");
    constructPatternObjectMapping();
}

void SceneRecognition::constructPatternObjectMapping(){
    //Constructing a mapping from pattern names to objects, belonging to pattern, from sqlite db that makes up our scene models.

    ROS_DEBUG_NAMED("SceneRecognition", "Building pattern_object_map_ from database.");
    for(string& pattern_name : table_helper_->getModelPatternNames())
    {
        set<pair<string, string> > object_type_and_id_in_pattern =
                table_helper_->getObjectTypesAndIdsBelongingToPattern(pattern_name);
        set<pair<string, string> > object_types_and_ids_non_sub;
        for (pair<string, string> object : object_type_and_id_in_pattern)
        {
            //Filter out patternNames contain _sub* endings, as they do not name entire scenes, we need.
            if(object.first.find("_sub") != string::npos)
                continue;
            object_types_and_ids_non_sub.insert(make_pair(object.first,object.second));
        }
        unsigned int last_index = pattern_name.find("_sub");
        if (last_index != string::npos)
        {
            //contains _sub ending
            string pattern_name_without_sub = pattern_name.substr(0, last_index);
            ROS_DEBUG_STREAM_NAMED("SceneRecognition", "Renamed scene " << pattern_name << " to " << pattern_name_without_sub << ".");
            pattern_name = pattern_name_without_sub;
        }
        map<string, set<pair<string, string> > >::iterator it = pattern_object_map_.find(pattern_name);
        if (it != pattern_object_map_.end())
        {
            set<pair<string, string> > old_set = it->second;
            old_set.insert(object_types_and_ids_non_sub.begin(), object_types_and_ids_non_sub.end());
            it->second = old_set;
        }
        else
        {
            pattern_object_map_.insert(pair<string, set<pair<string, string> > >(pattern_name, object_types_and_ids_non_sub));
        }
        for (auto obj : pattern_object_map_.find(pattern_name)->second)
        {
            ROS_DEBUG_STREAM_NAMED("SceneRecognition", "Map element for scene " << pattern_name << " contains object [" << obj.first << ", " << obj.second << "].");
        }
    }

    vector<string> pattern_names = table_helper_->getModelPatternNames();
    unique_pattern_names_.clear();
    for (string pattern_name : pattern_names)
    {
        if (pattern_name.find("_sub") == string::npos)
        {
            unique_pattern_names_.insert(pattern_name);
        }
    }
}

  void SceneRecognition::createObjectSetsFromPoseSamples(vector<asr_msgs::AsrObject>::iterator object_it, vector<asr_msgs::AsrObject>::iterator end_it,
                            ObjectSetPtr temp_set_ptr, vector<ObjectSetPtr>& object_sets, bool measured_value_only) {

    //We have added a sample from each object (type and id) to object set -> save it.
    if(object_it == end_it){
        object_sets.push_back(ObjectSetPtr( new ObjectSet(*temp_set_ptr)));
        return;
    }

    //Set it to only when we disable pose sample processing.
    unsigned int used_samples_count = measured_value_only ? 1 : object_it->sampledPoses.size();

    //Go through all samples poses for a given object (type and id).
    for (unsigned int i = 0; i < used_samples_count; i++) {

      geometry_msgs::Pose sample_pose = object_it->sampledPoses.at(i).pose;
      double sampled_position_x = sample_pose.position.x;
      double sampled_position_y = sample_pose.position.y;
      double sampled_position_z = sample_pose.position.z;

      double sampled_quaternion_w = sample_pose.orientation.w;
      double sampled_quaternion_x = sample_pose.orientation.x;
      double sampled_quaternion_y = sample_pose.orientation.y;
      double sampled_quaternion_z = sample_pose.orientation.z;

      QuaternionPtr ism_quaternion_ptr = QuaternionPtr(new Quaternion(sampled_quaternion_w,
								      sampled_quaternion_x,
								      sampled_quaternion_y,
								      sampled_quaternion_z));
      QuaternionPtr ism_quaternion_normalized_ptr = GeometryHelper::normalize(ism_quaternion_ptr);
      //Create ism object (from which object sets are made of) from the given asr object.
      ObjectPtr new_object(new Object(object_it->type,
				      new Pose(
					       new Point(sampled_position_x,
							 sampled_position_y,
							 sampled_position_z),
					       ism_quaternion_normalized_ptr),
                      object_it->identifier,
                      boost::filesystem::path(object_it->meshResourcePath).replace_extension(".dae").string()));
      //Add object sample to incomplete set for the object, considered in the current recursion level.
      temp_set_ptr->objects.push_back(new_object);
      //Go to next object in set to chose sample.
      createObjectSetsFromPoseSamples(object_it+1, end_it, temp_set_ptr, object_sets, measured_value_only);
      //Remove current sample of current object before adding another sample for the current object in next run of loop.
      temp_set_ptr->objects.pop_back();
    }

}

bool SceneRecognition::processFindScenesServiceCall(asr_recognizer_prediction_ism::FindScenes::Request &request, asr_recognizer_prediction_ism::FindScenes::Response &response)
{
    ROS_INFO("\n==================================================\n");
    ROS_INFO("Callback SceneRecognition::processFindScenesServiceCall() is called.");

    if (request.objects.empty())
    {
        ROS_INFO("No objects passed to scene detection callback.");
        response.result_size.data = 0;
        response.output = "";

        ROS_INFO("\n==================================================\n");
        return false;

      }
    for (asr_msgs::AsrObject pbd_object : request.objects)
    {
        ROS_ASSERT_MSG(!pbd_object.sampledPoses.empty(),"Object estimate with no samples not allowed.");
    }

    //Clear shared RecognitionResult vector.
    shared_recognition_results_ptr_->clearResults();

    ROS_DEBUG_STREAM_NAMED("SceneRecognition", shared_recognition_results_ptr_->getResultsNumber() << " recognition results in shared memory after flushing at the begin of findScenes().");

    //Contains an object set per combination of object pose samples for all input objects.
    vector<ObjectSetPtr> object_sets;
    object_types_and_ids_set_.clear();

    //Precalculate how many object sets would be generated by pose sample processing to avoid overflow. Samples just required when few poses available.
    unsigned int number_of_combinations = 1;

    for (asr_msgs::AsrObject asr_object : request.objects)
      {
    number_of_combinations *= asr_object.sampledPoses.size();
	//Get all objects (types and ids) from input set as well.
    object_types_and_ids_set_.insert(make_pair(asr_object.type, asr_object.identifier));
      }

    //Get all object sets from combining samples of all given objects.
    createObjectSetsFromPoseSamples(request.objects.begin(), request.objects.end(), ObjectSetPtr(new ObjectSet()),
                                    object_sets, number_of_combinations > object_set_max_count_);

    ROS_INFO_STREAM("Number of object set combinations: " << object_sets.size());

    //Only show pose samples in RViz, when processed.
    if(number_of_combinations <= object_set_max_count_)
    {
        ROS_ASSERT_MSG(number_of_combinations == object_sets.size(), "Error in recursive object set generation, expect: %u actual: %zu", number_of_combinations, object_sets.size());

        if (viz_samples_client_.exists()) {
            asr_world_model::VisualizeSampledPoses srv;
            for (asr_msgs::AsrObject pbd_object : request.objects)
            {
                srv.request.object_type = pbd_object.type;
                srv.request.object_id = pbd_object.identifier;
                //Show additional samples for this object.
                viz_samples_client_.call(srv);
            }

        } else {
            ROS_WARN("visualization server from world model does not exists -> sampled poses will not be visualized");
        }

    } else {
        ROS_ASSERT_MSG(1 == object_sets.size(), "Error in recursive object set generation, expect: %u actual: %zu", 1, object_sets.size());
    }

    //For every scene type (pattern name), return the root of the recognized scene model rated best of
    //the given object set (concerning its confidence).

    results_for_visualization_.clear();
    incomplete_recognition_results_.clear();
    std::vector<ISM::RecognitionResultPtr> recognition_results;

    //Call scene recognition for each object set separately since otherwise object samples get lost during scene recognition.
    for(vector<ObjectSetPtr>::iterator set_it = object_sets.begin(); set_it != object_sets.end(); set_it++) {

        //Backup xml file of object set for fake_object_recognition.
        writeObjectConstellationToXML((*set_it)->objects, object_constellation_file_counter_, distance(object_sets.begin(), set_it));

        //Normalize orientations of rotation invariant objects.
        converter->normalizeRotationInvariantObjects((*set_it));

        //THIS IS THE EFFECTIVE CALL TO ISM SCENE RECOGNITION.
        vector<RecognitionResultPtr> resultsForCombination = recognizer_->recognizePattern((*set_it), 0, -1);

        recognition_results.insert(recognition_results.end(), resultsForCombination.begin(), resultsForCombination.end());

    }

    ++object_constellation_file_counter_;
    ROS_INFO_STREAM("This is the " << object_constellation_file_counter_ << "th scene recognition run");

    if (recognition_results.empty())
    {
        ROS_INFO("Not any scene could be recognized by ism in input objects.");
        response.result_size.data = 0;
        response.output = "Not any scene could be recognized by ism in input objects.";

        ROS_INFO("\n==================================================\n");
        return true;
    }

    stringstream output;
    output << "Summary of results from SceneRecognition::processFindScenesServiceCall() in rp_ism_node." << endl;
    output << "These are the best recognition results, returned per patternName:";

    //Sorting MUST just happen once (at least before we used stable sort). Otherwise order of scenes with same confidence gets inconsistent.
    //recognitionResults are now sorted
    stable_sort(recognition_results.begin(), recognition_results.end(), compareConfidence);

    filterIncompleteRR(recognition_results);



    //Only fill shared memory with new recognition results if a new object has been detected since last execution of
    if (request.objects.size() != last_object_set_size_)
    {

        ROS_INFO("New objects detected since last scene recognition call.");
        ROS_INFO("Flushing shared memory with recognition results and inserting new results.");

        deleteAllNewRR();

        //Push back all incomplete recognition results to result vector in shared memory for prediction.
        shared_recognition_results_ptr_->addResults(incomplete_recognition_results_);

        ROS_DEBUG_STREAM_NAMED("SceneRecognition", shared_recognition_results_ptr_->getResultsNumber() << " recognition results in shared memory after adding new results.");

        last_object_set_size_ = request.objects.size();

        //reverse resultsForVisualization to show best scenes
        reverse(results_for_visualization_.begin(), results_for_visualization_.end());

        //We only want to log and visualize the best results for every pattern type. Therefore we have a list of all patterns for which no result has been shown yet.
        set<string> remaining_pattern_names_counter = unique_pattern_names_;


        int scene_count = 0;
        for (RecognitionResultPtr& recognition_result_ptr : results_for_visualization_)
        {
            const string current_pattern_name = recognition_result_ptr->patternName;
            if (remaining_pattern_names_counter.find(current_pattern_name) != remaining_pattern_names_counter.end())
            {
                scene_count++;
            }
            remaining_pattern_names_counter.erase(current_pattern_name);
        }

        if (is_visualization_active_) {

            res_visualizer_->clearAllMarkerOfTopic();
            res_visualizer_->setSceneCount(scene_count);

            ROS_DEBUG_NAMED("SceneRecognition", "Flushed visualization for scene recognition.");
        }

        sortBestRRperScene(output);

        if (is_visualization_active_){
            res_visualizer_->publishCollectedMarkers();

        }
    }
    //construct data for service call result
    std_msgs::UInt8 result_size;
    result_size.data = recognition_results.size();
    response.result_size = result_size;

    ROS_INFO_STREAM("OUT: "<< output.str());

    response.output = output.str();

    ROS_INFO("\n==================================================\n");
    return true;
}

void SceneRecognition::filterIncompleteRR(std::vector<ISM::RecognitionResultPtr> &recognition_results){
    //TODO-DONE: Create method for filtering incomplete recognition results.
    set<string> complete_pattern_names;
    vector<asr_world_model::CompletePattern> completePatternsToPush;

    ROS_INFO_STREAM("These are all sorted " << recognition_results.size() << " recognition results from the current recognition call:");
    //Get all patterns for which all objects have been found.
    for(RecognitionResultPtr& recognition_result_ptr : recognition_results)
    {
        ROS_DEBUG_STREAM_NAMED("SceneRecognition", recognition_result_ptr);

        if (SceneRecognition::isCompleteSubset(pattern_object_map_.find(recognition_result_ptr->patternName)->second, object_types_and_ids_set_))
        {
            complete_pattern_names.insert(recognition_result_ptr->patternName);
            vector<asr_world_model::CompletePattern>::iterator it = find_if(completePatternsToPush.begin(), completePatternsToPush.end(), [&recognition_result_ptr]
                                                                        (const asr_world_model::CompletePattern& r) { return r.patternName == recognition_result_ptr->patternName; } );
            if (it == completePatternsToPush.end()) {
                asr_world_model::CompletePattern completePattern;
                completePattern.patternName = recognition_result_ptr->patternName;
                completePattern.confidence = recognition_result_ptr->confidence;
                completePatternsToPush.push_back(completePattern);
            } else {
                it->confidence = max(it->confidence, static_cast<float>(recognition_result_ptr->confidence));
            }
        }
    }

    asr_world_model::PushCompletePatterns push_complete_patterns;
    for (const asr_world_model::CompletePattern &pattern : completePatternsToPush) {
        push_complete_patterns.request.completePatterns.push_back(pattern);
    }
    push_complete_patterns_server_.call(push_complete_patterns);

    for(string pattern_name : complete_pattern_names)
        ROS_INFO_STREAM("Found all objects in scene " << pattern_name << ", hence no pose prediction for it.");

    //Filter out completely detected patterns and construct copy of all results for viz (latter required for being able to resort recognition results for viz).
    for(RecognitionResultPtr& recognition_result_ptr : recognition_results)
    {
        results_for_visualization_.push_back(recognition_result_ptr);
        if (complete_pattern_names.find(recognition_result_ptr->patternName) == complete_pattern_names.end())
        {
            incomplete_recognition_results_.push_back(recognition_result_ptr); //use scene for pose prediction
        }
    }
}


void SceneRecognition::sortBestRRperScene(std::stringstream& output){
    set<string> remaining_pattern_names = unique_pattern_names_;
    // resultsForVisualization are sorted now.
    // Pick best result of incomplete scenes per scene and viz it.
    for (RecognitionResultPtr& recognition_result_ptr : results_for_visualization_)
    {
        const string current_pattern_name = recognition_result_ptr->patternName;
        if (remaining_pattern_names.find(current_pattern_name) != remaining_pattern_names.end())
        {
            output << endl << recognition_result_ptr << endl;
            if (is_visualization_active_) {

                object_model_visualizer_->drawObjectModels(extractRealObjects(recognition_result_ptr));
                res_visualizer_->addVisualization(recognition_result_ptr);

            }

            remaining_pattern_names.erase(current_pattern_name);
        }
        // best results for every scene type has been visualized
        if (remaining_pattern_names.size() == 0)
        {
            break;
        }
    }
}


void SceneRecognition::deleteAllNewRR(){
    ROS_INFO("Now DELETING all NEW RECOGNITION RESULTS, already used for pose prediction.");

    ROS_INFO_STREAM("Number of already used results: " << results_already_shared_.size());
    int number_of_erased_incomplete_recognition_results = 0;

    vector<RecognitionResultPtr>::iterator incomplete_it = incomplete_recognition_results_.begin();
    //Remove all current recognition results that have already been chosen by importance sampling to prevent them from being processed again. Inefficient otherwise...
    while (incomplete_it != incomplete_recognition_results_.end()) {
        if (find(results_already_shared_.begin(), results_already_shared_.end(), *incomplete_it) == results_already_shared_.end()) {
            ++incomplete_it;
        }
        else {
            incomplete_it = incomplete_recognition_results_.erase(incomplete_it);
            ++number_of_erased_incomplete_recognition_results;
        }
    }
    ROS_DEBUG_STREAM_NAMED("SceneRecognition", "Number of deleted results: " << number_of_erased_incomplete_recognition_results);
    ROS_DEBUG_STREAM_NAMED("SceneRecognition", "Remaining recognition results: " << incomplete_recognition_results_.size());
}


bool SceneRecognition::setLogFilePathServiceCall(asr_recognizer_prediction_ism::SetLogDir::Request &request, asr_recognizer_prediction_ism::SetLogDir::Response &response)
{
    if (boost::filesystem::exists(request.log_dir_path))
    {
        object_constellation_folder_path_ = request.log_dir_path;
        return true;
    }
    ROS_ERROR_STREAM(request.log_dir_path << " does not exist!");
    return false;
}

bool SceneRecognition::processToggleVisualizationServiceCall(Empty::Request &request,
                                                             Empty::Response &response)
{
    ROS_DEBUG_NAMED("SceneRecognition", "Callback SceneRecognition::processToggleVisualizationServiceCall() is called.");
    is_visualization_active_ = !is_visualization_active_;
    ROS_INFO_COND(is_visualization_active_, "Visualization is active");
    ROS_WARN_COND(!is_visualization_active_, "Visualization is not active");
    return true;
}

bool SceneRecognition::processResetServiceCall(Empty::Request &request, Empty::Response &response)
{
    ROS_DEBUG_NAMED("SceneRecognition", "Callback SceneRecognition::processResetServiceCall() is called.");
    ROS_DEBUG_NAMED("SceneRecognition", "Setting last object count from %d back to 0", last_object_set_size_);
    last_object_set_size_ = 0;
    ROS_DEBUG_NAMED("SceneRecognition", "Resetting shared mem with recognition results.");
    shared_recognition_results_ptr_->clearResults();
    ROS_DEBUG_NAMED("SceneRecognition", "Flushing buffer of results, already present in shared mem.");
    results_already_shared_.clear();
    ROS_DEBUG_NAMED("SceneRecognition", "Setting configuration file counter back to 0");
    object_constellation_file_counter_ = 0;
    return true;
}

void SceneRecognition::writeObjectConstellationToXML(vector<ObjectPtr> &objects, int scene_recog_count, int constellation_count)
{

    //Don`t do anything if no path was provided that specifies where the configuration should be stored.
    if (object_constellation_folder_path_.empty())
        return;

    if (objects.size() > 0)
    {
        std::ofstream myfile;
        string filePath = object_constellation_folder_path_ + "/" + "recognition_" + to_string(scene_recog_count) + "_constellation_" + to_string(constellation_count) + ".xml";
        myfile.open(filePath);
        myfile << "<Objects>";

        for (unsigned int i = 0; i < objects.size(); ++i)
        {
            ObjectPtr object = objects[i];
            PosePtr pose = object->pose;
            myfile << "<Object "
                   << "type=\"" << object->type
                   << "\" id=\"" << object->observedId
                   << "\" mesh=\"" << object->ressourcePath.string()
                   << "\" angles=\"quaternion\">"
                   << pose->point->eigen.x()
                   << "," << pose->point->eigen.y()
                   << "," << pose->point->eigen.z()
                   << "," << pose->quat->eigen.w()
                   << "," << pose->quat->eigen.x()
                   << "," << pose->quat->eigen.y()
                   << "," << pose->quat->eigen.z()
                   << " </Object>";
        }

        myfile << "</Objects>";
        myfile.close();
    }
}

vector<ObjectPtr> SceneRecognition::extractRealObjects(RecognitionResultPtr result)
{
    vector<ObjectPtr> ret_objects;
    for(ObjectPtr obj : result->recognizedSet->objects)
    {
        if( obj->type.find( result->patternName ) == string::npos)
        {
            ret_objects.push_back(obj);
        }
    }

    vector<ObjectPtr> temp_objects;
    for (RecognitionResultPtr sub_result_ptr : result->subPatterns)
    {
        temp_objects = extractRealObjects(sub_result_ptr);
        ret_objects.insert( ret_objects.end(), temp_objects.begin(), temp_objects.end());
    }
    return ret_objects;
}

bool SceneRecognition::compareConfidence(RecognitionResultPtr i, RecognitionResultPtr j) {
    return (i->confidence < j->confidence);
}

bool SceneRecognition::isCompleteSubset(set<pair<string, string> > set_a, set<pair<string, string> > set_b)
{
    unsigned int counter = 0;
    for (pair<string, string> el_a : set_a)
    {
        for (pair<string, string> el_b : set_b)
        {
            if(!el_a.first.compare(el_b.first) && !el_a.second.compare(el_b.second))
                counter++;
        }
    }
    return counter == set_a.size();
}
}
