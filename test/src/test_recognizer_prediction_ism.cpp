/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meissner Pascal, Stoeckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// Bring in my package's API, which is what I'm testing
#include <ros/console.h>
#include "asr_recognizer_prediction_ism/pose_prediction/PosePredictionNode.h"
#include "asr_recognizer_prediction_ism/SceneRecognizerService.h"
// Bring in gtest
#include <random>
#include <gtest/gtest.h>

// Declare a test
namespace recognizer_prediction_ism
{
    class PaperPredictionNORMALIZEDTest : public ::testing::Test
    {
     protected:

      // Per-test-case set-up.
      // Called before the first test in this test case.
      // Can be omitted if not needed.
        static void SetUpTestCase()
        {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
            node_handle = new ros::NodeHandle(ros::this_node::getName());
            node_handle->setParam("dbfilename", "/home/patrick/catkin_ws/src/ilcasRosFull/perception/scene_understanding/recognizer_prediction_ism/test/rsc/smacks_throne.sqlite");
            node_handle->setParam("foundObjectClientName", "/env/asr_world_model/get_found_object_list");
            node_handle->setParam("markerPublisherName", "/rp_ism_node/recognition_prediction");
            node_handle->setParam("PointCloudServerName", "get_point_cloud");
            node_handle->setParam("toggleVisualizationServerName", "recognizer_prediction_toggle_visualization");

            node_handle->setParam("sharedMemoryName", "SharedMemory");
            node_handle->setParam("sharedVectorName", "SharedrecognitionresultVector");
            node_handle->setParam("mutexName", "resultsMutex");
            node_handle->setParam("maxPointCloudSize", 0);
            node_handle->setParam("numberOfSpecifiers", 40);

            node_handle->setParam("enableVisualization", true);

            node_handle->setParam("baseFrame", "/map");
            node_handle->setParam("arrowLifeTime", 0);
            node_handle->setParam("pointLifeTime", 0);
            node_handle->setParam("referenceRadius", 0.1);
            node_handle->setParam("pointRadius", 0.02);
            node_handle->setParam("arrowScaleX", 0.002);
            node_handle->setParam("arrowScaleY", 0.05);
            node_handle->setParam("arrowScaleZ", 0.01);
            node_handle->setParam("colorStepSize", 20);
            node_handle->setParam("latched", true);
        }

      // Per-test-case tear-down.
      // Called after the last test in this test case.
      // Can be omitted if not needed.
      static void TearDownTestCase()
      {
        delete node_handle;
      }

      // You can define per-test set-up and tear-down logic as usual.
      virtual void SetUp()
      {
          using namespace boost::interprocess;
          shared_memory_object::remove("SharedMemory");

          managed_shared_memory segment
             (create_only
             ,"SharedMemory" //segment name
             ,65536);          //segment size in bytes

          //Initialize shared memory STL-compatible allocator
          const RecognitionResultAllocator alloc_inst (segment.get_segment_manager());

          //Construct a shared memory
          segment.construct<SharedrecognitionresultVector>("SharedrecognitionresultVector") //object name
                                     (alloc_inst);//first ctor parameter
          named_mutex::remove("resultsMutex");
          named_mutex mutex(open_or_create, "resultsMutex");
          patternName = "test";
          referencePose = ISM::PosePtr(new ISM::Pose);
          referencePose->getQuatPtr()->setW(1);
          recognizedSet = ISM::ObjectSetPtr(new ISM::ObjectSet());
          confidence = 1;

          resultPtr = ISM::RecognitionResultPtr(new ISM::RecognitionResult(patternName,referencePose,
                                                                           recognizedSet,confidence,
                                                                           idealPoints,votedPoints));
          recognitionResults.push_back(resultPtr);
          posePredictionNodePtr = PosePredictionNodePtr(new PosePredictionNode());
          sceneRecognizer = SceneRecognizerServicePtr(new SceneRecognizerService());

      }
      virtual void TearDown()
      {
        using namespace boost::interprocess;
        shared_memory_object::remove("SharedMemory");
        named_mutex::remove("resultsMutex");
      }

    public:
      static ros::NodeHandle* node_handle;
      std::string patternName;

      ISM::PosePtr referencePose;
      ISM::ObjectSetPtr recognizedSet;
      std::vector<ISM::PointPtr> idealPoints;
      std::vector<ISM::VotedPointsTypePtr> votedPoints;
      ISM::RecognitionResultPtr resultPtr;
      double confidence;
      std::vector<ISM::RecognitionResultPtr> recognitionResults;
      asr_recognizer_prediction_ism::GetPointCloud::Request req;
      asr_recognizer_prediction_ism::GetPointCloud::Response res;
      PosePredictionNodePtr posePredictionNodePtr;
      SceneRecognizerServicePtr sceneRecognizer;

    };

    ros::NodeHandle * PaperPredictionNORMALIZEDTest::node_handle = NULL;
    TEST_F(PaperPredictionNORMALIZEDTest, databaseTest)
     {
       using namespace ISM;

       PointPtr plateDeepPointPtr(new Point(-0.888345658850255, 0.632926776060424,
                                             0.660864250036386));
       QuaternionPtr plateDeepQuatPtr(new Quaternion(0.651570493634819, -0.643417330721776,
                                                      -0.27727505130885, 0.2908411529908));
       PosePtr plateDeepPosePtr(new Pose(plateDeepPointPtr, plateDeepQuatPtr));
       ObjectPtr objectPtr(new Object("PlateDeep", plateDeepPosePtr));
       recognizedSet->insert(objectPtr);
       double confidence = 0.333;
       RecognitionResult r(patternName, plateDeepPosePtr,recognizedSet,confidence,idealPoints,votedPoints);

       EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
       posePredictionNodePtr->getSharedMemoryManagerPtr()->addResult(r);
       EXPECT_EQ(1, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());


       posePredictionNodePtr->getPointCloud(req,res);
       EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
       PointPtr cupPdVPointPtr(new Point(-0.89555632189646,
                                         0.876282245569256,
                                         0.675197306649648));
       QuaternionPtr cupPdVQuatPtr(new Quaternion(0.0437368287079934,
                                                  -0.00284608589628875,
                                                  -0.673893495136102,
                                                  0.737527319373925));
       bool found = false;
       for (asr_msgs::AsrAttributedPoint attributedPoint : res.point_cloud.elements)
       {
         double x = attributedPoint.pose.position.x;
         double y = attributedPoint.pose.position.y;
         double z = attributedPoint.pose.position.z;
         double qw = attributedPoint.pose.orientation.w;
         double qx = attributedPoint.pose.orientation.x;
         double qy = attributedPoint.pose.orientation.y;
         double qz = attributedPoint.pose.orientation.z;

         PointPtr currentPointPtr(new Point(x,y,z));
         QuaternionPtr currentQuaternionPtr(new Quaternion(qw, qx, qy, qz));
         double distance = GeometryHelper::getDistanceBetweenPoints(currentPointPtr,cupPdVPointPtr);
         double angle = GeometryHelper::getAngleBetweenQuats(currentQuaternionPtr, cupPdVQuatPtr);

         if (distance < 0.03 && angle < 5)
         {
             //ROS_INFO("distance: %f, angle: %f", distance, angle);
             found = true;
         }
       }
       EXPECT_TRUE(found);
     }
    TEST_F(PaperPredictionNORMALIZEDTest, zeroPoseRecognizerPrediction)
    {
      EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
      posePredictionNodePtr->getSharedMemoryManagerPtr()->addResult(*resultPtr);
      EXPECT_EQ(1, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
      posePredictionNodePtr->getPointCloud(req,res);
      EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
      EXPECT_LT(0, res.point_cloud.elements.size());

      asr_msgs::AsrAttributedPointCloud pointCloud1 = res.point_cloud;
      posePredictionNodePtr->getSharedMemoryManagerPtr()->addResult(*resultPtr);
      posePredictionNodePtr->getPointCloud(req,res);
      asr_msgs::AsrAttributedPointCloud pointCloud2 = res.point_cloud;
      EXPECT_GE(pointCloud1.elements.size(), pointCloud2.elements.size());
    }

    TEST_F(PaperPredictionNORMALIZEDTest, sharedMemSceneRecognizerPrediction)
    {
        EXPECT_EQ(0, sceneRecognizer->getResultsNumber());
        sceneRecognizer->addRecognitionResults(recognitionResults);
        EXPECT_EQ(1, sceneRecognizer->getResultsNumber());
    }

    TEST_F(PaperPredictionNORMALIZEDTest, sharedMemTest)
    {
        EXPECT_EQ(0, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());

        sceneRecognizer->addRecognitionResults(recognitionResults);

        EXPECT_EQ(1, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(1, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
    }
    TEST_F(PaperPredictionNORMALIZEDTest, sortTest)
    {
        EXPECT_EQ(0, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(0, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());

        unsigned int size = 100;
        while (recognitionResults.size() < size)
        {
           double lower_bound = 0;
           double upper_bound = 1;
           std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
           std::default_random_engine re;
           double random_confidence = unif(re);
            ISM::RecognitionResultPtr resultPtr(new ISM::RecognitionResult(patternName,referencePose,
                                                                           recognizedSet,random_confidence,
                                                                           idealPoints,votedPoints));
           recognitionResults.push_back(resultPtr);
        }

        sceneRecognizer->addRecognitionResults(recognitionResults);

        EXPECT_EQ(size, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(size, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());

        posePredictionNodePtr->getSharedMemoryManagerPtr()->sortResults();

        EXPECT_EQ(size, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(size, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());

        std::vector<ISM::RecognitionResult> sortedResults = posePredictionNodePtr->getSharedMemoryManagerPtr()->getResults();

        double confidence = 0;
        for (unsigned int i = 0; i < sortedResults.size(); i++)
        {
            EXPECT_LE(confidence, sortedResults.at(i).getConfidence());
            confidence = sortedResults.at(i).getConfidence();
        }

        EXPECT_EQ(size, sceneRecognizer->getResultsNumber());
        EXPECT_EQ(size, posePredictionNodePtr->getSharedMemoryManagerPtr()->getResultsNumber());
        ISM::RecognitionResult best = posePredictionNodePtr->getSharedMemoryManagerPtr()->sortListAndgetBest();
        EXPECT_EQ(1.0, best.getConfidence());
    }
  //Please write a percentage_of_records_for_prediction test! We had no time left.
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_recognizer_prediction");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
