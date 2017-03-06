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
#include "asr_recognizer_prediction_ism/pose_prediction/PaperPredictionNormalized.h"
// Bring in gtest
#include <random>
#include <gtest/gtest.h>

// Declare a test
namespace recognizer_prediction_ism
{
    class PaperPredictionNormalizedTest : public ::testing::Test
    {
     protected:

      // Per-test-case set-up.
      // Called before the first test in this test case.
      // Can be omitted if not needed.
        static void SetUpTestCase()
        {
        }

      // Per-test-case tear-down.
      // Called after the last test in this test case.
      // Can be omitted if not needed.
      static void TearDownTestCase()
      {
      }

      // You can define per-test set-up and tear-down logic as usual.
      virtual void SetUp()
      {
          std::string dbfilename = "/home/patrick/catkin_ws/src/ilcasRosFull/perception/scene_understanding/recognizer_prediction_ism/test/rsc/smacks_throne.sqlite";
          double confidence = 1;
          double numberOfSpecifiers = 20;
          patternName= "test";
          confidence = 1;

          referencePose = ISM::PosePtr(new ISM::Pose);
          referencePose->getQuatPtr()->setW(1);
          recognizedSet = ISM::ObjectSetPtr(new ISM::ObjectSet());
          resultPtr = ISM::RecognitionResultPtr(new ISM::RecognitionResult(patternName,referencePose,
                                                                           recognizedSet,confidence,
                                                                           idealPoints,votedPoints));
          recognitionResults.push_back(resultPtr);
          paperPredictionNormalizedPtr = PaperPredictionNormalizedPtr(new PaperPredictionNormalized(confidence,
                                                                                                    numberOfSpecifiers,
                                                                                                    dbfilename));
      }
      virtual void TearDown()
      {

      }

    public:
      std::string  patternName;
      ISM::PosePtr referencePose;
      ISM::ObjectSetPtr recognizedSet;
      std::vector<ISM::PointPtr> idealPoints;
      std::vector<ISM::VotedPointsTypePtr> votedPoints;
      ISM::RecognitionResultPtr resultPtr;

      std::vector<ISM::RecognitionResultPtr> recognitionResults;
      PaperPredictionNormalizedPtr paperPredictionNormalizedPtr;

    };
    TEST_F(PaperPredictionNormalizedTest, databaseTest)
     {
       using namespace ISM;

       PointPtr plateDeepPointPtr(new Point(-0.888345658850255, 0.632926776060424,
                                             0.660864250036386));
       QuaternionPtr plateDeepQuatPtr(new Quaternion(0.651570493634819, -0.643417330721776,
                                                      -0.27727505130885, 0.2908411529908));
       PosePtr plateDeepPosePtr(new Pose(plateDeepPointPtr, plateDeepQuatPtr));

       paperPredictionNormalizedPtr->calcUnfoundPoses(plateDeepPosePtr,patternName,1);

       PointPtr cupPdVPointPtr(new Point(-0.89555632189646,
                                         0.876282245569256,
                                         0.675197306649648));
       QuaternionPtr cupPdVQuatPtr(new Quaternion(0.0437368287079934,
                                                  -0.00284608589628875,
                                                  -0.673893495136102,
                                                  0.737527319373925));
       bool found = false;
       asr_next_best_view::AttributedPointCloud attributedPointCloud = paperPredictionNormalizedPtr->getAttributedPointCloud();
       for (asr_next_best_view::AttributedPoint attributedPoint : attributedPointCloud.elements)
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

         if (distance < 0.003 && angle < 5)
         {
             found = true;
         }
       }
       EXPECT_TRUE(found);
     }

}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
