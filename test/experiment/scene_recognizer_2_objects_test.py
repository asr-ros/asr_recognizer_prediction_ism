#!/usr/bin/env python

'''
Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meissner Pascal, Stoeckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from asr_msgs.msg import AsrObject
from asr_recognizer_prediction_ism.srv import FindScenes, FindScenesRequest
import roslib
import rospy


def main():
    req = FindScenesRequest()
    objects = []
    obj = AsrObject()
    obj.type = 'CupPdV'


    obj.poseEstimation.pose.position.x = -0.895672837120925
    obj.poseEstimation.pose.position.y = 0.876364548041306
    obj.poseEstimation.pose.position.z = 0.675630282005366
    obj.poseEstimation.pose.orientation.w = 0.0438159930101303
    obj.poseEstimation.pose.orientation.x = -0.00274503713451766
    obj.poseEstimation.pose.orientation.y = -0.673982743489664
    obj.poseEstimation.pose.orientation.z = 0.737441445137044

    obj1 = AsrObject()
    obj1.type = 'PlateDeep'
    obj1.poseEstimation.pose.position.x = -0.884891287861248
    obj1.poseEstimation.pose.position.y = 0.639571172744163
    obj1.poseEstimation.pose.position.z = 0.660505174542679
    obj1.poseEstimation.pose.orientation.w = 0.650782548685679
    obj1.poseEstimation.pose.orientation.x = -0.64554621902581
    obj1.poseEstimation.pose.orientation.y = -0.27131992194386
    obj1.poseEstimation.pose.orientation.z = 0.293492169203933

    objects.append(obj)
    objects.append(obj1)

    try:
        rospy.wait_for_service('/rp_ism_node/find_scenes', timeout=5)
    except rospy.exceptions.ROSException, e:
        rospy.loginfo('Could not reach service')

    try:
        find_scenes = rospy.ServiceProxy('/rp_ism_node/find_scenes',
                                    FindScenes)
        print find_scenes(objects)
    except rospy.ServiceException, e:
        rospy.loginfo('Could not find scene')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
