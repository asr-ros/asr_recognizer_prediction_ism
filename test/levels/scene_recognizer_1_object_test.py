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
    obj.type = 'Knaeckebrot'
    obj.header.frame_id = '/camera_left_frame'
    obj.identifier = ''
    obj.providedBy = 'textured'
    obj.colorName = 'textured'
    obj.meshResourcePath = 'package://asr_object_database/rsc/databases/textured_objects/Knaeckebrot/Knaeckebrot.dae'
    obj.poseEstimation.pose.position.x = 0.15
    obj.poseEstimation.pose.position.y = 0.15
    obj.poseEstimation.pose.position.z = 0
    obj.poseEstimation.pose.orientation.w = 1
    obj.poseEstimation.pose.orientation.x = 0
    obj.poseEstimation.pose.orientation.y = 0
    obj.poseEstimation.pose.orientation.z = 0

    objects.append(obj)

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
