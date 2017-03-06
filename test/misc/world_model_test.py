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

import roslib
import rospy
from asr_msgs.msg import AsrObject
from asr_world_model.srv import PushFoundObject, PushFoundObjectRequest
from geometry_msgs.msg import PoseWithCovariance



def main():
    req = PushFoundObjectRequest()
    obj = AsrObject()
    obj.header.frame_id = '/camera_left_frame'
    obj.type = 'Smacks'
    obj.providedBy = 'textured'
    obj.colorName = 'textured'
    obj.meshResourcePath = 'package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae'
    pose = PoseWithCovariance()
    pose.pose.position.x = -1.20467103562224
    pose.pose.position.y = 0.589709102059754
    pose.pose.position.z = 1.02933642432456
    pose.pose.orientation.w = 0.573287415807964
    pose.pose.orientation.x = -0.58793291283527
    pose.pose.orientation.y = -0.394117786539212
    pose.pose.orientation.z = 0.412731873272096
    obj.sampledPoses.append(pose)

    try:
        rospy.wait_for_service('/env/asr_world_model/push_found_object', timeout=5)
    except rospy.exceptions.ROSException, e:
        rospy.loginfo('Could not reach service')

    try:
        push_found_object = rospy.ServiceProxy('/env/asr_world_model/push_found_object',
                                    PushFoundObject)

        push_found_object(obj)
    except rospy.ServiceException, e:
        rospy.loginfo('Could not push object')



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
