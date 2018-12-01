#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Tobias Mueller. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the association nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script broadcasts a TF.
"""

import argparse
import time
import numpy
from math import sin, cos, atan2, hypot, fabs, sqrt, pi

import rospy
import actionlib
import tf
import tf2_ros

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
# from summit_xl_a_msgs.srv import *


class SetpointBroadcaster(object):


    def __init__(self, name):
        '''
        @brief 
        '''

        self._name = name
        self.br = tf2_ros.TransformBroadcaster()
        # using Topic AND Service for handling single and looping pose change
        # self.service = rospy.Service('/agv_mecanum/setpoint_pose', SetPose, self.handle_service)
        self.sub_sp = rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.callbackSetpoint)

        self.setpoint = Path()
        self.t = TransformStamped()
        self.t.header.frame_id = "odom"
        #self.t.header.frame_id = "agv_base_footprint"
        self.t.child_frame_id = "setpoint_pose"
        self.t.transform.translation.x = 0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0
        self.t.transform.rotation.x = 0
        self.t.transform.rotation.y = 0
        self.t.transform.rotation.z = 0
        self.t.transform.rotation.w = 1

        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Start setpoint broadcaster.")

    def cleanup(self):
        '''
        @brief destructor
        '''
        rospy.loginfo("Stop setpoint broadcaster.")
        time.sleep(0.5)

    def callbackSetpoint(self, msg):
	for i in msg.poses: 
		self.t.transform.translation.x = i.pose.position.x
		self.t.transform.translation.y = i.pose.position.y
		self.t.transform.translation.z = i.pose.position.z
		self.t.transform.rotation.x = i.pose.orientation.x
		self.t.transform.rotation.y = i.pose.orientation.y
		self.t.transform.rotation.z = i.pose.orientation.z
		self.t.transform.rotation.w = i.pose.orientation.w

    
    def handle_service(self, req):
        self.t.transform.translation.x = req.poses.pose.pose.position.x
        self.t.transform.translation.y = req.poses.pose.pose.position.y
        self.t.transform.translation.z = req.poses.pose.pose.position.z
        self.t.transform.rotation.x = req.poses.pose.pose.orientation.x
        self.t.transform.rotation.y = req.poses.pose.pose.orientation.y
        self.t.transform.rotation.z = req.poses.pose.pose.orientation.z
        self.t.transform.rotation.w = req.poses.pose.pose.orientation.w
        return SetPoseResponse(True, "New setpoint set.")


    def broadcast(self):
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = __doc__)
    # Accept control rate. Default is 20 hz.
    parser.add_argument("--rate", help="Broadcast rate", type=float, default=20.0)
    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node("setpoint_broadcaster")

        sb = SetpointBroadcaster(rospy.get_name())
        rate = rospy.Rate(args.rate)

        # Keep looping unless the system receives shutdown signal.
        while not rospy.is_shutdown():
            sb.broadcast()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
