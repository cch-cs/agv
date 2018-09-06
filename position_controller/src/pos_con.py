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
This script is used to control the velocity with position setpoints.
"""
# Python imports
import argparse
import time
import numpy
#import math
from math import sin, cos, atan2, hypot, fabs, sqrt, pi

# ROS imports
import rospy
import tf
import tf2_ros

# Messages
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from std_msgs.msg import Float64, Bool

# Services
from std_srvs.srv import *

class PositionController(object):


    def __init__(self, name):
        '''
        @brief
        '''

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.service = rospy.Service('/summit_xl_a/enable_pos_ctrl', SetBool, self.handle_service)
        self.activator_x = rospy.Publisher("/summit_xl_a/pid_x/pid_enable", Bool, queue_size=1)
        self.activator_y = rospy.Publisher("/summit_xl_a/pid_y/pid_enable", Bool, queue_size=1)
        self.activator_yaw = rospy.Publisher("/summit_xl_a/pid_yaw/pid_enable", Bool, queue_size=1)
        self.sub_pid_x_effort = rospy.Subscriber("/summit_xl_a/pid_x/control_effort", Float64, self.callback_x)
        self.sub_pid_y_effort = rospy.Subscriber("/summit_xl_a/pid_y/control_effort", Float64, self.callback_y)
        self.sub_pid_yaw_effort = rospy.Subscriber("/summit_xl_a/pid_yaw/control_effort", Float64, self.callback_yaw)
        self.pub_pid_x_state = rospy.Publisher("/summit_xl_a/pid_x/state", Float64, queue_size=1)
        self.pub_pid_y_state = rospy.Publisher("/summit_xl_a/pid_y/state", Float64, queue_size=1)
        self.pub_pid_yaw_state = rospy.Publisher("/summit_xl_a/pid_yaw/state", Float64, queue_size=1)
        self.pub_pid_x_setpoint = rospy.Publisher("/summit_xl_a/pid_x/setpoint", Float64, queue_size=1)
        self.pub_pid_y_setpoint = rospy.Publisher("/summit_xl_a/pid_y/setpoint", Float64, queue_size=1)
        self.pub_pid_yaw_setpoint = rospy.Publisher("/summit_xl_a/pid_yaw/setpoint", Float64, queue_size=1)
        self.pub_cmd = rospy.Publisher("/summit_xl_a/cmd_vel", Twist, queue_size=1)

        self.cmd = Twist()
        self.pos_x = .0
        self.pos_y = .0
        self.yaw = .0

        self.sp_pos_x = Float64()
        self.sp_pos_y = Float64()
        self.sp_yaw = Float64()

        self.control_effort_x = Float64()
        self.control_effort_y = Float64()
        self.control_effort_yaw = Float64()

        self.pid_enabled = True 
        self.yawflag = False
        self.posflagX = False
        self.posflagY = False

        self.targetPos = TransformStamped()
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Start position controller.")


    def cleanup(self):
        '''
        @brief destructor
        '''
        self.handle_service(False)
        rospy.loginfo("Stop position controller")


    def lookupTransform(self, parent, child):
        try:
            trans = self.tfBuffer.lookup_transform(parent, child, rospy.Time(0))			#rospy.Time(0) for latest available transform
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def pubTransform(self, inp_x, inp_y, inp_yaw):        
        self.targetPos.header.stamp = rospy.Time(0)
        # pc.targetPos.header.frame_id = "summit_xl_a_odom" 
        self.targetPos.child_frame_id = "summit_xl_a_base_footprint" 
        self.targetPos.transform.translation.x = inp_x            
        self.targetPos.transform.translation.y = inp_y            
        quatr = tf.transformations.quaternion_from_euler(0,0,inp_yaw)
        print "Quaternion angle = "
        print quatr
        self.targetPos.transform.rotation = quatr
        # pc.targetPos.sendTransform((sp_x,sp_y,0),tf.transformations.quaternion_from_euler(0,0,sp_yaw), rospy.Time.now(), "summit_xl_a_base_footprint", "summit_xl_a_odom")

    def atSetpointPosX(self):
        print "Entering atSetpointX" # debug
        deadband = 0.01
        disp = False                                #split x y into separate functions as euclidean averages badly
        targetdisp = self.targetPos.transform.translation.x
        disp = self.lookupTransform("summit_xl_a_odom", "summit_xl_a_base_footprint")
        print "TargetX:"
        print self.targetPos.transform.translation.x

        if disp:
            displacement = disp.transform.translation.x
            print "CurrX:"
            print disp.transform.translation.x

            real_error = displacement - targetdisp
            error = round(real_error, 2)

            if abs(error) <= deadband:
                self.posflagX = True
                print "X reached setpoint value. returning TRUE" # debug 
                return True
            else:
                self.posflagX = False
                print "x NOT at setpoint value. returning FALSE" # debug
                return False
        else:
            return False

    def atSetpointPosY(self):
        print "Entering atSetpointY" # debug
        deadband = 0.01
        disp = False                                #split x y into separate functions as euclidean averages badly
        targetdisp = self.targetPos.transform.translation.y
        disp = self.lookupTransform("summit_xl_a_odom", "summit_xl_a_base_footprint")
        print "TargetY:"
        print self.targetPos.transform.translation.y
        
        if disp:
            displacement = disp.transform.translation.y
            print "CurrY:"
            print disp.transform.translation.y

            real_error = targetdisp - displacement          
            error = round(real_error, 2)
            print "Y Error:"
            print error

            if abs(error) <= deadband:
                self.posflagY = True
                print "Y reached setpoint value. returning TRUE" # debug          
                return True
            else:
                self.posflagY = False
                print "Y NOT at setpoint value. returning FALSE" # debug
                return False
        else:
            return False

    def atSetpointYaw(self):
        print "Entering atSetpointYaw" # debug
        deadband = 0.01   #0.17453 # 10 deg in rad
        disp = False
        targetquat = self.targetPos.transform.rotation
        targeteuler = tf.transformations.euler_from_quaternion(targetquat)
        targetyaw = targeteuler[2]
        disp = self.lookupTransform("summit_xl_a_odom", "summit_xl_a_base_footprint")
        print "TargetYaw:"
        print targetyaw

        if disp:
            quat = [disp.transform.rotation.x, disp.transform.rotation.y,
                    disp.transform.rotation.z, disp.transform.rotation.w]
            euler = tf.transformations.euler_from_quaternion(quat)
            yaw = euler[2]
            print "CurrYaw:"
            print yaw

            real_error = yaw - targetyaw
            error = round(real_error, 2)
            print "Yaw Error" # debug
            print error # debug

            if abs(error) <= deadband:
                self.yawflag = True
                print "Yaw reached setpoint value. returning TRUE" # debug
                return True
            else:
                self.yawflag = False
                print "Yaw NOT at setpoint value. returning FALSE" # debug
                return False
                
        else:
            return False

    def control(self):
        '''
        @brief
        '''
        if self.pid_enabled:
            setpoint = False
            trans = False

            # assign goal
            setpoint = self.targetPos
            trans = self.lookupTransform("summit_xl_a_odom", "summit_xl_a_base_footprint")

            if setpoint and trans:
                
                #setpoint

                #self.pub_pid_x_setpoint.publish(0)													#########check for x y flip!!!
                #self.pub_pid_y_setpoint.publish(0)
                #self.pub_pid_yaw_setpoint.publish(0)
                self.sp_pos_x = setpoint.transform.translation.x
                self.sp_pos_y = setpoint.transform.translation.y
                quat_sp = setpoint.transform.rotation 
                euler_sp = tf.transformations.euler_from_quaternion(quat_sp)
                self.sp_yaw = euler_sp[2]
               
                self.pub_pid_x_setpoint.publish(self.sp_pos_x)
                self.pub_pid_y_setpoint.publish(self.sp_pos_y)
                self.pub_pid_yaw_setpoint.publish(self.sp_yaw)

                #trans

                self.pos_x = trans.transform.translation.x
                self.pos_y = trans.transform.translation.y
                quat = [trans.transform.rotation.x, trans.transform.rotation.y,
                        trans.transform.rotation.z, trans.transform.rotation.w]
                euler = tf.transformations.euler_from_quaternion(quat)
                self.yaw = euler[2]
                self.pub_pid_x_state.publish(self.pos_x)
                self.pub_pid_y_state.publish(self.pos_y)
                self.pub_pid_yaw_state.publish(self.yaw)
               
                if not self.atSetpointYaw():
                    self.cmd.linear.x = 0
                    self.cmd.linear.y = 0
                    self.cmd.angular.z = self.control_effort_yaw.data
                    self.pub_cmd.publish(self.cmd)

                elif not self.atSetpointPosY():
                    self.cmd.linear.y = self.control_effort_y.data
                    self.cmd.linear.x = 0
                    self.cmd.angular.z = 0
                    # self.cmd.linear.y = self.control_effort_x.data                    
                    # self.cmd.angular.z = self.control_effort_yaw.data
                    self.pub_cmd.publish(self.cmd)

                elif not self.atSetpointPosX():
                    self.cmd.linear.x = self.control_effort_x.data                    
                    self.cmd.linear.y = self.control_effort_y.data                    
                    self.cmd.angular.z = self.control_effort_yaw.data
                    self.pub_cmd.publish(self.cmd)
 
                if self.posflagX and self.posflagY and self.yawflag:
                    self.handle_service(False)
                    print("shutting down PID")
                    return True
                else:
                    return False

    def callback_x(self, msg):
        self.control_effort_x = msg

    def callback_y(self, msg):
        self.control_effort_y = msg

    def callback_yaw(self, msg):
        self.control_effort_yaw = msg

    def handle_service(self, req):
        rate = rospy.Rate(25)
        #check if flag is toggled
        if req != self.pid_enabled:
            # toggle PID controller to prevent integration
            # dirtyfix: for loop secures connection between sub and pub
            for i in range(1, 5):
                self.activator_x.publish(req)
                self.activator_y.publish(req)
                self.activator_yaw.publish(req)
                rate.sleep()
            # set control effort equal zero
            self.cmd.linear.x = 0
            self.cmd.linear.y = 0
            self.cmd.angular.z = 0
            self.pub_cmd.publish(self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.pub_cmd.publish(self.cmd)
            self.pid_enabled = req
            return SetBoolResponse(True, "Positioncontroller set to {}".format(req))
        return SetBoolResponse(True, "Positioncontroller was set to {}".format(req))

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        inp_x = float(sys.argv[1])
        inp_y = float(sys.argv[2])
        inp_yaw = float(sys.argv[3])
        
        parser = argparse.ArgumentParser(description = __doc__)
        # Accept control rate. Default is 50 hz.
        parser.add_argument("--rate", help="Controller rate", type=float, default=50.0)
        args, unknown = parser.parse_known_args()

        try:
            rospy.init_node("summit_xl_a_position_controller")

            pc = PositionController(rospy.get_name())
            rate = rospy.Rate(args.rate)

            # Publishing the transform over tf
            pc.pubTransform(inp_x, inp_y, inp_yaw)
            
            # Keep looping unless the system receives shutdown signal.
            while not rospy.is_shutdown():
                pc.control()
                rate.sleep()

        except rospy.ROSInterruptException:
            pass
        rospy.spin()

    else:
        print usage()
        sys.exit(1)
