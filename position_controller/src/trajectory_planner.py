#!/usr/bin/env python

"""
This script publishes a sample trajectory
"""

import argparse
import time
import math

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64

class trajectoryPlanner(object):

    def __init__(self, name):

        self._name = name
        self.br = tf2_ros.TransformBroadcaster()
        self.tr_pub = rospy.Publisher("/agv_mecanum/sp_pose", PoseStamped, queue_size=1)
        
        self.setpoint = PoseStamped()
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = 0.0
        self.setpoint.pose.orientation.x = 0.0
        self.setpoint.pose.orientation.y = 0.0
        self.setpoint.pose.orientation.z = 0.0
        self.setpoint.pose.orientation.w = 1.0

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Start Trajectory planner")

    def cleanup(self):
        '''
        @brief destructor
        '''
        rospy.loginfo("Stop Trajectory planner")
        rospy.sleep(0.5)

    def broadcast(self, x , y, yaw):
        yaw = yaw * math.pi / 180.0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        #print x, y, quat 
        
        self.setpoint.pose.position.x = x
        self.setpoint.pose.position.y = y
        self.setpoint.pose.orientation.x = quat[0]
        self.setpoint.pose.orientation.y = quat[1]
        self.setpoint.pose.orientation.z = quat[2]
        self.setpoint.pose.orientation.w = quat[3]
        #self.setpoint.header.stamp = rospy.Time.now()
        #print self.setpoint
        self.tr_pub.publish(self.setpoint)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", help="Broadcast Rate", type=float, default=5.0)
    parser.add_argument("--points", help="Trajectory points", type=int, default=1)
    args, unknown = parser.parse_known_args()
    
    try:
        rospy.init_node("Trajectory_planner")

        tp = trajectoryPlanner(rospy.get_name())
        rate = rospy.Rate(args.rate)
        i = 0
        x = [1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0]
        y = [2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.5, 3.5, 4.0, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.0, 8.0]
        #yaw = [0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        yaw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.0 ]

        while not rospy.is_shutdown():
            if (i <= len(x) - 1):
                tp.broadcast(x[i],y[i],yaw[i])
            #tp.tr_pub.publish(tp.setpoint)
            # print i
                i += 1
                rate.sleep()
            else:
                break
        #rate.sleep()


    except rospy.ROSInterruptException:
        pass
    # rospy.spin()

