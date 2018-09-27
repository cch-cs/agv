 /*
     Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
     All rights reserved.
 
     Redistribution and use in source and binary forms, with or without
     modification, are permitted provided that the following conditions are met:
         * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
         * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
         * Neither the name of the <organization> nor the
         names of its contributors may be used to endorse or promote products
         derived from this software without specific prior written permission.
 
     THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
     EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
     DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
     DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
     ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */
 
 /*
  * \file  gazebo_ros_diff_drive.cpp
  *
  * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
  * developed for the erratic robot (see copyright notice above). The original
  * plugin can be found in the ROS package gazebo_erratic_plugins.
  *
  * \author  Piyush Khandelwal (piyushk@gmail.com)
  *
  * $ Id: 06/21/2013 11:23:40 AM piyushk $
  */
 
 
 /*
  *
  * The support of acceleration limit was added by
  * \author   George Todoran <todorangrg@gmail.com>
  * \author   Markus Bader <markus.bader@tuwien.ac.at>
  * \date 22th of May 2014
  */
 
 #include <algorithm>
 #include <assert.h>
 
 #include "fourwheel_drive_visual_wheel.h"
 
 #include <ignition/math/Angle.hh>
 #include <ignition/math/Pose3.hh>
 #include <ignition/math/Quaternion.hh>
 #include <ignition/math/Vector3.hh>
 #include <sdf/sdf.hh>
 
 #include <ros/ros.h>
 
 namespace gazebo
 {
 
 enum {
 	  FRONT_RIGHT,
     FRONT_LEFT,
     BACK_RIGHT,
     BACK_LEFT,
 };
 
 GazeboRosfourwheelvisualDrive::GazeboRosfourwheelvisualDrive() {}
 
 // Destructor
 GazeboRosfourwheelvisualDrive::~GazeboRosfourwheelvisualDrive() {}
 
 // Load the controller
 void GazeboRosfourwheelvisualDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
 {
 
     this->parent = _parent;
     gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "GazeboRosfourwheelvisualDrive" ) );
     // Make sure the ROS node for Gazebo has already been initialized
     gazebo_ros_->isInitialized();
 
     gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
     gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
     gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
     gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
     gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", false); //default is true
     gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", true ); //default is false
     gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
     gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
     gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
     gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
     std::map<std::string, OdomSource> odomOptions;
     odomOptions["encoder"] = ENCODER;
     odomOptions["world"] = WORLD;
     gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );
 
 
     joints_.resize ( 4 );
     joints_[FRONT_LEFT] = gazebo_ros_->getJoint ( parent, "frontleftJoint", "front_left_joint" );
     joints_[FRONT_RIGHT] = gazebo_ros_->getJoint ( parent, "frontrightJoint", "front_right_joint" );
     joints_[BACK_LEFT] = gazebo_ros_->getJoint ( parent, "backleftJoint", "back_left_joint" );
     joints_[BACK_RIGHT] = gazebo_ros_->getJoint ( parent, "backrightJoint", "back_right_joint" );
     joints_[FRONT_LEFT]->SetParam ( "fmax", 0, wheel_torque );
     joints_[FRONT_RIGHT]->SetParam ( "fmax", 0, wheel_torque );
     joints_[BACK_LEFT]->SetParam ( "fmax", 0, wheel_torque );
     joints_[BACK_RIGHT]->SetParam ( "fmax", 0, wheel_torque );
 
 
 
     this->publish_tf_ = true;
     if (!_sdf->HasElement("publishTf")) {
       ROS_WARN_NAMED("fourwheelvisualDrive", "GazeboRosfourwheelvisualDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
           this->robot_namespace_.c_str(), this->publish_tf_);
     } else {
       this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
     }
 
     // Initialize update rate stuff
     if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
     else this->update_period_ = 0.0;
 #if GAZEBO_MAJOR_VERSION >= 8
     last_update_time_ = parent->GetWorld()->SimTime();
 #else
     last_update_time_ = parent->GetWorld()->GetSimTime();
 #endif
 
     // Initialize velocity stuff
     wheel_speed_[FRONT_RIGHT] = 0;
     wheel_speed_[FRONT_LEFT] = 0;
     wheel_speed_[BACK_RIGHT] = 0;
     wheel_speed_[BACK_LEFT] = 0;
 
     // Initialize velocity support stuff
     wheel_speed_instr_[FRONT_RIGHT] = 0;
     wheel_speed_instr_[FRONT_LEFT] = 0;
     wheel_speed_instr_[BACK_RIGHT] = 0;
     wheel_speed_instr_[BACK_LEFT] = 0;
 
     x_ = 0;
     y_ = 0;
     rot_ = 0;
     alive_ = true;
 
 
     if (this->publishWheelJointState_)
     {
         joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
         //ROS_INFO_NAMED("fourwheel_visual_drive", "%s: Advertise joint_states", gazebo_ros_->info());
     }
 
     transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
 
     // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
     //ROS_INFO_NAMED("fourwheel_visual_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
 
     ros::SubscribeOptions so =
         ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                 boost::bind(&GazeboRosfourwheelvisualDrive::cmdVelCallback, this, _1),
                 ros::VoidPtr(), &queue_);
 
     cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
     //ROS_INFO_NAMED("fourwheel_visual_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
 
     if (this->publish_tf_)
     {
       //odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
       //ROS_INFO_NAMED("fourwheel_visual_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
     }
 
     // start custom queue for diff drive
     this->callback_queue_thread_ =
         boost::thread ( boost::bind ( &GazeboRosfourwheelvisualDrive::QueueThread, this ) );
 
     // listen to the update event (broadcast every simulation iteration)
     this->update_connection_ =
         event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosfourwheelvisualDrive::UpdateChild, this ) );
 
 }
 
 void GazeboRosfourwheelvisualDrive::Reset()
 {
 #if GAZEBO_MAJOR_VERSION >= 8
   last_update_time_ = parent->GetWorld()->SimTime();
 #else
   last_update_time_ = parent->GetWorld()->GetSimTime();
 #endif
   pose_encoder_.x = 0;
   pose_encoder_.y = 0;
   pose_encoder_.theta = 0;
   x_ = 0;
   y_ = 0;
   rot_ = 0;
   joints_[FRONT_LEFT]->SetParam ( "fmax", 0, wheel_torque );
   joints_[FRONT_RIGHT]->SetParam ( "fmax", 0, wheel_torque );
   joints_[BACK_LEFT]->SetParam ( "fmax", 0, wheel_torque );
   joints_[BACK_RIGHT]->SetParam ( "fmax", 0, wheel_torque );
 }
 
 void GazeboRosfourwheelvisualDrive::publishWheelJointState()
 {
     ros::Time current_time = ros::Time::now();
 
     joint_state_.header.stamp = current_time;
     joint_state_.name.resize ( joints_.size() );
     joint_state_.position.resize ( joints_.size() );
 
     for ( int i = 0; i < 4; i++ ) {
         physics::JointPtr joint = joints_[i];
 #if GAZEBO_MAJOR_VERSION >= 8
         double position = joint->Position ( 0 );
 #else
         double position = joint->GetAngle ( 0 ).Radian();
 #endif
         joint_state_.name[i] = joint->GetName();
         joint_state_.position[i] = position;
     }
     joint_state_publisher_.publish ( joint_state_ );
 }
 
 void GazeboRosfourwheelvisualDrive::publishWheelTF()
 {
     ros::Time current_time = ros::Time::now();
     for ( int i = 0; i < 4; i++ ) {
 
         std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
         std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());
 
 #if GAZEBO_MAJOR_VERSION >= 8
         ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
 #else
         ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
 #endif
 
         tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
         tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );
 
         tf::Transform tfWheel ( qt, vt );
         transform_broadcaster_->sendTransform (
             tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
     }
 }
 
 // Update the controller
 void GazeboRosfourwheelvisualDrive::UpdateChild()
 {
 
     /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
        https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
        (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
        and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboRosDiffDrive::Reset
        (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
     */
     for ( int i = 0; i < 4; i++ ) {
       if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
         joints_[i]->SetParam ( "fmax", 0, wheel_torque );
       }
     }
 
 
     if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
 #if GAZEBO_MAJOR_VERSION >= 8
     common::Time current_time = parent->GetWorld()->SimTime();
 #else
     common::Time current_time = parent->GetWorld()->GetSimTime();
 #endif
     double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
 
     if ( seconds_since_last_update > update_period_ ) {
         //if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
         if ( publishWheelTF_ ) publishWheelTF();
         if ( publishWheelJointState_ ) publishWheelJointState();
 
         // Update robot in case new velocities have been requested
         getWheelVelocities();
 
         double current_speed[4];
 
         current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
         current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
         current_speed[BACK_LEFT] = joints_[BACK_LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
         current_speed[BACK_RIGHT] = joints_[BACK_RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
 
         if ( wheel_accel == 0 ||
                 ( fabs ( wheel_speed_[FRONT_LEFT] - current_speed[FRONT_LEFT] ) < 0.01 ) ||
                 ( fabs ( wheel_speed_[FRONT_RIGHT] - current_speed[FRONT_RIGHT] ) < 0.01 ) ||
                 ( fabs ( wheel_speed_[BACK_LEFT] - current_speed[BACK_LEFT] ) < 0.01 ) ||
                 ( fabs ( wheel_speed_[BACK_RIGHT] - current_speed[BACK_RIGHT] ) < 0.01 ) ) {
             //if max_accel == 0, or target speed is reached
             joints_[FRONT_LEFT]->SetParam ( "vel", 0, wheel_speed_[FRONT_LEFT]/ ( wheel_diameter_ / 2.0 ) );
             joints_[FRONT_RIGHT]->SetParam ( "vel", 0, wheel_speed_[FRONT_RIGHT]/ ( wheel_diameter_ / 2.0 ) );
             joints_[BACK_LEFT]->SetParam ( "vel", 0, wheel_speed_[BACK_LEFT]/ ( wheel_diameter_ / 2.0 ) );
             joints_[BACK_RIGHT]->SetParam ( "vel", 0, wheel_speed_[BACK_RIGHT]/ ( wheel_diameter_ / 2.0 ) );
         } else {
             if ( wheel_speed_[FRONT_LEFT]>=current_speed[FRONT_LEFT] )
                 wheel_speed_instr_[FRONT_LEFT]+=fmin ( wheel_speed_[FRONT_LEFT]-current_speed[FRONT_LEFT],  wheel_accel * seconds_since_last_update );
             else
                 wheel_speed_instr_[FRONT_LEFT]+=fmax ( wheel_speed_[FRONT_LEFT]-current_speed[FRONT_LEFT], -wheel_accel * seconds_since_last_update );
 
             if ( wheel_speed_[FRONT_RIGHT]>current_speed[FRONT_RIGHT] )
                 wheel_speed_instr_[FRONT_RIGHT]+=fmin ( wheel_speed_[FRONT_RIGHT]-current_speed[FRONT_RIGHT], wheel_accel * seconds_since_last_update );
             else
                 wheel_speed_instr_[FRONT_RIGHT]+=fmax ( wheel_speed_[FRONT_RIGHT]-current_speed[FRONT_RIGHT], -wheel_accel * seconds_since_last_update );
                 
             if ( wheel_speed_[BACK_LEFT]>=current_speed[BACK_LEFT] )
                 wheel_speed_instr_[BACK_LEFT]+=fmin ( wheel_speed_[BACK_LEFT]-current_speed[BACK_LEFT],  wheel_accel * seconds_since_last_update );
             else
                 wheel_speed_instr_[BACK_LEFT]+=fmax ( wheel_speed_[BACK_LEFT]-current_speed[BACK_LEFT], -wheel_accel * seconds_since_last_update );
 
             if ( wheel_speed_[BACK_RIGHT]>current_speed[BACK_RIGHT] )
                 wheel_speed_instr_[BACK_RIGHT]+=fmin ( wheel_speed_[BACK_RIGHT]-current_speed[BACK_RIGHT], wheel_accel * seconds_since_last_update );
             else
                 wheel_speed_instr_[BACK_RIGHT]+=fmax ( wheel_speed_[BACK_RIGHT]-current_speed[BACK_RIGHT], -wheel_accel * seconds_since_last_update );
 
             // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
             // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);
 
             joints_[FRONT_LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[FRONT_LEFT] / ( wheel_diameter_ / 2.0 ) );
             joints_[FRONT_RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[FRONT_RIGHT] / ( wheel_diameter_ / 2.0 ) );
             joints_[BACK_LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[BACK_LEFT] / ( wheel_diameter_ / 2.0 ) );
             joints_[BACK_RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[BACK_RIGHT] / ( wheel_diameter_ / 2.0 ) );
         }
         last_update_time_+= common::Time ( update_period_ );
     }
 }
 
 // Finalize the controller
 void GazeboRosfourwheelvisualDrive::FiniChild()
 {
     alive_ = false;
     queue_.clear();
     queue_.disable();
     gazebo_ros_->node()->shutdown();
     callback_queue_thread_.join();
 }
 
 void GazeboRosfourwheelvisualDrive::getWheelVelocities()
 {
     boost::mutex::scoped_lock scoped_lock ( lock );
 
     double vx = x_;
     double vy = y_;
     double va = rot_;
     if ((va < 0.01) && (va > -0.01))
     {
     	if ( (vx > 0 || vx < 0) && (vy < 0.01 && vy > -0.01) )
     	{
     		wheel_speed_[FRONT_LEFT] = vx;
     		wheel_speed_[FRONT_RIGHT] = vx;
     		wheel_speed_[BACK_LEFT] = vx;
     		wheel_speed_[BACK_RIGHT] = vx;
     	}
     	else if ( (vx < 0.01 && vx > -0.01) && (vy > 0 || vy < 0) )
     	{
     		wheel_speed_[FRONT_LEFT] = vy;
     		wheel_speed_[FRONT_RIGHT] = -vy;
     		wheel_speed_[BACK_LEFT] = -vy;
     		wheel_speed_[BACK_RIGHT] = vy;
     	}
     	else if ( vx == vy )
     	{
     		wheel_speed_[FRONT_LEFT] = 0.0;
     		wheel_speed_[FRONT_RIGHT] = vx;
     		wheel_speed_[BACK_LEFT] = vx;
     		wheel_speed_[BACK_RIGHT] = 0.0;
  	  	}
     	else if ( vx == -vy )
     	{
     		wheel_speed_[FRONT_RIGHT] = 0.0;
     		wheel_speed_[BACK_LEFT] = 0.0;
     		wheel_speed_[FRONT_LEFT] = vx;
     		wheel_speed_[BACK_RIGHT] = vx;	
     	}
    }
    else
    {
     	wheel_speed_[FRONT_LEFT] = -va;
     	wheel_speed_[FRONT_RIGHT] = va;
     	wheel_speed_[BACK_LEFT] = -va;
     	wheel_speed_[BACK_RIGHT] = va;   
   }
 }
 
 void GazeboRosfourwheelvisualDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
 {
     boost::mutex::scoped_lock scoped_lock ( lock );
     x_ = cmd_msg->linear.x;
     y_ = cmd_msg->linear.y;
     rot_ = cmd_msg->angular.z;
 }
 
 void GazeboRosfourwheelvisualDrive::QueueThread()
 {
     static const double timeout = 0.01;
 
     while ( alive_ && gazebo_ros_->node()->ok() ) {
         queue_.callAvailable ( ros::WallDuration ( timeout ) );
     }
 }

 
 GZ_REGISTER_MODEL_PLUGIN ( GazeboRosfourwheelvisualDrive )
 }

