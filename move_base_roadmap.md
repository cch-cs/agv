# base_local_planner ROADMAP

## move_base.cpp

91- vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

**What does the createInstance method do?**
130-     //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);

899- if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }

## trajectory_planner_ros.cpp

**Where is Trajectory Planner defined?**
142- TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,

234- tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

376- bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

399- geometry_msgs::PoseStamped drive_cmds;

402- geometry_msgs::PoseStamped robot_vel;
403- odom_helper_.getRobotVel(robot_vel);

447- Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

493- cmd_vel.linear.x = drive_cmds.pose.position.x;
494- cmd_vel.linear.y = drive_cmds.pose.position.y;
495- cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
    .
    .
    .
    }


## trajectory_planner.cpp

214- void TrajectoryPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      Trajectory& traj) {

229- double vx_i, vy_i, vtheta_i;

231- vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

256- traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;

345- //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

**Why is this done?**
348-  //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

353- //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);
      .
      .
      .
      }

536- Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta) {

557- min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

568- double vx_samp = min_vel_x;

573- Trajectory* best_traj = &traj_one;

618- if (holonomic_robot_) {
        //explore trajectories that move forward but also strafe slightly
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
    .
    .
    .
    }

902- return *best_traj;
      }

907- Trajectory TrajectoryPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose,
      geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities) {

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));

935- //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_);

966- if(best.cost_ < 0){
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    }
    else{
      drive_velocities.pose.position.x = best.xv_;
      drive_velocities.pose.position.y = best.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, best.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return best;
  }

## trajectory_planner.h
366- inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if((vg - vi) >= 0) {
          return std::min(vg, vi + a_max * dt);
        }
        return std::max(vg, vi - a_max * dt);
      }

# dwa_local_planner ROADMAP

## dwa_planner_ros.cpp
105- l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

108- costmap_ros_->getRobotPose(current_pose_);

115-  //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

120-  odom_helper_.setOdomTopic( odom_topic_ );

# move_base.cpp

401- bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
}

474- geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {

480- const geometry_msgs::PoseStamped& start = global_pose;

529- geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){

538- try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
}

639- void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {

649- planner_goal_ = goal;

654- current_goal_pub_.publish(goal);

  }

784- double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

789- bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

858- switch(state_){

870- case CONTROLLING:

874- if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

899- if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);



# move_base.h

180- MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;