#include<uav_local_planner/local_planner.h>

UAVLocalPlanner::UAVLocalPlanner()
{
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ROS_INFO("[local_planner] Getting params");
    ph.param("size_x", sizex_, 4.0);
    ph.param("size_y", sizey_, 4.0);
    ph.param("size_z", sizez_, 4.0);
    ph.param("resolution", resolution_, 0.1);

    ph.param("controller_frequency", controller_frequency_, 50.0);
    ph.param("collision_map_tolerance", collision_map_tolerance_, 0.5);
    ph.param("pose_tolerance", pose_tolerance_, 0.1);

    ph.param("landing_height", landing_height_, 0.4);
    ph.param("nominal_height", nominal_height_, 0.6);
    ph.param("nominal_linear_velocity", nominal_linear_velocity_, 0.3);
    ph.param("nominal_angular_velocity", nominal_angular_velocity_, M_PI / 2);

    flt_mode_req_topic_ = "flight_mode_request";
    flt_mode_stat_topic_ = "flight_mode_status";
    ctrl_cmd_topic_ = "controller_cmd";
    goal_pub_topic_ = "goal";
    goal_sub_topic_ = "goal";
    next_waypoint_topic_ = "controller/next_waypoint";
    local_collision_topic_ = "local_collision_map";
    uav_state_topic_ = "uav_state";
    path_topic_ = "path";

    ROS_INFO("[local_planner] done getting params");
    path_idx_ = 0;

    // publish UAV commands and goals (in case we detect a collision up ahead we
    // publish the same goal state to engage the planner)
    waypoint_vis_pub_ = nh.advertise<visualization_msgs::Marker>(next_waypoint_topic_, 1);
    command_pub_ = nh.advertise<uav_msgs::ControllerCommand>(ctrl_cmd_topic_, 1);
    RPYT_pub_ = nh.advertise<uav_msgs::ControllerCommand>("RPYT_cmd", 1);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_pub_topic_, 1);

    status_pub_ = nh.advertise<uav_msgs::FlightModeStatus>(flt_mode_stat_topic_, 1);

    // set up a occupancy grid triple buffer
    new_grid_ = false;

    // set up a path triple buffer
    controller_path_ = new nav_msgs::Path();
    latest_path_ = new nav_msgs::Path();
    callback_path_ = new nav_msgs::Path();
    new_path_ = false;

    // spawn the controller thread
    controller_thread_ = new boost::thread(boost::bind(&UAVLocalPlanner::controllerThread, this));

    // subscribe to the collision map, tf, path, goal, and flight mode
    path_sub_ = nh.subscribe(path_topic_, 1, &UAVLocalPlanner::pathCallback, this);
    goal_sub_ = nh.subscribe(goal_sub_topic_, 1, &UAVLocalPlanner::goalCallback, this);
    state_sub_ = nh.subscribe(uav_state_topic_, 1, &UAVLocalPlanner::stateCallback, this);
    flight_mode_sub_ = nh.subscribe(flt_mode_req_topic_, 1, &UAVLocalPlanner::flightModeCallback, this);

    dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig>::CallbackType f;
    f = boost::bind(&UAVController::dynamic_reconfigure_callback, &controller, _1, _2);
    dynamic_reconfigure_server_.setCallback(f);
}

UAVLocalPlanner::~UAVLocalPlanner()
{
    delete controller_path_;
    delete latest_path_;
    delete callback_path_;

    controller_thread_->join();
}

/***************** MAIN LOOP *****************/

// only the controller thread can update the UAV's state. callbacks must set
// flags to request change in state.
void UAVLocalPlanner::controllerThread()
{
    ROS_INFO("[controller] Starting controller thread...");
    ros::NodeHandle n;
    ros::Rate r(controller_frequency_);
//    double last_collision_map_update = ros::Time::now().toSec();
    uav_msgs::FlightModeStatus state;
    state.mode = uav_msgs::FlightModeStatus::LANDED;
    last_state_.mode = uav_msgs::FlightModeStatus::LANDED;
    while (n.ok()) {
        // if the robot is hovering and we get a new path, switch to following.
        // conversely, if we get a new goal but don't have a fresh path yet, go
        // back to hover
        ros::Time start_ = ros::Time::now();

        bool isNewPath = updatePath(state);
        // try to update the collision map. if the map we have is too old and we
        // are following a path, switch to hover.

        // check if we should be landing or taking off...
        getFlightMode(state);

        // get the robot's pose. if it is out of date....complain?
        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped velocity;
        getRobotPose(pose, velocity);
        if (ros::Time::now().toSec() - pose.header.stamp.toSec() > pose_tolerance_) {
//            ROS_ERROR("[controller] UAV pose is old...hit the deck! %f -%f= %1.4f", ros::Time::now().toSec(),pose.header.stamp.toSec(), ros::Time::now().toSec()-pose.header.stamp.toSec() ); //TODO: why is this always late?
        }

        if (state.mode == uav_msgs::FlightModeStatus::HOVER && last_state_.mode != uav_msgs::FlightModeStatus::HOVER) {
            ROS_INFO("[controller] transitioning to hover (setting the hover pose)");
        }

        last_state_ = state;
        uav_msgs::ControllerCommand u;
        controller.setTrackedVelocity(0.0, 0.0);
        switch (state.mode) {
        case uav_msgs::FlightModeStatus::LANDED:
//            ROS_INFO("[controller] state: LANDED");
            hover_pose_ = pose;
            u.thrust = 0;
            u.roll = 0;
            u.yaw = 0;
            u.pitch = 0;
            break;
        case uav_msgs::FlightModeStatus::LANDING:
//            ROS_INFO("[controller] state: LANDING");
            u = land(pose, velocity, state);
            break;
        case uav_msgs::FlightModeStatus::TAKE_OFF:
            ROS_INFO("[controller] state: TAKE_OFF");
            u = takeOff(pose, velocity, state);
//            hover_pose_.pose.position.x = pose.pose.position.x;
//            hover_pose_.pose.position.y = pose.pose.position.y;
            hover_pose_.pose.position.z = 1.65;
//            hover_pose_.pose.orientation.x = 0;
//            hover_pose_.pose.orientation.y = 0;
//            hover_pose_.pose.orientation.z = 0;
//            hover_pose_.pose.orientation.w = 1;
            break;
        case uav_msgs::FlightModeStatus::HOVER:
            ROS_INFO("[controller] state: HOVER");
            u = hover(pose, velocity);
            break;
        case uav_msgs::FlightModeStatus::FOLLOWING:
            ROS_INFO("[controller] state: FOLLOWING");
            hover_pose_ = pose;
            u = followPath(pose, velocity, state, isNewPath);
            break;
        default:
            ROS_INFO("[controller] In an invalid state! Landing...");
            state.mode = uav_msgs::FlightModeStatus::LANDING;
            landing_z_ = pose.pose.position.z;
            break;
        }
        last_u_ = u;
        ROS_DEBUG("###### pose   X:%f Y:%f Z:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        u.header.stamp = ros::Time::now();
        command_pub_.publish(u);
        RPYT_pub_.publish(u);
        status_pub_.publish(state);
        ros::Time stop_ = ros::Time::now();
        ROS_DEBUG("[local_planner] main loop %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());

        r.sleep();
    }
}

/***************** STATE FUNCTIONS *****************/

uav_msgs::ControllerCommand UAVLocalPlanner::land(
    geometry_msgs::PoseStamped pose,
    geometry_msgs::TwistStamped vel,
    uav_msgs::FlightModeStatus & state)
{
    uav_msgs::ControllerCommand u;

    if (pose.pose.position.z <= landing_height_) {
        u = last_u_;
        u.roll = 0;
        u.pitch = 0;
        u.yaw = 0;
        if (u.thrust <= 3.0) {
            state.mode = uav_msgs::FlightModeStatus::LANDED;
        }
        else {
            u.thrust -= 0.2;
        }
    }
    else {
        geometry_msgs::PoseStamped target = hover_pose_;
        if (pose.pose.position.z <= landing_z_ + 0.2) {
            landing_z_ -= 0.003;
        }
        target.pose.position.z = landing_z_;
        visualizeTargetPose(target);
        u = controller.Controller(pose, vel, target);
    }
    return u;
}

uav_msgs::ControllerCommand UAVLocalPlanner::takeOff(
    geometry_msgs::PoseStamped pose,
    geometry_msgs::TwistStamped vel,
    uav_msgs::FlightModeStatus& state)
{
    geometry_msgs::PoseStamped target = hover_pose_;
    if (pose.pose.position.z >= nominal_height_) {
        state.mode = uav_msgs::FlightModeStatus::HOVER;
    }
    else {
        target.pose.position.z = pose.pose.position.z + 0.2;
    }
    visualizeTargetPose(target);
    return controller.Controller(pose, vel, target);
}

uav_msgs::ControllerCommand UAVLocalPlanner::hover(
    geometry_msgs::PoseStamped pose,
    geometry_msgs::TwistStamped vel)
{
    visualizeTargetPose(hover_pose_);
    return controller.Controller(pose, vel, hover_pose_);
}

uav_msgs::ControllerCommand UAVLocalPlanner::followPath(
    geometry_msgs::PoseStamped pose,
    geometry_msgs::TwistStamped vel,
    uav_msgs::FlightModeStatus& state,
    bool isNewPath)
{
    if (isNewPath) {
        path_idx_ = 0;
    }
    ROS_WARN("size is %zu ", controller_path_->poses.size());
    double dx = pose.pose.position.x - controller_path_->poses[path_idx_].pose.position.x;
    double dy = pose.pose.position.y - controller_path_->poses[path_idx_].pose.position.y;
    double dz = pose.pose.position.z - controller_path_->poses[path_idx_].pose.position.z;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    const double carrot_dist = 0.3;
    if(dist < carrot_dist)
    {
        ++path_idx_;
        dx = pose.pose.position.x - controller_path_->poses[path_idx_].pose.position.x;
        dy = pose.pose.position.y - controller_path_->poses[path_idx_].pose.position.y;
        dz = pose.pose.position.z - controller_path_->poses[path_idx_].pose.position.z;
        dist = sqrt(dx * dx + dy * dy + dz * dz);
    }
    unsigned int i;
    ROS_DEBUG("a");
    for (i = path_idx_ + 1; i < controller_path_->poses.size(); i++) {
        dx = pose.pose.position.x - controller_path_->poses[i].pose.position.x;
        dy = pose.pose.position.y - controller_path_->poses[i].pose.position.y;
        dz = pose.pose.position.z - controller_path_->poses[i].pose.position.z;
        double temp = sqrt(dx * dx + dy * dy + dz * dz);
        if (temp > dist && temp > carrot_dist) {
            break;
        }
        dist = temp;
    }
    ROS_DEBUG("b");
    path_idx_ = i - 1;
    if (3 >= ((int)(controller_path_->poses.size()) - ((int)i))) {
        state.mode = uav_msgs::FlightModeStatus::HOVER;
        hover_pose_ = controller_path_->poses[controller_path_->poses.size() - 1];
        i = controller_path_->poses.size() - 1;
    }

    ///////////////////////////////////////////////////////////////////
    // PATH_IDX_ IS DONE CHANGING BY HERE...AND NOW MARKS THE CARROT //
    ///////////////////////////////////////////////////////////////////

    const double nominal_vel = 0.3; // TODO: configurate
    if (path_idx_ < (int)controller_path_->poses.size() - 1) {
      const geometry_msgs::Point& curr_tracked = controller_path_->poses[path_idx_].pose.position;
      const geometry_msgs::Point& next_tracked = controller_path_->poses[path_idx_ + 1].pose.position;
      double dx = next_tracked.x - curr_tracked.x;
      double dy = next_tracked.y - curr_tracked.y;
      tf::Vector3 d(dx, dy, 0.0);
      if (d.length2() > 0.0) {
        d = nominal_vel * d.normalized();
      }
      controller.setTrackedVelocity(d.x(), d.y());
    }
    else {
      controller.setTrackedVelocity(0.0, 0.0);
    }

    ROS_DEBUG("c");
    ROS_DEBUG("point index is %d", i);

    if (controller_path_->poses[i].pose.position.z < 0.6) {
        ROS_DEBUG("Target height was %f, Setting it to 0.6", controller_path_->poses[i].pose.position.z);
        controller_path_->poses[i].pose.position.z = 0.6;
    }

    // TODO: verify that i is never out of bounds

    // TODO: collision check the path from our pose to the target pose (just check the straight line)
    // TODO: collision check the path from the target to the next few points (use a time horizon)
    const geometry_msgs::PoseStamped& target = controller_path_->poses[i];
    ROS_DEBUG("next target is %f %f %f", target.pose.position.x, target.pose.position.y, target.pose.position.z);
    visualizeTargetPose(target);

//    const double taper_dist = 0.4; // TODO: configurate
//    double vel_goal_ratio = nominal_vel / taper_dist;
//    double dx_goal = pose.pose.position.x - controller_path_->poses.back().pose.position.x;
//    double dy_goal = pose.pose.position.y - controller_path_->poses.back().pose.position.y;
//    double d_goal = sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
//    if(d_goal > taper_dist) {
//        controller.setTrackedVelocity(nominal_vel);
//    }
//    else {
//        controller.setTrackedVelocity(vel_goal_ratio*d_goal);
//    }

    uav_msgs::ControllerCommand u = controller.Controller(pose, vel, target);
    // TODO: collision check the controls for some very short period of time
    return u;
}

/***************** UPDATE FUNCTIONS *****************/

bool UAVLocalPlanner::updateCollisionMap()
{
    // if there is a new occupancy grid in latest then swap into the controller
    // occupancy grid
    bool updated = false;
    return updated;
}

bool UAVLocalPlanner::updatePath(uav_msgs::FlightModeStatus &state)
{
    // if there is a new path in latest then swap into the controller path
    bool ret = false;
    boost::unique_lock<boost::mutex> lock(path_mutex_);
    if (new_path_) {
        ROS_ERROR("[controller] received new path");
        nav_msgs::Path* temp = latest_path_;
        latest_path_ = controller_path_;
        controller_path_ = temp;
        new_path_ = false;
        ret = true;

        boost::unique_lock<boost::mutex> goal_lock(goal_mutex_);
        if (controller_path_->poses.empty() ||
            latest_goal_.header.stamp.toSec() > controller_path_->header.stamp.toSec())
        {
            // switch from following to hover if the new path is old (or empty)
            if (state.mode == uav_msgs::FlightModeStatus::FOLLOWING) {
                state.mode = uav_msgs::FlightModeStatus::HOVER;
                hover_pose_ = latest_goal_;
            }
        }
        else {
            // switch from hover to following if the new path is up to date
            if (state.mode == uav_msgs::FlightModeStatus::HOVER) {
                state.mode = uav_msgs::FlightModeStatus::FOLLOWING;
            }
        }
        goal_lock.unlock();
    }
    lock.unlock();

    return ret;
}

void UAVLocalPlanner::getFlightMode(uav_msgs::FlightModeStatus &state)
{
    boost::unique_lock<boost::mutex> lock(flight_mode_mutex_);
    uav_msgs::FlightModeRequest flightMode = flight_mode_;
    flight_mode_.mode = uav_msgs::FlightModeRequest::NONE;
    lock.unlock();

    if (flightMode.mode == uav_msgs::FlightModeRequest::LAND) {
        if (state.mode == uav_msgs::FlightModeStatus::TAKE_OFF ||
            state.mode == uav_msgs::FlightModeStatus::HOVER ||
            state.mode == uav_msgs::FlightModeStatus::FOLLOWING)
        {
            state.mode = uav_msgs::FlightModeStatus::LANDING;
            landing_z_ = latest_state_.pose.pose.position.z;
        }
        else
            ROS_WARN("[controller] Asked to land, but UAV is landed or already landing.");
    }
    else if (flightMode.mode == uav_msgs::FlightModeRequest::TAKE_OFF) {
        if (state.mode == uav_msgs::FlightModeStatus::LANDED ||
            state.mode == uav_msgs::FlightModeStatus::LANDING)
        {
            state.mode = uav_msgs::FlightModeStatus::TAKE_OFF;
        }
        else {
            ROS_WARN("[controller] Asked to take off, but UAV is already in the air (or working on it).");
        }
    }
    else if (flightMode.mode == uav_msgs::FlightModeRequest::HOVER) {
        if (state.mode == uav_msgs::FlightModeStatus::FOLLOWING) {
            state.mode = uav_msgs::FlightModeStatus::HOVER;
        }
        else {
            ROS_WARN("[controller] Asked to hover, but the UAV can only go to hover from path following.");
        }
    }
}

void UAVLocalPlanner::getRobotPose(
    geometry_msgs::PoseStamped& pose,
    geometry_msgs::TwistStamped& velocity)
{
    boost::unique_lock<boost::mutex> lock(state_mutex_);
    pose.header = latest_state_.header;
    pose.pose = latest_state_.pose.pose;
    velocity.header = latest_state_.header;
    velocity.twist = latest_state_.twist.twist;
    lock.unlock();
}

/***************** CALLBACKS *****************/

void UAVLocalPlanner::pathCallback(nav_msgs::PathConstPtr path)
{
    ros::Time start_ = ros::Time::now();

    *callback_path_ = *path;
    boost::unique_lock<boost::mutex> lock(path_mutex_);
    nav_msgs::Path* temp = latest_path_;
    latest_path_ = callback_path_;
    callback_path_ = temp;
    new_path_ = true;
    lock.unlock();
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[local_planner] path callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

void UAVLocalPlanner::goalCallback(geometry_msgs::PoseStampedConstPtr goal)
{
    ros::Time start_ = ros::Time::now();

    boost::unique_lock<boost::mutex> lock(goal_mutex_);
    latest_goal_ = *goal;
    lock.unlock();
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[local_planner] goal callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

void UAVLocalPlanner::stateCallback(nav_msgs::OdometryConstPtr state)
{
    ros::Time start_ = ros::Time::now();

    boost::unique_lock<boost::mutex> lock(state_mutex_);
    latest_state_ = *state;
    lock.unlock();
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[local_planner] state callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

void UAVLocalPlanner::flightModeCallback(uav_msgs::FlightModeRequestConstPtr req)
{
    ros::Time start_ = ros::Time::now();
    boost::unique_lock<boost::mutex> lock(flight_mode_mutex_);
    flight_mode_ = *req;
    lock.unlock();
    ros::Time stop_ = ros::Time::now();
    ROS_DEBUG("[local_planner] flight mode request callback %f %f = %f", start_.toSec(), stop_.toSec(), stop_.toSec() - start_.toSec());
}

/***************** VISUALIZATION *****************/

void UAVLocalPlanner::visualizeTargetPose(geometry_msgs::PoseStamped p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time(0);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = p.pose;
    marker.scale.z = 0.3;
    marker.scale.y = 0.20;
    marker.scale.x = 0.20;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.ns = "controller target";
    marker.id = 0;
    marker.lifetime = ros::Duration(0);
    waypoint_vis_pub_.publish(marker);
}

/***************** MAIN *****************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_local_planner");
    ROS_INFO("[local planner] starting local planner");
    UAVLocalPlanner local_planner;
    ROS_INFO("[local planner] going to spin");
    ros::spin();
    return 0;
}
