#include <uav_local_planner/twist_local_planner.h>

#include <ostream>

#include <Eigen/Dense>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <uav_msgs/ControllerCommand.h>
#include <visualization_msgs/Marker.h>

namespace uav_msgs {

std::ostream& operator<<(std::ostream& o, uav_msgs::FlightModeStatus status)
{
    switch (status.mode) {
    case uav_msgs::FlightModeStatus::LANDED:
        o << "LANDED";
        break;
    case uav_msgs::FlightModeStatus::LANDING:
        o << "LANDING";
        break;
    case uav_msgs::FlightModeStatus::TAKE_OFF:
        o << "TAKE_OFF";
        break;
    case uav_msgs::FlightModeStatus::HOVER:
        o << "HOVER";
        break;
    case uav_msgs::FlightModeStatus::FOLLOWING:
        o << "FOLLOWING";
        break;
    }
    return o;
}

std::string to_string(uav_msgs::FlightModeStatus status)
{
    std::stringstream ss;
    ss << status;
    return ss.str();
}

} // namespace uav_msgs

namespace uav_local_planner {

TwistLocalPlanner::TwistLocalPlanner() :
    m_nh(),
    m_ph("~"),
    m_waypoint_vis_pub(),
    m_command_pub(),
    m_status_pub(),
    m_goal_pub(),
    m_path_sub(),
    m_goal_sub(),
    m_state_sub(),
    m_flight_mode_sub(),
    m_controller_path(),
    m_latest_path(),
    m_callback_path(),
    m_flight_mode(),
    m_latest_goal(),
    m_latest_state(),
    m_controller_frequency(0.0),
    m_landing_height(0.0),
    m_nominal_height(0.0),
    m_nominal_angular_velocity(0.0),
    m_new_path(false),
    m_path_idx(0)
{
    m_ph.param("controller_frequency", m_controller_frequency, 50.0);
    m_ph.param("landing_height", m_landing_height, 0.4);
    m_ph.param("nominal_height", m_nominal_height, 0.6);
    m_ph.param("nominal_linear_velocity", m_nominal_linear_velocity, 0.3);
    m_ph.param("nominal_angular_velocity", m_nominal_angular_velocity, M_PI / 2.0);

    m_waypoint_vis_pub = m_nh.advertise<visualization_msgs::Marker>("controller/next_waypoint", 1);
    m_command_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
    m_status_pub = m_nh.advertise<uav_msgs::FlightModeStatus>("flight_mode_status", 1);

    m_path_sub = m_nh.subscribe("path", 1, &TwistLocalPlanner::pathCallback, this);
    m_goal_sub = m_nh.subscribe("goal", 1, &TwistLocalPlanner::goalCallback, this);
    m_state_sub = m_nh.subscribe("uav_state", 1, &TwistLocalPlanner::stateCallback, this);
    m_flight_mode_sub = m_nh.subscribe("flight_mode_request", 1, &TwistLocalPlanner::flightModeCallback, this);

    m_controller_path = new nav_msgs::Path;
    m_latest_path = new nav_msgs::Path;
    m_callback_path = new nav_msgs::Path;
    m_new_path = false;

    m_prev_state.mode = uav_msgs::FlightModeStatus::LANDED;
    m_curr_state.mode = uav_msgs::FlightModeStatus::LANDED;
    m_next_state.mode = uav_msgs::FlightModeStatus::LANDED;

    State state;

    state.enter = &TwistLocalPlanner::onEnter_Landed;
    state.on = &TwistLocalPlanner::on_Landed;
    state.exit = &TwistLocalPlanner::onExit_Landed;
    m_states.insert(std::make_pair(uav_msgs::FlightModeStatus::LANDED, state));

    state.enter = &TwistLocalPlanner::onEnter_Landing;
    state.on = &TwistLocalPlanner::on_Landing;
    state.exit = &TwistLocalPlanner::onExit_Landing;
    m_states.insert(std::make_pair(uav_msgs::FlightModeStatus::LANDING, state));

    state.enter = &TwistLocalPlanner::onEnter_TakeOff;
    state.on = &TwistLocalPlanner::on_TakeOff;
    state.exit = &TwistLocalPlanner::onExit_TakeOff;
    m_states.insert(std::make_pair(uav_msgs::FlightModeStatus::TAKE_OFF, state));

    state.enter = &TwistLocalPlanner::onEnter_Hover;
    state.on = &TwistLocalPlanner::on_Hover;
    state.exit = &TwistLocalPlanner::onExit_Hover;
    m_states.insert(std::make_pair(uav_msgs::FlightModeStatus::HOVER, state));

    state.enter = &TwistLocalPlanner::onEnter_Following;
    state.on = &TwistLocalPlanner::on_Following;
    state.exit = &TwistLocalPlanner::onExit_Following;
    m_states.insert(std::make_pair(uav_msgs::FlightModeStatus::FOLLOWING, state));
}

bool TwistLocalPlanner::init()
{
    return true;
}

int TwistLocalPlanner::run()
{
    uint32_t cmd_seqno = 0;
    ros::Rate loop_rate(m_controller_frequency);

    geometry_msgs::Twist curr_cmd;

    TrackingContext context;
    while (ros::ok()) {
        ros::spinOnce(); // pump callbacks

        ros::Time start = ros::Time::now();

        // update current tracking context

        bool old_new_path = context.new_path;
        uav_msgs::FlightModeRequest old_flight_mode_request;
        old_flight_mode_request = context.flight_mode_request;

        context.new_path = updatePath();
        context.flight_mode_request = getFlightMode();

        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped vel;
        if (!getRobotPose(pose, vel)) {
            // failed to get tracking context, maintain path and request vars...
            // TODO: security measures here...additional checking to make sure
            // pose is sane and recovery/safety behaviors
            context.new_path = old_new_path;
            context.flight_mode_request = old_flight_mode_request;
            loop_rate.sleep();
            continue;
        }

        // update tracking context
        context.pose = pose;
        context.vel = vel;

        // enter the current state
        if (m_curr_state.mode != m_prev_state.mode) {
            ROS_INFO("State Changed (%s -> %s)", to_string(m_prev_state).c_str(), to_string(m_curr_state).c_str());
            (this->*(m_states[m_curr_state.mode].enter))(context, m_prev_state.mode);
        }

        // spin the current state
        m_next_state.mode = (this->*(m_states[m_curr_state.mode].on))(context, curr_cmd);

        // exit the current state
        if (m_next_state.mode != m_curr_state.mode) {
            (this->*(m_states[m_curr_state.mode].exit))(context, m_next_state.mode);
        }

        m_prev_state = m_curr_state;
        m_curr_state = m_next_state;

//        curr_cmd.header.stamp = ros::Time::now();
//        curr_cmd.header.frame_id = "";
//        curr_cmd.header.seq = cmd_seqno++;
        m_command_pub.publish(curr_cmd);
        m_prev_cmd = curr_cmd;

        m_status_pub.publish(m_curr_state);

        loop_rate.sleep();
    }
    return 0;
}

geometry_msgs::Twist TwistLocalPlanner::control(
    const geometry_msgs::Pose& pose,
    const geometry_msgs::Twist& vel,
    const geometry_msgs::Pose& target)
{
    Eigen::Vector2d planar_vel;
    planar_vel.x() = target.position.x - pose.position.x;
    planar_vel.y() = target.position.y - pose.position.y;

    if (planar_vel.squaredNorm() >
            m_nominal_linear_velocity * m_nominal_linear_velocity)
    {
        // normalize to m_nominal_linear_velocity
        planar_vel.normalize();
        planar_vel *= m_nominal_linear_velocity;
    }

    double z_vel = target.position.z - pose.position.z;

    if (fabs(z_vel) > m_nominal_linear_velocity) {
        // normalize z velocity
        z_vel = (z_vel < 0) ? -m_nominal_linear_velocity : m_nominal_linear_velocity;
    }

    Eigen::Vector3d lvel(planar_vel.x(), planar_vel.y(), z_vel);

    double curr_roll, curr_pitch, curr_yaw;
    tf::Quaternion curr_quat;
    tf::quaternionMsgToTF(pose.orientation, curr_quat);
    tf::Matrix3x3(curr_quat).getRPY(curr_roll, curr_pitch, curr_yaw);

    double dest_roll, dest_pitch, dest_yaw;
    tf::Quaternion dest_quat;
    tf::quaternionMsgToTF(target.orientation, dest_quat);
    tf::Matrix3x3(dest_quat).getRPY(dest_roll, dest_pitch, dest_yaw);

    double dyaw = angles::shortest_angular_distance(curr_yaw, dest_yaw);

    if (fabs(dyaw) > m_nominal_angular_velocity) {
        dyaw = (dyaw < 0) ? -m_nominal_angular_velocity : m_nominal_angular_velocity;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = lvel.x();
    cmd.linear.y = lvel.y();
    cmd.linear.z = lvel.z();
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = dyaw;
    return cmd;
}

void TwistLocalPlanner::pathCallback(
    const nav_msgs::Path::ConstPtr& msg)
{
    *m_callback_path = *msg;
    std::swap(m_callback_path, m_latest_path);
    m_new_path = true;
}

void TwistLocalPlanner::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    m_latest_goal = *msg;
}

void TwistLocalPlanner::stateCallback(
    const nav_msgs::Odometry::ConstPtr& msg)
{
    m_latest_state = msg;
}

void TwistLocalPlanner::flightModeCallback(
    const uav_msgs::FlightModeRequest::ConstPtr& msg)
{
    m_flight_mode = *msg;
}

bool TwistLocalPlanner::updatePath()
{
    // if there is a new path in latest then swap into the controller path
    if (m_new_path) {
        ROS_ERROR("Received new path");
        std::swap(m_latest_path, m_controller_path);
        m_new_path = false;
        return true;
    }

    return false;
}

uav_msgs::FlightModeRequest TwistLocalPlanner::getFlightMode()
{
    uav_msgs::FlightModeRequest flightMode = m_flight_mode;
    m_flight_mode.mode = uav_msgs::FlightModeRequest::NONE;
    return flightMode;
}

bool TwistLocalPlanner::getRobotPose(
    geometry_msgs::PoseStamped& pose,
    geometry_msgs::TwistStamped& vel)
{
    if (m_latest_state) {
        pose.header = m_latest_state->header;
        pose.pose = m_latest_state->pose.pose;
        vel.header = m_latest_state->header;
        vel.twist = m_latest_state->twist.twist;
        return true;
    }
    else {
        return false;
    }
}

void TwistLocalPlanner::onEnter_Landed(
    const TrackingContext& context,
    int8_t prev_status)
{

}

int8_t TwistLocalPlanner::on_Landed(
    const TrackingContext& context,
    geometry_msgs::Twist& cmd)
{
    switch (context.flight_mode_request.mode) {
    case uav_msgs::FlightModeRequest::LAND:
        ROS_WARN("Asked to land, but UAV is landed");
        break;
    case uav_msgs::FlightModeRequest::TAKE_OFF:
        return uav_msgs::FlightModeStatus::TAKE_OFF;
    case uav_msgs::FlightModeRequest::HOVER:
        ROS_WARN("Asked to hover, but UAV is landed");
        break;
    case uav_msgs::FlightModeRequest::FOLLOW:
        ROS_WARN("Asked to follow path, but UAV is landed");
        break;
    default:
        break;
    }

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    return uav_msgs::FlightModeStatus::LANDED;
}

void TwistLocalPlanner::onExit_Landed(
    const TrackingContext& context,
    int8_t next_status)
{

}

void TwistLocalPlanner::onEnter_Landing(
    const TrackingContext& context,
    int8_t prev_status)
{

}

int8_t TwistLocalPlanner::on_Landing(
    const TrackingContext& context,
    geometry_msgs::Twist& cmd)
{
    switch (context.flight_mode_request.mode) {
    case uav_msgs::FlightModeRequest::LAND:
        ROS_WARN("Asked to land, but UAV is landing");
        break;
    case uav_msgs::FlightModeRequest::TAKE_OFF:
        return uav_msgs::FlightModeStatus::TAKE_OFF;
    case uav_msgs::FlightModeRequest::HOVER:
        ROS_WARN("Asked to hover, but UAV is landing");
        break;
    case uav_msgs::FlightModeRequest::FOLLOW:
        ROS_WARN("Asked to follow path, but UAV is landing");
        break;
    default:
        break;
    }

    if (context.pose.pose.position.z <= m_landing_height) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        return uav_msgs::FlightModeStatus::LANDED;
    }
    else {
        geometry_msgs::PoseStamped target = m_hover_pose;
        if (context.pose.pose.position.z <= m_landing_z + 0.2) {
            m_landing_z -= 0.003;
        }
        target.pose.position.z = m_landing_z;
        cmd = control(context.pose.pose, context.vel.twist, target.pose);
    }

    return uav_msgs::FlightModeStatus::LANDING;
}

void TwistLocalPlanner::onExit_Landing(
    const TrackingContext& context,
    int8_t next_status)
{

}

void TwistLocalPlanner::onEnter_TakeOff(
    const TrackingContext& context,
    int8_t prev_status)
{
    switch (prev_status) {
    case uav_msgs::FlightModeStatus::LANDED:
        m_hover_pose = context.pose;
        m_hover_pose.pose.position.z = m_nominal_height;
        break;
    default:
        break;
    }
}

int8_t TwistLocalPlanner::on_TakeOff(
    const TrackingContext& context,
    geometry_msgs::Twist& cmd)
{
    switch (context.flight_mode_request.mode) {
    case uav_msgs::FlightModeRequest::LAND:
        m_landing_z = context.pose.pose.position.z;
        return uav_msgs::FlightModeStatus::LANDING;
    case uav_msgs::FlightModeRequest::TAKE_OFF:
        ROS_WARN("Asked to take off, but UAV is already taking off");
        break;
    case uav_msgs::FlightModeRequest::HOVER:
        ROS_WARN("Asked to hover, but UAV is taking off");
        break;
    case uav_msgs::FlightModeRequest::FOLLOW:
        ROS_WARN("Asked to follow path, but UAV is taking off");
        break;
    default:
        break;
    }

    if (context.pose.pose.position.z >= m_nominal_height) {
        return uav_msgs::FlightModeStatus::HOVER;
    }

    geometry_msgs::PoseStamped target_pose = m_hover_pose;
    target_pose.pose.position.z = context.pose.pose.position.z + 0.2;
    cmd = control(context.pose.pose, context.vel.twist, target_pose.pose);
    return uav_msgs::FlightModeStatus::TAKE_OFF;
}

void TwistLocalPlanner::onExit_TakeOff(
    const TrackingContext& context,
    int8_t next_status)
{

}

void TwistLocalPlanner::onEnter_Hover(
    const TrackingContext& context,
    int8_t prev_status)
{
}

int8_t TwistLocalPlanner::on_Hover(
    const TrackingContext& context,
    geometry_msgs::Twist& cmd)
{
    switch (context.flight_mode_request.mode) {
    case uav_msgs::FlightModeRequest::LAND:
        m_landing_z = context.pose.pose.position.z;
        return uav_msgs::FlightModeStatus::LANDING;
    case uav_msgs::FlightModeRequest::TAKE_OFF:
        ROS_WARN("Asked to take off, but UAV is already hovering");
        break;
    case uav_msgs::FlightModeRequest::HOVER:
        ROS_WARN("Asked to hover, but UAV is already hovering");
        break;
    case uav_msgs::FlightModeRequest::FOLLOW:
        ROS_WARN("Asked to follow path, but follow path requests are handled by receiving new paths");
        break;
    default:
        break;
    }

    if (context.new_path) {
        return uav_msgs::FlightModeStatus::FOLLOWING;
    }

    cmd = control(context.pose.pose, context.vel.twist, m_hover_pose.pose);
    return uav_msgs::FlightModeStatus::HOVER;
}

void TwistLocalPlanner::onExit_Hover(
    const TrackingContext& context,
    int8_t next_status)
{

}

void TwistLocalPlanner::onEnter_Following(
    const TrackingContext& context,
    int8_t prev_status)
{

}

int8_t TwistLocalPlanner::on_Following(
    const TrackingContext& context,
    geometry_msgs::Twist& cmd)
{
    switch (context.flight_mode_request.mode) {
    case uav_msgs::FlightModeRequest::LAND:
        m_landing_z = context.pose.pose.position.z;
        return uav_msgs::FlightModeStatus::LANDING;
    case uav_msgs::FlightModeRequest::TAKE_OFF:
        ROS_WARN("Asked to take off, but UAV is following a path");
        break;
    case uav_msgs::FlightModeRequest::HOVER:
        return uav_msgs::FlightModeStatus::HOVER;
    case uav_msgs::FlightModeRequest::FOLLOW:
        break;
    default:
        break;
    }

    if (context.new_path) {
        // path is empty or latest goal is newer => hover
        if (m_controller_path->poses.empty() ||
            m_latest_goal.header.stamp.toSec() >
                    m_controller_path->header.stamp.toSec())
        {
            m_hover_pose = m_latest_goal;
            return uav_msgs::FlightModeStatus::HOVER;
        }
    }

    if (context.new_path) {
        m_path_idx = 0;
    }
    ROS_WARN("size is %zu ", m_controller_path->poses.size());

    double dx = context.pose.pose.position.x - m_controller_path->poses[m_path_idx].pose.position.x;
    double dy = context.pose.pose.position.y - m_controller_path->poses[m_path_idx].pose.position.y;
    double dz = context.pose.pose.position.z - m_controller_path->poses[m_path_idx].pose.position.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    unsigned int i;
    for (i = m_path_idx + 1; i < m_controller_path->poses.size(); i++) {
        dx = context.pose.pose.position.x - m_controller_path->poses[i].pose.position.x;
        dy = context.pose.pose.position.y - m_controller_path->poses[i].pose.position.y;
        dz = context.pose.pose.position.z - m_controller_path->poses[i].pose.position.z;
        double temp = sqrt(dx*dx + dy*dy + dz*dz);
        if (temp > dist && temp > 0.3) {
            break;
        }
        dist = temp;
    }

    m_path_idx = i - 1;
    bool hover = false;
    if (m_controller_path->poses.size() - i < 3) {
        hover = true;
        m_hover_pose = m_controller_path->poses.back();
        i = m_controller_path->poses.size() - 1;
    }

    ROS_DEBUG("point index is %d", i);

    const double m_minimum_height = 0.6;
    if (m_controller_path->poses[i].pose.position.z < m_minimum_height) {
        ROS_DEBUG("Target height was %f, Setting it to 0.6", m_controller_path->poses[i].pose.position.z);
        m_controller_path->poses[i].pose.position.z = m_minimum_height;
    }

    // TODO: verify that i is never out of bounds

    // TODO: collision check the path from our pose to the target pose (just check the straight line)
    // TODO: collision check the path from the target to the next few points (use a time horizon)

    geometry_msgs::PoseStamped target = m_controller_path->poses[i];
    ROS_DEBUG("next target is %f %f %f", target.pose.position.x, target.pose.position.y, target.pose.position.z);
    cmd = control(context.pose.pose, context.vel.twist, target.pose);

    // TODO: collision check the controls for some very short period of time

    if (hover) {
        return uav_msgs::FlightModeStatus::HOVER;
    }

    return uav_msgs::FlightModeStatus::FOLLOWING;
}

void TwistLocalPlanner::onExit_Following(
    const TrackingContext& context,
    int8_t next_status)
{

}

} // namespace uav_local_planner

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twist_local_planner");

    uav_local_planner::TwistLocalPlanner lp;
    if (!lp.init()) {
        return 1;
    }
    return lp.run();
}
