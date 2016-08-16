#ifndef UAV_LOCAL_PLANNER_H
#define UAV_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <uav_msgs/FlightModeRequest.h>
#include <uav_msgs/FlightModeStatus.h>
#include <uav_msgs/ControllerCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <uav_local_planner/controller.h>
#include <string>

// TODO: add local planner collision avoidance
#define QUADRAD 0.65
#define DISTANCE(x1, x2, y1, y2, z1, z2) sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2))
#define PI 3.14159265358979323846

class UAVLocalPlanner
{
public:

    UAVLocalPlanner();
    ~UAVLocalPlanner();

    void controllerThread();

    uav_msgs::ControllerCommand land(
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped,
        uav_msgs::FlightModeStatus&);
    uav_msgs::ControllerCommand takeOff(
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped,
        uav_msgs::FlightModeStatus&);
    uav_msgs::ControllerCommand hover(
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped);
    uav_msgs::ControllerCommand followPath(
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped,
        uav_msgs::FlightModeStatus&,
        bool isNewPath);

private:

    bool updateCollisionMap();
    bool updatePath(uav_msgs::FlightModeStatus&);
    void getFlightMode(uav_msgs::FlightModeStatus &state);
    void getRobotPose(
        geometry_msgs::PoseStamped& pose,
        geometry_msgs::TwistStamped& velocity);

    float inline quad(float a, float b, float c);
    bool willCollide(geometry_msgs::Pose pose, geometry_msgs::Pose target);

    void pathCallback(nav_msgs::PathConstPtr path);
    void goalCallback(geometry_msgs::PoseStampedConstPtr goal);
    void stateCallback(nav_msgs::OdometryConstPtr state);
    void flightModeCallback(uav_msgs::FlightModeRequestConstPtr req);
    void fixedLaserCallback(sensor_msgs::LaserScanConstPtr scan);
    void panningLaserCallback(sensor_msgs::LaserScanConstPtr scan);
    void visualizeTargetPose(geometry_msgs::PoseStamped p);
    void visualizeCollisionPath(geometry_msgs::Pose pose, geometry_msgs::Pose target);
    void visualizeCollisionVision(int start, int end, double angle_inc);

    UAVController controller;
    dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig> dynamic_reconfigure_server_;

    double sizex_;
    double sizey_;
    double sizez_;
    double resolution_;

    bool ignore_lasers_[720];

    std::string flt_mode_req_topic_;
    std::string flt_mode_stat_topic_;
    std::string ctrl_cmd_topic_;
    std::string goal_pub_topic_;
    std::string goal_sub_topic_;
    std::string next_waypoint_topic_;
    std::string local_collision_topic_;
    std::string uav_state_topic_;
    std::string path_topic_;
    std::string f_laser_topic_;
    std::string p_laser_topic_;

    ros::Publisher waypoint_vis_pub_;
    ros::Publisher command_pub_;
    ros::Publisher RPYT_pub_;
    ros::Publisher status_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher rviz_pub_;
    ros::Subscriber collision_map_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber flight_mode_sub_;
    ros::Subscriber f_laser_sub_;
    ros::Subscriber p_laser_sub_;

    nav_msgs::Path* controller_path_;
    nav_msgs::Path* latest_path_;
    nav_msgs::Path* callback_path_;
    uav_msgs::FlightModeRequest flight_mode_;
    geometry_msgs::PoseStamped latest_goal_;
    nav_msgs::Odometry latest_state_;
    sensor_msgs::LaserScan latest_scan_;

    boost::thread* controller_thread_;
    boost::mutex grid_mutex_;
    boost::mutex path_mutex_;
    boost::mutex goal_mutex_;
    boost::mutex state_mutex_;
    boost::mutex flight_mode_mutex_;
    bool new_grid_, new_path_;

    double controller_frequency_;
    double collision_map_tolerance_, pose_tolerance_;
    double landing_height_;
    double nominal_height_;
    double nominal_linear_velocity_;
    double nominal_angular_velocity_;
    double landing_z_;

    uav_msgs::ControllerCommand last_u_;
    geometry_msgs::PoseStamped hover_pose_;
    uav_msgs::FlightModeStatus last_state_;
    unsigned int path_idx_;
    double dx_prev_;
    double dy_prev_;
};

#endif
