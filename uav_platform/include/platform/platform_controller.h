#ifndef PLATFORM_CONTROLLER_H
#define PLATFORM_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <uav_msgs/mode_msg.h>
#include <uav_msgs/camera_msg.h>
#include <string.h>
#include <math.h>

#define PI  3.14159265

#define DISTANCE_FROM_PLATFORM  2       //in meters
#define WIDTH_PLATFORM			0.3
#define MARKER_ANGLE            0       //angle of the marker on respect to the plataform
#define INTERVALS_ANGLE         15      //angle to mark for each zone
#define HOVER_ABOVE_PLATFORM    1.0     //in meters
#define PLATFORM_ANGLE          180     //In angles

/* Track modes */
#define ALIGN_FRONT 	1
#define ALIGN_TOP   	2
#define ROTATE      	3
#define LAND        	4
#define CHANGE_FRONT 	5
#define CHANGE_BOTTOM	6

#define FRONT_CAMERA    0
#define BOTTOM_CAMERA   1

const ros::Duration IN_RANGE_TIME(5.0);	// in seconds
const double	IN_RANGE_DIST =	0.20;	// in meters

typedef struct {
	double x;
	double y;
	double z;
} position;

typedef struct {
	double x;
	double y;
	double z;
} rotation;

typedef struct {
	position pos;
	rotation rot;
} pose;

class platform_controller {
	public:
		platform_controller(ros::NodeHandle &n);
		~platform_controller();

	private:
		void init();
		void get_params();
		void create_publishers();
		void create_subscribers();
		
		void transform_callback(const tf::tfMessageConstPtr msg);
		void mode_callback(const uav_msgs::mode_msg msg);
		void align_done_callback(const ros::TimerEvent&);

		void align_front(tf::tfMessageConstPtr msg);
		void align_top(tf::tfMessageConstPtr msg, int camera, bool rotate);
		void land(tf::tfMessageConstPtr msg);

		void check_time(tf::tfMessageConstPtr msg);
		bool check_in_range(tf::tfMessageConstPtr msg);

		void publish_goal(double x, double y, double z, double theta);
		void get_transform(std::string parent, std::string child,
						   tf::StampedTransform &transform);
		void get_pose_from_msg(tf::tfMessageConstPtr msg);
		void get_pose_from_tf(double pos[3], double quat[4], 
					 		  tf::StampedTransform transform);
		void quat_to_euler(double q[4], double r[3]);

		ros::NodeHandle n_;
		ros::Publisher goal_pose_pub_;
		ros::Subscriber tf_sub_;
		ros::Subscriber track_sub_;

		ros::Timer timer_;

		int track_mode_;
		std::string align_front_;
		std::string align_top_;
		pose pose_;
};

#endif
