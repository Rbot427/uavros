#ifndef UAV_STATE_PUBLISHER_H
#define UAV_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include <list>

class FIFO{
  public:
    FIFO(int s){
      size_ = s;
      head_ = 0;
      tail_ = 0;
      elements_ = 0;
      q_ = new double[s];
    }

    ~FIFO(){
      delete [] q_;
    }

    void insert(double x){
      *(q_+head_) = x;
      head_++;
      if(head_==size_)
        head_ = 0;
      if(elements_ < size_)
        elements_++;
    }

    double operator[](int i){
      int j=tail_+i;
      if(j>=size_)
        j -= size_;
      return *(q_+j);
    }

    int size(){
      return elements_;
    }

  private:
    int size_, head_, tail_, elements_;
    double* q_;
  };

class UAVDebug{

public:
  UAVDebug();
  ~UAVDebug(){};

  
  class velo_list{
    
  public:

    velo_list(int s)
    {
      count =0;
      size = s;
    }
    
    ~velo_list()
    {
    }
    
    void add_value(double val)
    {
      if(count == size )
      {
	my_list.pop_front();
      }
      else
      {
	count++;
      }
      my_list.push_back(val);
    }
    double get_last()
    {
      return my_list.front();
    }
    
  private:
    std::list<double> my_list;
    int count;
    int size;
  };
  
  struct reading{
   
    double value;
    ros::Time time;
    
  };
  
  class integrated_accel{
    
    public:
      
      integrated_accel(int s)
      {
	size = s;
	count = 0;
	offset_set = false;
	offset = 0;
      }
      
      ~integrated_accel()
      {
      }
      
      void set_value(double val, ros::Time time)
      {
	if(!offset_set) { offset_set = true; offset = val; }

	if(count == size )
        {
	  my_list.pop_front();
        }
        else
        {
	  count++;
        }
        reading r;
	r.value = val - offset;
	r.time = time;
        my_list.push_back(r);
      }
      
      double get_list_sum()
      {
	double s=0;
	int index=0;
	reading pr ;
	for (std::list<reading>::iterator it=my_list.begin(); it!=my_list.end(); ++it)
	{
	  if(index != 0)
	  {
	    reading r = *it;
	    ros::Duration dt = r.time - pr.time;
	    double val = (pr.value + r.value)/2;
	    s += (val * dt.toSec());
	    //ROS_ERROR("index %d val %f dt %f val*dt %f sum %f", index, val, dt.toSec(), val * dt.toSec(), s);
	  }
	  pr = *it;
	  index++;
	}
	return s;
      }
      
  private:
      std::list<reading> my_list;
      int count;
      int size;
      bool offset_set;
      double offset;
  };

  void ekfCallback(nav_msgs::OdometryConstPtr p);
  void rawImuCallback(sensor_msgs::Imu imu);
  void slamCallback(geometry_msgs::PoseStampedConstPtr slam_msg);

  std::string state_pub_topic_, z_laser_topic_, z_laser_median_topic_, position_sub_topic_, vertical_laser_data_topic_, vertical_laser_frame_topic_, slam_topic_, map_topic_, body_topic_, body_map_aligned_topic_, body_stabilized_topic_, imu_topic_;

  nav_msgs::Odometry state_;
  ros::Publisher vel_pub_;
  ros::Publisher ac_pub_;
  ros::Publisher last_vel_pub_;
  ros::Publisher slam_vel_pub_;
  ros::Publisher slam_acc_pub_;
  ros::Subscriber ekf_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber slam_sub_;

  tf::StampedTransform Pan2BodyTransform_;

  tf::TransformListener tf_;
  tf::TransformBroadcaster tf_broadcaster;
  
    FIFO z_fifo_;
  FIFO z_time_fifo_;
  
  
  integrated_accel* x_integrated_;
  integrated_accel* y_integrated_;
  velo_list* x_velo_;
  velo_list* y_velo_;
  
  geometry_msgs::PointStamped last_slam_pos_;
  geometry_msgs::PointStamped current_slam_pos_;
  
  geometry_msgs::PointStamped last_slam_vel_;
  geometry_msgs::PointStamped current_slam_vel_;
  
  geometry_msgs::PointStamped current_slam_acc_;
  
  

};

#endif
