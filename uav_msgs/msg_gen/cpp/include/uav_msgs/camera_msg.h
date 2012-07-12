/* Auto-generated by genmsg_cpp for file /home/eduardo/uavros/uav_msgs/msg/camera_msg.msg */
#ifndef UAV_MSGS_MESSAGE_CAMERA_MSG_H
#define UAV_MSGS_MESSAGE_CAMERA_MSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace uav_msgs
{
template <class ContainerAllocator>
struct camera_msg_ {
  typedef camera_msg_<ContainerAllocator> Type;

  camera_msg_()
  : change_res(0)
  {
  }

  camera_msg_(const ContainerAllocator& _alloc)
  : change_res(0)
  {
  }

  typedef int32_t _change_res_type;
  int32_t change_res;


private:
  static const char* __s_getDataType_() { return "uav_msgs/camera_msg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "c7d90e90999ece5a54c013134180c080"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 change_res\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, change_res);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, change_res);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(change_res);
    return size;
  }

  typedef boost::shared_ptr< ::uav_msgs::camera_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::uav_msgs::camera_msg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct camera_msg
typedef  ::uav_msgs::camera_msg_<std::allocator<void> > camera_msg;

typedef boost::shared_ptr< ::uav_msgs::camera_msg> camera_msgPtr;
typedef boost::shared_ptr< ::uav_msgs::camera_msg const> camera_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::uav_msgs::camera_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::uav_msgs::camera_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace uav_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::uav_msgs::camera_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::uav_msgs::camera_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::uav_msgs::camera_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c7d90e90999ece5a54c013134180c080";
  }

  static const char* value(const  ::uav_msgs::camera_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc7d90e90999ece5aULL;
  static const uint64_t static_value2 = 0x54c013134180c080ULL;
};

template<class ContainerAllocator>
struct DataType< ::uav_msgs::camera_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uav_msgs/camera_msg";
  }

  static const char* value(const  ::uav_msgs::camera_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::uav_msgs::camera_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 change_res\n\
\n\
";
  }

  static const char* value(const  ::uav_msgs::camera_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::uav_msgs::camera_msg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::uav_msgs::camera_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.change_res);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct camera_msg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::uav_msgs::camera_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::uav_msgs::camera_msg_<ContainerAllocator> & v) 
  {
    s << indent << "change_res: ";
    Printer<int32_t>::stream(s, indent + "  ", v.change_res);
  }
};


} // namespace message_operations
} // namespace ros

#endif // UAV_MSGS_MESSAGE_CAMERA_MSG_H

