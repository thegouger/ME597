/* Auto-generated by genmsg_cpp for file /home/parth/Code/ME597/RobotCode/clearpath_horizon/msg/TurnSetpt.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_TURNSETPT_H
#define CLEARPATH_HORIZON_MESSAGE_TURNSETPT_H
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

#include "std_msgs/Header.h"

namespace clearpath_horizon
{
template <class ContainerAllocator>
struct TurnSetpt_ {
  typedef TurnSetpt_<ContainerAllocator> Type;

  TurnSetpt_()
  : header()
  , trans_vel(0.0)
  , turn_radius(0.0)
  , trans_accel(0.0)
  {
  }

  TurnSetpt_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , trans_vel(0.0)
  , turn_radius(0.0)
  , trans_accel(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _trans_vel_type;
  double trans_vel;

  typedef double _turn_radius_type;
  double turn_radius;

  typedef double _trans_accel_type;
  double trans_accel;


  typedef boost::shared_ptr< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::TurnSetpt_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TurnSetpt
typedef  ::clearpath_horizon::TurnSetpt_<std::allocator<void> > TurnSetpt;

typedef boost::shared_ptr< ::clearpath_horizon::TurnSetpt> TurnSetptPtr;
typedef boost::shared_ptr< ::clearpath_horizon::TurnSetpt const> TurnSetptConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::TurnSetpt_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::TurnSetpt_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > {
  static const char* value() 
  {
    return "023314e739de17bd5207788d54c661df";
  }

  static const char* value(const  ::clearpath_horizon::TurnSetpt_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x023314e739de17bdULL;
  static const uint64_t static_value2 = 0x5207788d54c661dfULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/TurnSetpt";
  }

  static const char* value(const  ::clearpath_horizon::TurnSetpt_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 trans_vel\n\
float64 turn_radius\n\
float64 trans_accel\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::clearpath_horizon::TurnSetpt_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::TurnSetpt_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.trans_vel);
    stream.next(m.turn_radius);
    stream.next(m.trans_accel);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TurnSetpt_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::TurnSetpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::TurnSetpt_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "trans_vel: ";
    Printer<double>::stream(s, indent + "  ", v.trans_vel);
    s << indent << "turn_radius: ";
    Printer<double>::stream(s, indent + "  ", v.turn_radius);
    s << indent << "trans_accel: ";
    Printer<double>::stream(s, indent + "  ", v.trans_accel);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_TURNSETPT_H

