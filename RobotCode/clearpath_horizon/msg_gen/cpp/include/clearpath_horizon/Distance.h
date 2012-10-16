/* Auto-generated by genmsg_cpp for file /home/wavelab/TeamAwesome/ME597/RobotCode/clearpath_horizon/msg/Distance.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_DISTANCE_H
#define CLEARPATH_HORIZON_MESSAGE_DISTANCE_H
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
struct Distance_ {
  typedef Distance_<ContainerAllocator> Type;

  Distance_()
  : header()
  , distances()
  {
  }

  Distance_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , distances(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _distances_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  distances;


  typedef boost::shared_ptr< ::clearpath_horizon::Distance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::Distance_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Distance
typedef  ::clearpath_horizon::Distance_<std::allocator<void> > Distance;

typedef boost::shared_ptr< ::clearpath_horizon::Distance> DistancePtr;
typedef boost::shared_ptr< ::clearpath_horizon::Distance const> DistanceConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::Distance_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::Distance_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::Distance_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::Distance_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::Distance_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5cd97954696edc5d59dd80cb0218765a";
  }

  static const char* value(const  ::clearpath_horizon::Distance_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5cd97954696edc5dULL;
  static const uint64_t static_value2 = 0x59dd80cb0218765aULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::Distance_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/Distance";
  }

  static const char* value(const  ::clearpath_horizon::Distance_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::Distance_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64[] distances\n\
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

  static const char* value(const  ::clearpath_horizon::Distance_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::Distance_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::Distance_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::Distance_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.distances);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Distance_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::Distance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::Distance_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "distances[]" << std::endl;
    for (size_t i = 0; i < v.distances.size(); ++i)
    {
      s << indent << "  distances[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.distances[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_DISTANCE_H

