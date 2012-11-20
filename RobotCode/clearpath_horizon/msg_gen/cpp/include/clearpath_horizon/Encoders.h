/* Auto-generated by genmsg_cpp for file /home/parth/Code/ME597/RobotCode/clearpath_horizon/msg/Encoders.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_ENCODERS_H
#define CLEARPATH_HORIZON_MESSAGE_ENCODERS_H
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
#include "clearpath_horizon/Encoder.h"

namespace clearpath_horizon
{
template <class ContainerAllocator>
struct Encoders_ {
  typedef Encoders_<ContainerAllocator> Type;

  Encoders_()
  : header()
  , encoders()
  {
  }

  Encoders_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , encoders(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::clearpath_horizon::Encoder_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::clearpath_horizon::Encoder_<ContainerAllocator> >::other >  _encoders_type;
  std::vector< ::clearpath_horizon::Encoder_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::clearpath_horizon::Encoder_<ContainerAllocator> >::other >  encoders;


  typedef boost::shared_ptr< ::clearpath_horizon::Encoders_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::Encoders_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Encoders
typedef  ::clearpath_horizon::Encoders_<std::allocator<void> > Encoders;

typedef boost::shared_ptr< ::clearpath_horizon::Encoders> EncodersPtr;
typedef boost::shared_ptr< ::clearpath_horizon::Encoders const> EncodersConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::Encoders_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::Encoders_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::Encoders_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::Encoders_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::Encoders_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2ea748832c2014369ffabd316d5aad8c";
  }

  static const char* value(const  ::clearpath_horizon::Encoders_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2ea748832c201436ULL;
  static const uint64_t static_value2 = 0x9ffabd316d5aad8cULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::Encoders_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/Encoders";
  }

  static const char* value(const  ::clearpath_horizon::Encoders_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::Encoders_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
Encoder[] encoders\n\
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
================================================================================\n\
MSG: clearpath_horizon/Encoder\n\
float64 travel\n\
float64 speed\n\
\n\
";
  }

  static const char* value(const  ::clearpath_horizon::Encoders_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::Encoders_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::Encoders_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::Encoders_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.encoders);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Encoders_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::Encoders_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::Encoders_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "encoders[]" << std::endl;
    for (size_t i = 0; i < v.encoders.size(); ++i)
    {
      s << indent << "  encoders[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::clearpath_horizon::Encoder_<ContainerAllocator> >::stream(s, indent + "    ", v.encoders[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_ENCODERS_H

