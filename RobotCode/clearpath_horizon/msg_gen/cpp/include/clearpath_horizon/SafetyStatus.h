/* Auto-generated by genmsg_cpp for file /home/wavelab/TeamAwesome/ME597/RobotCode/clearpath_horizon/msg/SafetyStatus.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_SAFETYSTATUS_H
#define CLEARPATH_HORIZON_MESSAGE_SAFETYSTATUS_H
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
struct SafetyStatus_ {
  typedef SafetyStatus_<ContainerAllocator> Type;

  SafetyStatus_()
  : header()
  , flags(0)
  , estop(false)
  {
  }

  SafetyStatus_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , flags(0)
  , estop(false)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint16_t _flags_type;
  uint16_t flags;

  typedef uint8_t _estop_type;
  uint8_t estop;


  typedef boost::shared_ptr< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::SafetyStatus_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SafetyStatus
typedef  ::clearpath_horizon::SafetyStatus_<std::allocator<void> > SafetyStatus;

typedef boost::shared_ptr< ::clearpath_horizon::SafetyStatus> SafetyStatusPtr;
typedef boost::shared_ptr< ::clearpath_horizon::SafetyStatus const> SafetyStatusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::SafetyStatus_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::SafetyStatus_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cf78d6042b92d64ebda55641e06d66fa";
  }

  static const char* value(const  ::clearpath_horizon::SafetyStatus_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcf78d6042b92d64eULL;
  static const uint64_t static_value2 = 0xbda55641e06d66faULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/SafetyStatus";
  }

  static const char* value(const  ::clearpath_horizon::SafetyStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
uint16 flags\n\
bool estop\n\
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

  static const char* value(const  ::clearpath_horizon::SafetyStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::SafetyStatus_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.flags);
    stream.next(m.estop);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SafetyStatus_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::SafetyStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::SafetyStatus_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "flags: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.flags);
    s << indent << "estop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.estop);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_SAFETYSTATUS_H
