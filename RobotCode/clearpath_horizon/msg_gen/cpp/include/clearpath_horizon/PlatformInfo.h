/* Auto-generated by genmsg_cpp for file /home/parth/Code/ME597/RobotCode/clearpath_horizon/msg/PlatformInfo.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_PLATFORMINFO_H
#define CLEARPATH_HORIZON_MESSAGE_PLATFORMINFO_H
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
struct PlatformInfo_ {
  typedef PlatformInfo_<ContainerAllocator> Type;

  PlatformInfo_()
  : header()
  , model()
  , revision(0)
  , serial(0)
  {
  }

  PlatformInfo_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , model(_alloc)
  , revision(0)
  , serial(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _model_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  model;

  typedef int8_t _revision_type;
  int8_t revision;

  typedef uint32_t _serial_type;
  uint32_t serial;


  typedef boost::shared_ptr< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::PlatformInfo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlatformInfo
typedef  ::clearpath_horizon::PlatformInfo_<std::allocator<void> > PlatformInfo;

typedef boost::shared_ptr< ::clearpath_horizon::PlatformInfo> PlatformInfoPtr;
typedef boost::shared_ptr< ::clearpath_horizon::PlatformInfo const> PlatformInfoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::PlatformInfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::PlatformInfo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ff95c25c6ef78f06bbb7ef85aad5735e";
  }

  static const char* value(const  ::clearpath_horizon::PlatformInfo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xff95c25c6ef78f06ULL;
  static const uint64_t static_value2 = 0xbbb7ef85aad5735eULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/PlatformInfo";
  }

  static const char* value(const  ::clearpath_horizon::PlatformInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
string model\n\
int8 revision\n\
uint32 serial\n\
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

  static const char* value(const  ::clearpath_horizon::PlatformInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::PlatformInfo_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.model);
    stream.next(m.revision);
    stream.next(m.serial);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlatformInfo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::PlatformInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::PlatformInfo_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "model: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.model);
    s << indent << "revision: ";
    Printer<int8_t>::stream(s, indent + "  ", v.revision);
    s << indent << "serial: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.serial);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_PLATFORMINFO_H

