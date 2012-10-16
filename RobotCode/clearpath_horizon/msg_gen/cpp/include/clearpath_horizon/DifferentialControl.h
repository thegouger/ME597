/* Auto-generated by genmsg_cpp for file /home/wavelab/TeamAwesome/ME597/RobotCode/clearpath_horizon/msg/DifferentialControl.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_DIFFERENTIALCONTROL_H
#define CLEARPATH_HORIZON_MESSAGE_DIFFERENTIALCONTROL_H
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
struct DifferentialControl_ {
  typedef DifferentialControl_<ContainerAllocator> Type;

  DifferentialControl_()
  : header()
  , left_p(0.0)
  , left_i(0.0)
  , left_d(0.0)
  , left_ff(0.0)
  , left_sc(0.0)
  , left_il(0.0)
  , right_p(0.0)
  , right_i(0.0)
  , right_d(0.0)
  , right_ff(0.0)
  , right_sc(0.0)
  , right_il(0.0)
  {
  }

  DifferentialControl_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , left_p(0.0)
  , left_i(0.0)
  , left_d(0.0)
  , left_ff(0.0)
  , left_sc(0.0)
  , left_il(0.0)
  , right_p(0.0)
  , right_i(0.0)
  , right_d(0.0)
  , right_ff(0.0)
  , right_sc(0.0)
  , right_il(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _left_p_type;
  double left_p;

  typedef double _left_i_type;
  double left_i;

  typedef double _left_d_type;
  double left_d;

  typedef double _left_ff_type;
  double left_ff;

  typedef double _left_sc_type;
  double left_sc;

  typedef double _left_il_type;
  double left_il;

  typedef double _right_p_type;
  double right_p;

  typedef double _right_i_type;
  double right_i;

  typedef double _right_d_type;
  double right_d;

  typedef double _right_ff_type;
  double right_ff;

  typedef double _right_sc_type;
  double right_sc;

  typedef double _right_il_type;
  double right_il;


  typedef boost::shared_ptr< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::DifferentialControl_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DifferentialControl
typedef  ::clearpath_horizon::DifferentialControl_<std::allocator<void> > DifferentialControl;

typedef boost::shared_ptr< ::clearpath_horizon::DifferentialControl> DifferentialControlPtr;
typedef boost::shared_ptr< ::clearpath_horizon::DifferentialControl const> DifferentialControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::DifferentialControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::DifferentialControl_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7fc8e33b74ccc82007a751ce82aba911";
  }

  static const char* value(const  ::clearpath_horizon::DifferentialControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7fc8e33b74ccc820ULL;
  static const uint64_t static_value2 = 0x07a751ce82aba911ULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/DifferentialControl";
  }

  static const char* value(const  ::clearpath_horizon::DifferentialControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 left_p\n\
float64 left_i\n\
float64 left_d\n\
float64 left_ff\n\
float64 left_sc\n\
float64 left_il\n\
float64 right_p\n\
float64 right_i\n\
float64 right_d\n\
float64 right_ff\n\
float64 right_sc\n\
float64 right_il\n\
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

  static const char* value(const  ::clearpath_horizon::DifferentialControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::clearpath_horizon::DifferentialControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.left_p);
    stream.next(m.left_i);
    stream.next(m.left_d);
    stream.next(m.left_ff);
    stream.next(m.left_sc);
    stream.next(m.left_il);
    stream.next(m.right_p);
    stream.next(m.right_i);
    stream.next(m.right_d);
    stream.next(m.right_ff);
    stream.next(m.right_sc);
    stream.next(m.right_il);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DifferentialControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::DifferentialControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::DifferentialControl_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "left_p: ";
    Printer<double>::stream(s, indent + "  ", v.left_p);
    s << indent << "left_i: ";
    Printer<double>::stream(s, indent + "  ", v.left_i);
    s << indent << "left_d: ";
    Printer<double>::stream(s, indent + "  ", v.left_d);
    s << indent << "left_ff: ";
    Printer<double>::stream(s, indent + "  ", v.left_ff);
    s << indent << "left_sc: ";
    Printer<double>::stream(s, indent + "  ", v.left_sc);
    s << indent << "left_il: ";
    Printer<double>::stream(s, indent + "  ", v.left_il);
    s << indent << "right_p: ";
    Printer<double>::stream(s, indent + "  ", v.right_p);
    s << indent << "right_i: ";
    Printer<double>::stream(s, indent + "  ", v.right_i);
    s << indent << "right_d: ";
    Printer<double>::stream(s, indent + "  ", v.right_d);
    s << indent << "right_ff: ";
    Printer<double>::stream(s, indent + "  ", v.right_ff);
    s << indent << "right_sc: ";
    Printer<double>::stream(s, indent + "  ", v.right_sc);
    s << indent << "right_il: ";
    Printer<double>::stream(s, indent + "  ", v.right_il);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_DIFFERENTIALCONTROL_H

