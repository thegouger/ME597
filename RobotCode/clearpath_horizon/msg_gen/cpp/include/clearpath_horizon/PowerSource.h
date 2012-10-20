/* Auto-generated by genmsg_cpp for file /home/wavelab/TeamAwesome/ME597/RobotCode/clearpath_horizon/msg/PowerSource.msg */
#ifndef CLEARPATH_HORIZON_MESSAGE_POWERSOURCE_H
#define CLEARPATH_HORIZON_MESSAGE_POWERSOURCE_H
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


namespace clearpath_horizon
{
template <class ContainerAllocator>
struct PowerSource_ {
  typedef PowerSource_<ContainerAllocator> Type;

  PowerSource_()
  : charge(0.0)
  , capacity(0)
  , present(false)
  , in_use(false)
  , description(0)
  {
  }

  PowerSource_(const ContainerAllocator& _alloc)
  : charge(0.0)
  , capacity(0)
  , present(false)
  , in_use(false)
  , description(0)
  {
  }

  typedef float _charge_type;
  float charge;

  typedef int16_t _capacity_type;
  int16_t capacity;

  typedef uint8_t _present_type;
  uint8_t present;

  typedef uint8_t _in_use_type;
  uint8_t in_use;

  typedef uint8_t _description_type;
  uint8_t description;


  typedef boost::shared_ptr< ::clearpath_horizon::PowerSource_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_horizon::PowerSource_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PowerSource
typedef  ::clearpath_horizon::PowerSource_<std::allocator<void> > PowerSource;

typedef boost::shared_ptr< ::clearpath_horizon::PowerSource> PowerSourcePtr;
typedef boost::shared_ptr< ::clearpath_horizon::PowerSource const> PowerSourceConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::clearpath_horizon::PowerSource_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::clearpath_horizon::PowerSource_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace clearpath_horizon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::PowerSource_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::clearpath_horizon::PowerSource_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::clearpath_horizon::PowerSource_<ContainerAllocator> > {
  static const char* value() 
  {
    return "adbe384d7d69a337a7f2b6bf1d0139cb";
  }

  static const char* value(const  ::clearpath_horizon::PowerSource_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xadbe384d7d69a337ULL;
  static const uint64_t static_value2 = 0xa7f2b6bf1d0139cbULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_horizon::PowerSource_<ContainerAllocator> > {
  static const char* value() 
  {
    return "clearpath_horizon/PowerSource";
  }

  static const char* value(const  ::clearpath_horizon::PowerSource_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::clearpath_horizon::PowerSource_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 charge\n\
int16 capacity\n\
bool present\n\
bool in_use\n\
uint8 description\n\
\n\
";
  }

  static const char* value(const  ::clearpath_horizon::PowerSource_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::clearpath_horizon::PowerSource_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::clearpath_horizon::PowerSource_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.charge);
    stream.next(m.capacity);
    stream.next(m.present);
    stream.next(m.in_use);
    stream.next(m.description);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PowerSource_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_horizon::PowerSource_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::clearpath_horizon::PowerSource_<ContainerAllocator> & v) 
  {
    s << indent << "charge: ";
    Printer<float>::stream(s, indent + "  ", v.charge);
    s << indent << "capacity: ";
    Printer<int16_t>::stream(s, indent + "  ", v.capacity);
    s << indent << "present: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.present);
    s << indent << "in_use: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.in_use);
    s << indent << "description: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.description);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_HORIZON_MESSAGE_POWERSOURCE_H
