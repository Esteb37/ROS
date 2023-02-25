// Generated by gencpp from file pid_control/set_point.msg
// DO NOT EDIT!


#ifndef PID_CONTROL_MESSAGE_SET_POINT_H
#define PID_CONTROL_MESSAGE_SET_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pid_control
{
template <class ContainerAllocator>
struct set_point_
{
  typedef set_point_<ContainerAllocator> Type;

  set_point_()
    : value(0.0)
    , time(0.0)  {
    }
  set_point_(const ContainerAllocator& _alloc)
    : value(0.0)
    , time(0.0)  {
  (void)_alloc;
    }



   typedef float _value_type;
  _value_type value;

   typedef float _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::pid_control::set_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pid_control::set_point_<ContainerAllocator> const> ConstPtr;

}; // struct set_point_

typedef ::pid_control::set_point_<std::allocator<void> > set_point;

typedef boost::shared_ptr< ::pid_control::set_point > set_pointPtr;
typedef boost::shared_ptr< ::pid_control::set_point const> set_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pid_control::set_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pid_control::set_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pid_control::set_point_<ContainerAllocator1> & lhs, const ::pid_control::set_point_<ContainerAllocator2> & rhs)
{
  return lhs.value == rhs.value &&
    lhs.time == rhs.time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pid_control::set_point_<ContainerAllocator1> & lhs, const ::pid_control::set_point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pid_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pid_control::set_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pid_control::set_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pid_control::set_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pid_control::set_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pid_control::set_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pid_control::set_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pid_control::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01346652ed5c09b39a6e088152e53548";
  }

  static const char* value(const ::pid_control::set_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01346652ed5c09b3ULL;
  static const uint64_t static_value2 = 0x9a6e088152e53548ULL;
};

template<class ContainerAllocator>
struct DataType< ::pid_control::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pid_control/set_point";
  }

  static const char* value(const ::pid_control::set_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pid_control::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 value\n"
"float32 time\n"
;
  }

  static const char* value(const ::pid_control::set_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pid_control::set_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pid_control::set_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pid_control::set_point_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
    s << indent << "time: ";
    Printer<float>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PID_CONTROL_MESSAGE_SET_POINT_H
