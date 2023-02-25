// Generated by gencpp from file pid_control/motor_output.msg
// DO NOT EDIT!


#ifndef PID_CONTROL_MESSAGE_MOTOR_OUTPUT_H
#define PID_CONTROL_MESSAGE_MOTOR_OUTPUT_H


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
struct motor_output_
{
  typedef motor_output_<ContainerAllocator> Type;

  motor_output_()
    : output(0.0)
    , time(0.0)
    , status()  {
    }
  motor_output_(const ContainerAllocator& _alloc)
    : output(0.0)
    , time(0.0)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef float _output_type;
  _output_type output;

   typedef double _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::pid_control::motor_output_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pid_control::motor_output_<ContainerAllocator> const> ConstPtr;

}; // struct motor_output_

typedef ::pid_control::motor_output_<std::allocator<void> > motor_output;

typedef boost::shared_ptr< ::pid_control::motor_output > motor_outputPtr;
typedef boost::shared_ptr< ::pid_control::motor_output const> motor_outputConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pid_control::motor_output_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pid_control::motor_output_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pid_control::motor_output_<ContainerAllocator1> & lhs, const ::pid_control::motor_output_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output &&
    lhs.time == rhs.time &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pid_control::motor_output_<ContainerAllocator1> & lhs, const ::pid_control::motor_output_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pid_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pid_control::motor_output_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pid_control::motor_output_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pid_control::motor_output_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pid_control::motor_output_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pid_control::motor_output_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pid_control::motor_output_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pid_control::motor_output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f371a723b35f6884908910ed6144b2d";
  }

  static const char* value(const ::pid_control::motor_output_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f371a723b35f688ULL;
  static const uint64_t static_value2 = 0x4908910ed6144b2dULL;
};

template<class ContainerAllocator>
struct DataType< ::pid_control::motor_output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pid_control/motor_output";
  }

  static const char* value(const ::pid_control::motor_output_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pid_control::motor_output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 output\n"
"float64 time\n"
"string status\n"
;
  }

  static const char* value(const ::pid_control::motor_output_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pid_control::motor_output_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
      stream.next(m.time);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct motor_output_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pid_control::motor_output_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pid_control::motor_output_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    Printer<float>::stream(s, indent + "  ", v.output);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PID_CONTROL_MESSAGE_MOTOR_OUTPUT_H
