// Generated by gencpp from file reto/set_point.msg
// DO NOT EDIT!


#ifndef RETO_MESSAGE_SET_POINT_H
#define RETO_MESSAGE_SET_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace reto
{
template <class ContainerAllocator>
struct set_point_
{
  typedef set_point_<ContainerAllocator> Type;

  set_point_()
    : value(0.0)
    , mode(0)  {
    }
  set_point_(const ContainerAllocator& _alloc)
    : value(0.0)
    , mode(0)  {
  (void)_alloc;
    }



   typedef float _value_type;
  _value_type value;

   typedef int32_t _mode_type;
  _mode_type mode;





  typedef boost::shared_ptr< ::reto::set_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reto::set_point_<ContainerAllocator> const> ConstPtr;

}; // struct set_point_

typedef ::reto::set_point_<std::allocator<void> > set_point;

typedef boost::shared_ptr< ::reto::set_point > set_pointPtr;
typedef boost::shared_ptr< ::reto::set_point const> set_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reto::set_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reto::set_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reto::set_point_<ContainerAllocator1> & lhs, const ::reto::set_point_<ContainerAllocator2> & rhs)
{
  return lhs.value == rhs.value &&
    lhs.mode == rhs.mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reto::set_point_<ContainerAllocator1> & lhs, const ::reto::set_point_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reto

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::reto::set_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reto::set_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reto::set_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reto::set_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reto::set_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reto::set_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reto::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "13ab31516f5e73a712f39652b01f4fe5";
  }

  static const char* value(const ::reto::set_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x13ab31516f5e73a7ULL;
  static const uint64_t static_value2 = 0x12f39652b01f4fe5ULL;
};

template<class ContainerAllocator>
struct DataType< ::reto::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reto/set_point";
  }

  static const char* value(const ::reto::set_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reto::set_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 value\n"
"int32 mode\n"
;
  }

  static const char* value(const ::reto::set_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reto::set_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
      stream.next(m.mode);
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
struct Printer< ::reto::set_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reto::set_point_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RETO_MESSAGE_SET_POINT_H
