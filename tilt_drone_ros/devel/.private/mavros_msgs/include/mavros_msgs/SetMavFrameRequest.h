// Generated by gencpp from file mavros_msgs/SetMavFrameRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H
#define MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct SetMavFrameRequest_
{
  typedef SetMavFrameRequest_<ContainerAllocator> Type;

  SetMavFrameRequest_()
    : mav_frame(0)  {
    }
  SetMavFrameRequest_(const ContainerAllocator& _alloc)
    : mav_frame(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mav_frame_type;
  _mav_frame_type mav_frame;



  enum {
    FRAME_GLOBAL = 0u,
    FRAME_LOCAL_NED = 1u,
    FRAME_MISSION = 2u,
    FRAME_GLOBAL_RELATIVE_ALT = 3u,
    FRAME_LOCAL_ENU = 4u,
    FRAME_GLOBAL_INT = 5u,
    FRAME_GLOBAL_RELATIVE_ALT_INT = 6u,
    FRAME_LOCAL_OFFSET_NED = 7u,
    FRAME_BODY_NED = 8u,
    FRAME_BODY_OFFSET_NED = 9u,
    FRAME_GLOBAL_TERRAIN_ALT = 10u,
    FRAME_GLOBAL_TERRAIN_ALT_INT = 11u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetMavFrameRequest_

typedef ::mavros_msgs::SetMavFrameRequest_<std::allocator<void> > SetMavFrameRequest;

typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest > SetMavFrameRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest const> SetMavFrameRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geographic_msgs': ['/opt/ros/kinetic/share/geographic_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'mavros_msgs': ['/home/radu/tiltUp3_ws/src/mavros/mavros_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'uuid_msgs': ['/opt/ros/kinetic/share/uuid_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4102fcf8d7971e4f06392711a40bc2cd";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4102fcf8d7971e4fULL;
  static const uint64_t static_value2 = 0x06392711a40bc2cdULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/SetMavFrameRequest";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
uint8 FRAME_GLOBAL = 0\n\
uint8 FRAME_LOCAL_NED = 1\n\
uint8 FRAME_MISSION = 2\n\
uint8 FRAME_GLOBAL_RELATIVE_ALT = 3\n\
uint8 FRAME_LOCAL_ENU = 4\n\
uint8 FRAME_GLOBAL_INT = 5\n\
uint8 FRAME_GLOBAL_RELATIVE_ALT_INT = 6\n\
uint8 FRAME_LOCAL_OFFSET_NED = 7\n\
uint8 FRAME_BODY_NED = 8\n\
uint8 FRAME_BODY_OFFSET_NED = 9\n\
uint8 FRAME_GLOBAL_TERRAIN_ALT = 10\n\
uint8 FRAME_GLOBAL_TERRAIN_ALT_INT = 11\n\
\n\
\n\
uint8 mav_frame\n\
";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mav_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMavFrameRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>& v)
  {
    s << indent << "mav_frame: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mav_frame);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H
