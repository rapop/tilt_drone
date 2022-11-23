// Generated by gencpp from file mavros_msgs/FileOpen.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_FILEOPEN_H
#define MAVROS_MSGS_MESSAGE_FILEOPEN_H

#include <ros/service_traits.h>


#include <mavros_msgs/FileOpenRequest.h>
#include <mavros_msgs/FileOpenResponse.h>


namespace mavros_msgs
{

struct FileOpen
{

typedef FileOpenRequest Request;
typedef FileOpenResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FileOpen
} // namespace mavros_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mavros_msgs::FileOpen > {
  static const char* value()
  {
    return "99a3f49cc67b91477cf49ff15c42af0e";
  }

  static const char* value(const ::mavros_msgs::FileOpen&) { return value(); }
};

template<>
struct DataType< ::mavros_msgs::FileOpen > {
  static const char* value()
  {
    return "mavros_msgs/FileOpen";
  }

  static const char* value(const ::mavros_msgs::FileOpen&) { return value(); }
};


// service_traits::MD5Sum< ::mavros_msgs::FileOpenRequest> should match 
// service_traits::MD5Sum< ::mavros_msgs::FileOpen > 
template<>
struct MD5Sum< ::mavros_msgs::FileOpenRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::FileOpen >::value();
  }
  static const char* value(const ::mavros_msgs::FileOpenRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::FileOpenRequest> should match 
// service_traits::DataType< ::mavros_msgs::FileOpen > 
template<>
struct DataType< ::mavros_msgs::FileOpenRequest>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::FileOpen >::value();
  }
  static const char* value(const ::mavros_msgs::FileOpenRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mavros_msgs::FileOpenResponse> should match 
// service_traits::MD5Sum< ::mavros_msgs::FileOpen > 
template<>
struct MD5Sum< ::mavros_msgs::FileOpenResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mavros_msgs::FileOpen >::value();
  }
  static const char* value(const ::mavros_msgs::FileOpenResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mavros_msgs::FileOpenResponse> should match 
// service_traits::DataType< ::mavros_msgs::FileOpen > 
template<>
struct DataType< ::mavros_msgs::FileOpenResponse>
{
  static const char* value()
  {
    return DataType< ::mavros_msgs::FileOpen >::value();
  }
  static const char* value(const ::mavros_msgs::FileOpenResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_FILEOPEN_H
