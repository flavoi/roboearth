/* Auto-generated by genmsg_cpp for file /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_srvs/srv/LoadVslamMap.srv */
#ifndef RE_SRVS_SERVICE_LOADVSLAMMAP_H
#define RE_SRVS_SERVICE_LOADVSLAMMAP_H
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

#include "ros/service_traits.h"




namespace re_srvs
{
template <class ContainerAllocator>
struct LoadVslamMapRequest_ {
  typedef LoadVslamMapRequest_<ContainerAllocator> Type;

  LoadVslamMapRequest_()
  : mapUID()
  {
  }

  LoadVslamMapRequest_(const ContainerAllocator& _alloc)
  : mapUID(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mapUID_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  mapUID;


  typedef boost::shared_ptr< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LoadVslamMapRequest
typedef  ::re_srvs::LoadVslamMapRequest_<std::allocator<void> > LoadVslamMapRequest;

typedef boost::shared_ptr< ::re_srvs::LoadVslamMapRequest> LoadVslamMapRequestPtr;
typedef boost::shared_ptr< ::re_srvs::LoadVslamMapRequest const> LoadVslamMapRequestConstPtr;


template <class ContainerAllocator>
struct LoadVslamMapResponse_ {
  typedef LoadVslamMapResponse_<ContainerAllocator> Type;

  LoadVslamMapResponse_()
  : success(false)
  {
  }

  LoadVslamMapResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LoadVslamMapResponse
typedef  ::re_srvs::LoadVslamMapResponse_<std::allocator<void> > LoadVslamMapResponse;

typedef boost::shared_ptr< ::re_srvs::LoadVslamMapResponse> LoadVslamMapResponsePtr;
typedef boost::shared_ptr< ::re_srvs::LoadVslamMapResponse const> LoadVslamMapResponseConstPtr;

struct LoadVslamMap
{

typedef LoadVslamMapRequest Request;
typedef LoadVslamMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct LoadVslamMap
} // namespace re_srvs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9d98bb71a81c635de8886fd1fc890b66";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9d98bb71a81c635dULL;
  static const uint64_t static_value2 = 0xe8886fd1fc890b66ULL;
};

template<class ContainerAllocator>
struct DataType< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re_srvs/LoadVslamMapRequest";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string mapUID\n\
\n\
\n\
";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re_srvs/LoadVslamMapResponse";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
\n\
";
  }

  static const char* value(const  ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::re_srvs::LoadVslamMapRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.mapUID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LoadVslamMapRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::re_srvs::LoadVslamMapResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LoadVslamMapResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<re_srvs::LoadVslamMap> {
  static const char* value() 
  {
    return "3dfc35c40fc8503a4b6fd8ff5668fcf2";
  }

  static const char* value(const re_srvs::LoadVslamMap&) { return value(); } 
};

template<>
struct DataType<re_srvs::LoadVslamMap> {
  static const char* value() 
  {
    return "re_srvs/LoadVslamMap";
  }

  static const char* value(const re_srvs::LoadVslamMap&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<re_srvs::LoadVslamMapRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3dfc35c40fc8503a4b6fd8ff5668fcf2";
  }

  static const char* value(const re_srvs::LoadVslamMapRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<re_srvs::LoadVslamMapRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re_srvs/LoadVslamMap";
  }

  static const char* value(const re_srvs::LoadVslamMapRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<re_srvs::LoadVslamMapResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3dfc35c40fc8503a4b6fd8ff5668fcf2";
  }

  static const char* value(const re_srvs::LoadVslamMapResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<re_srvs::LoadVslamMapResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "re_srvs/LoadVslamMap";
  }

  static const char* value(const re_srvs::LoadVslamMapResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RE_SRVS_SERVICE_LOADVSLAMMAP_H

