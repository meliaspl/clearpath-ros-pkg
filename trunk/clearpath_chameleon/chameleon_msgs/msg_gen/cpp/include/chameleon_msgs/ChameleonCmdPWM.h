/* Auto-generated by genmsg_cpp for file /home/mpurvis/Clearpath/r200/software/ros-pkg/chameleon_msgs/msg/ChameleonCmdPWM.msg */
#ifndef CHAMELEON_MSGS_MESSAGE_CHAMELEONCMDPWM_H
#define CHAMELEON_MSGS_MESSAGE_CHAMELEONCMDPWM_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace chameleon_msgs
{
template <class ContainerAllocator>
struct ChameleonCmdPWM_ : public ros::Message
{
  typedef ChameleonCmdPWM_<ContainerAllocator> Type;

  ChameleonCmdPWM_()
  : left(0.0)
  , right(0.0)
  {
  }

  ChameleonCmdPWM_(const ContainerAllocator& _alloc)
  : left(0.0)
  , right(0.0)
  {
  }

  typedef float _left_type;
  float left;

  typedef float _right_type;
  float right;


private:
  static const char* __s_getDataType_() { return "chameleon_msgs/ChameleonCmdPWM"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "3a927990ab5d5c3d628e2d52b8533e52"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 left   # 0..1\n\
float32 right  # 0..1\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, left);
    ros::serialization::serialize(stream, right);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, left);
    ros::serialization::deserialize(stream, right);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(left);
    size += ros::serialization::serializationLength(right);
    return size;
  }

  typedef boost::shared_ptr< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator>  const> ConstPtr;
}; // struct ChameleonCmdPWM
typedef  ::chameleon_msgs::ChameleonCmdPWM_<std::allocator<void> > ChameleonCmdPWM;

typedef boost::shared_ptr< ::chameleon_msgs::ChameleonCmdPWM> ChameleonCmdPWMPtr;
typedef boost::shared_ptr< ::chameleon_msgs::ChameleonCmdPWM const> ChameleonCmdPWMConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace chameleon_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3a927990ab5d5c3d628e2d52b8533e52";
  }

  static const char* value(const  ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3a927990ab5d5c3dULL;
  static const uint64_t static_value2 = 0x628e2d52b8533e52ULL;
};

template<class ContainerAllocator>
struct DataType< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "chameleon_msgs/ChameleonCmdPWM";
  }

  static const char* value(const  ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 left   # 0..1\n\
float32 right  # 0..1\n\
";
  }

  static const char* value(const  ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.left);
    stream.next(m.right);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ChameleonCmdPWM_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::chameleon_msgs::ChameleonCmdPWM_<ContainerAllocator> & v) 
  {
    s << indent << "left: ";
    Printer<float>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<float>::stream(s, indent + "  ", v.right);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CHAMELEON_MSGS_MESSAGE_CHAMELEONCMDPWM_H
