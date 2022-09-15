// Generated by gencpp from file visp_hand2eye_calibration/compute_effector_cameraResponse.msg
// DO NOT EDIT!


#ifndef VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERARESPONSE_H
#define VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERARESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Transform.h>

namespace visp_hand2eye_calibration
{
template <class ContainerAllocator>
struct compute_effector_cameraResponse_
{
  typedef compute_effector_cameraResponse_<ContainerAllocator> Type;

  compute_effector_cameraResponse_()
    : effector_camera()  {
    }
  compute_effector_cameraResponse_(const ContainerAllocator& _alloc)
    : effector_camera(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _effector_camera_type;
  _effector_camera_type effector_camera;





  typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> const> ConstPtr;

}; // struct compute_effector_cameraResponse_

typedef ::visp_hand2eye_calibration::compute_effector_cameraResponse_<std::allocator<void> > compute_effector_cameraResponse;

typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraResponse > compute_effector_cameraResponsePtr;
typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraResponse const> compute_effector_cameraResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator1> & lhs, const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator2> & rhs)
{
  return lhs.effector_camera == rhs.effector_camera;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator1> & lhs, const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace visp_hand2eye_calibration

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e28a9ea34e6e135a6309cbdf6fb0ad0d";
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe28a9ea34e6e135aULL;
  static const uint64_t static_value2 = 0x6309cbdf6fb0ad0dULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_hand2eye_calibration/compute_effector_cameraResponse";
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Transform effector_camera\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.effector_camera);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct compute_effector_cameraResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visp_hand2eye_calibration::compute_effector_cameraResponse_<ContainerAllocator>& v)
  {
    s << indent << "effector_camera: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.effector_camera);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERARESPONSE_H
