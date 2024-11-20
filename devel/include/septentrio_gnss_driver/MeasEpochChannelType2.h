// Generated by gencpp from file septentrio_gnss_driver/MeasEpochChannelType2.msg
// DO NOT EDIT!


#ifndef SEPTENTRIO_GNSS_DRIVER_MESSAGE_MEASEPOCHCHANNELTYPE2_H
#define SEPTENTRIO_GNSS_DRIVER_MESSAGE_MEASEPOCHCHANNELTYPE2_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace septentrio_gnss_driver
{
template <class ContainerAllocator>
struct MeasEpochChannelType2_
{
  typedef MeasEpochChannelType2_<ContainerAllocator> Type;

  MeasEpochChannelType2_()
    : type(0)
    , lock_time(0)
    , cn0(0)
    , offsets_msb(0)
    , carrier_msb(0)
    , obs_info(0)
    , code_offset_lsb(0)
    , carrier_lsb(0)
    , doppler_offset_lsb(0)  {
    }
  MeasEpochChannelType2_(const ContainerAllocator& _alloc)
    : type(0)
    , lock_time(0)
    , cn0(0)
    , offsets_msb(0)
    , carrier_msb(0)
    , obs_info(0)
    , code_offset_lsb(0)
    , carrier_lsb(0)
    , doppler_offset_lsb(0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _lock_time_type;
  _lock_time_type lock_time;

   typedef uint8_t _cn0_type;
  _cn0_type cn0;

   typedef uint8_t _offsets_msb_type;
  _offsets_msb_type offsets_msb;

   typedef int8_t _carrier_msb_type;
  _carrier_msb_type carrier_msb;

   typedef uint8_t _obs_info_type;
  _obs_info_type obs_info;

   typedef uint16_t _code_offset_lsb_type;
  _code_offset_lsb_type code_offset_lsb;

   typedef uint16_t _carrier_lsb_type;
  _carrier_lsb_type carrier_lsb;

   typedef uint16_t _doppler_offset_lsb_type;
  _doppler_offset_lsb_type doppler_offset_lsb;





  typedef boost::shared_ptr< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> const> ConstPtr;

}; // struct MeasEpochChannelType2_

typedef ::septentrio_gnss_driver::MeasEpochChannelType2_<std::allocator<void> > MeasEpochChannelType2;

typedef boost::shared_ptr< ::septentrio_gnss_driver::MeasEpochChannelType2 > MeasEpochChannelType2Ptr;
typedef boost::shared_ptr< ::septentrio_gnss_driver::MeasEpochChannelType2 const> MeasEpochChannelType2ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator1> & lhs, const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.lock_time == rhs.lock_time &&
    lhs.cn0 == rhs.cn0 &&
    lhs.offsets_msb == rhs.offsets_msb &&
    lhs.carrier_msb == rhs.carrier_msb &&
    lhs.obs_info == rhs.obs_info &&
    lhs.code_offset_lsb == rhs.code_offset_lsb &&
    lhs.carrier_lsb == rhs.carrier_lsb &&
    lhs.doppler_offset_lsb == rhs.doppler_offset_lsb;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator1> & lhs, const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace septentrio_gnss_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "204efbe40d2e56cd210b50c219f3a400";
  }

  static const char* value(const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x204efbe40d2e56cdULL;
  static const uint64_t static_value2 = 0x210b50c219f3a400ULL;
};

template<class ContainerAllocator>
struct DataType< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "septentrio_gnss_driver/MeasEpochChannelType2";
  }

  static const char* value(const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MeasEpochChannelType2 block\n"
"   \n"
"uint8  type\n"
"uint8  lock_time          # s\n"
"uint8  cn0                # 0.25 dB-Hz\n"
"uint8  offsets_msb        # 65.536 m or 65.536 Hz\n"
"int8   carrier_msb        # 65.536 cycles\n"
"uint8  obs_info\n"
"uint16 code_offset_lsb    # 0.001 m\n"
"uint16 carrier_lsb        # 0.001 cycles\n"
"uint16 doppler_offset_lsb # 0.0001 Hz\n"
;
  }

  static const char* value(const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.lock_time);
      stream.next(m.cn0);
      stream.next(m.offsets_msb);
      stream.next(m.carrier_msb);
      stream.next(m.obs_info);
      stream.next(m.code_offset_lsb);
      stream.next(m.carrier_lsb);
      stream.next(m.doppler_offset_lsb);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MeasEpochChannelType2_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::septentrio_gnss_driver::MeasEpochChannelType2_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "lock_time: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lock_time);
    s << indent << "cn0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cn0);
    s << indent << "offsets_msb: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.offsets_msb);
    s << indent << "carrier_msb: ";
    Printer<int8_t>::stream(s, indent + "  ", v.carrier_msb);
    s << indent << "obs_info: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.obs_info);
    s << indent << "code_offset_lsb: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.code_offset_lsb);
    s << indent << "carrier_lsb: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.carrier_lsb);
    s << indent << "doppler_offset_lsb: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.doppler_offset_lsb);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEPTENTRIO_GNSS_DRIVER_MESSAGE_MEASEPOCHCHANNELTYPE2_H