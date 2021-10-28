// Generated by gencpp from file ublox_serial/gps.msg
// DO NOT EDIT!


#ifndef UBLOX_SERIAL_MESSAGE_GPS_H
#define UBLOX_SERIAL_MESSAGE_GPS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_serial
{
template <class ContainerAllocator>
struct gps_
{
  typedef gps_<ContainerAllocator> Type;

  gps_()
    : header()
    , gpsTime()
    , status()
    , lat()
    , nsIndicator()
    , lon()
    , ewIndicator()
    , spd()
    , cog()
    , date()
    , mv()
    , mvEW()
    , poseMode()
    , navStatus()
    , checksum()
    , CrLf()  {
    }
  gps_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , gpsTime(_alloc)
    , status(_alloc)
    , lat(_alloc)
    , nsIndicator(_alloc)
    , lon(_alloc)
    , ewIndicator(_alloc)
    , spd(_alloc)
    , cog(_alloc)
    , date(_alloc)
    , mv(_alloc)
    , mvEW(_alloc)
    , poseMode(_alloc)
    , navStatus(_alloc)
    , checksum(_alloc)
    , CrLf(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _gpsTime_type;
  _gpsTime_type gpsTime;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _lat_type;
  _lat_type lat;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _nsIndicator_type;
  _nsIndicator_type nsIndicator;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _lon_type;
  _lon_type lon;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _ewIndicator_type;
  _ewIndicator_type ewIndicator;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _spd_type;
  _spd_type spd;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cog_type;
  _cog_type cog;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _date_type;
  _date_type date;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mv_type;
  _mv_type mv;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mvEW_type;
  _mvEW_type mvEW;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _poseMode_type;
  _poseMode_type poseMode;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _navStatus_type;
  _navStatus_type navStatus;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _checksum_type;
  _checksum_type checksum;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _CrLf_type;
  _CrLf_type CrLf;





  typedef boost::shared_ptr< ::ublox_serial::gps_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_serial::gps_<ContainerAllocator> const> ConstPtr;

}; // struct gps_

typedef ::ublox_serial::gps_<std::allocator<void> > gps;

typedef boost::shared_ptr< ::ublox_serial::gps > gpsPtr;
typedef boost::shared_ptr< ::ublox_serial::gps const> gpsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_serial::gps_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_serial::gps_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_serial::gps_<ContainerAllocator1> & lhs, const ::ublox_serial::gps_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.gpsTime == rhs.gpsTime &&
    lhs.status == rhs.status &&
    lhs.lat == rhs.lat &&
    lhs.nsIndicator == rhs.nsIndicator &&
    lhs.lon == rhs.lon &&
    lhs.ewIndicator == rhs.ewIndicator &&
    lhs.spd == rhs.spd &&
    lhs.cog == rhs.cog &&
    lhs.date == rhs.date &&
    lhs.mv == rhs.mv &&
    lhs.mvEW == rhs.mvEW &&
    lhs.poseMode == rhs.poseMode &&
    lhs.navStatus == rhs.navStatus &&
    lhs.checksum == rhs.checksum &&
    lhs.CrLf == rhs.CrLf;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_serial::gps_<ContainerAllocator1> & lhs, const ::ublox_serial::gps_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_serial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ublox_serial::gps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_serial::gps_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_serial::gps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_serial::gps_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_serial::gps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_serial::gps_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_serial::gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e452e3e0fcf86e029a8550de79070f3";
  }

  static const char* value(const ::ublox_serial::gps_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e452e3e0fcf86e0ULL;
  static const uint64_t static_value2 = 0x29a8550de79070f3ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_serial::gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_serial/gps";
  }

  static const char* value(const ::ublox_serial::gps_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_serial::gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string header\n"
"string gpsTime\n"
"string status\n"
"string lat\n"
"string nsIndicator\n"
"string lon\n"
"string ewIndicator\n"
"string spd\n"
"string cog\n"
"string date\n"
"string mv\n"
"string mvEW\n"
"string poseMode\n"
"string navStatus\n"
"string checksum\n"
"string CrLf\n"
;
  }

  static const char* value(const ::ublox_serial::gps_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_serial::gps_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.gpsTime);
      stream.next(m.status);
      stream.next(m.lat);
      stream.next(m.nsIndicator);
      stream.next(m.lon);
      stream.next(m.ewIndicator);
      stream.next(m.spd);
      stream.next(m.cog);
      stream.next(m.date);
      stream.next(m.mv);
      stream.next(m.mvEW);
      stream.next(m.poseMode);
      stream.next(m.navStatus);
      stream.next(m.checksum);
      stream.next(m.CrLf);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gps_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_serial::gps_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_serial::gps_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.header);
    s << indent << "gpsTime: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.gpsTime);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
    s << indent << "lat: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.lat);
    s << indent << "nsIndicator: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.nsIndicator);
    s << indent << "lon: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.lon);
    s << indent << "ewIndicator: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ewIndicator);
    s << indent << "spd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.spd);
    s << indent << "cog: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cog);
    s << indent << "date: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.date);
    s << indent << "mv: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mv);
    s << indent << "mvEW: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mvEW);
    s << indent << "poseMode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.poseMode);
    s << indent << "navStatus: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.navStatus);
    s << indent << "checksum: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.checksum);
    s << indent << "CrLf: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.CrLf);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_SERIAL_MESSAGE_GPS_H
