#ifndef _ROS_mav_msgs_FilteredSensorData_h
#define _ROS_mav_msgs_FilteredSensorData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace mav_msgs
{

  class FilteredSensorData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _accelerometer_type;
      _accelerometer_type accelerometer;
      typedef geometry_msgs::Vector3 _gyroscope_type;
      _gyroscope_type gyroscope;
      typedef geometry_msgs::Vector3 _magnetometer_type;
      _magnetometer_type magnetometer;
      typedef float _barometer_type;
      _barometer_type barometer;

    FilteredSensorData():
      header(),
      accelerometer(),
      gyroscope(),
      magnetometer(),
      barometer(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->accelerometer.serialize(outbuffer + offset);
      offset += this->gyroscope.serialize(outbuffer + offset);
      offset += this->magnetometer.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->barometer);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->accelerometer.deserialize(inbuffer + offset);
      offset += this->gyroscope.deserialize(inbuffer + offset);
      offset += this->magnetometer.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->barometer));
     return offset;
    }

    const char * getType(){ return "mav_msgs/FilteredSensorData"; };
    const char * getMD5(){ return "a9b51fae1f4ed3a8a0522b4ffe61659f"; };

  };

}
#endif
