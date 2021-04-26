#ifndef _ROS_mav_msgs_GpsWaypoint_h
#define _ROS_mav_msgs_GpsWaypoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mav_msgs
{

  class GpsWaypoint : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _heading_type;
      _heading_type heading;
      typedef float _maxSpeed_type;
      _maxSpeed_type maxSpeed;
      typedef float _maxAcc_type;
      _maxAcc_type maxAcc;

    GpsWaypoint():
      header(),
      latitude(0),
      longitude(0),
      altitude(0),
      heading(0),
      maxSpeed(0),
      maxAcc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading);
      offset += serializeAvrFloat64(outbuffer + offset, this->maxSpeed);
      offset += serializeAvrFloat64(outbuffer + offset, this->maxAcc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxSpeed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxAcc));
     return offset;
    }

    const char * getType(){ return "mav_msgs/GpsWaypoint"; };
    const char * getMD5(){ return "61c3751c96f3b418f93879727f9a070a"; };

  };

}
#endif
