#ifndef _ROS_mav_msgs_RollPitchYawrateThrust_h
#define _ROS_mav_msgs_RollPitchYawrateThrust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace mav_msgs
{

  class RollPitchYawrateThrust : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_rate_type;
      _yaw_rate_type yaw_rate;
      typedef geometry_msgs::Vector3 _thrust_type;
      _thrust_type thrust;

    RollPitchYawrateThrust():
      header(),
      roll(0),
      pitch(0),
      yaw_rate(0),
      thrust()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_rate);
      offset += this->thrust.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw_rate));
      offset += this->thrust.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mav_msgs/RollPitchYawrateThrust"; };
    const char * getMD5(){ return "10a56a30857affade0889a3662fc2bc9"; };

  };

}
#endif
