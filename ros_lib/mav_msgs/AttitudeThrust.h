#ifndef _ROS_mav_msgs_AttitudeThrust_h
#define _ROS_mav_msgs_AttitudeThrust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

namespace mav_msgs
{

  class AttitudeThrust : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Quaternion _attitude_type;
      _attitude_type attitude;
      typedef geometry_msgs::Vector3 _thrust_type;
      _thrust_type thrust;

    AttitudeThrust():
      header(),
      attitude(),
      thrust()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->attitude.serialize(outbuffer + offset);
      offset += this->thrust.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->attitude.deserialize(inbuffer + offset);
      offset += this->thrust.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mav_msgs/AttitudeThrust"; };
    const char * getMD5(){ return "7cee443b02735e42bda0ad5910302718"; };

  };

}
#endif
