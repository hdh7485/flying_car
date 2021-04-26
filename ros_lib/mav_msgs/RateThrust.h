#ifndef _ROS_mav_msgs_RateThrust_h
#define _ROS_mav_msgs_RateThrust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace mav_msgs
{

  class RateThrust : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _angular_rates_type;
      _angular_rates_type angular_rates;
      typedef geometry_msgs::Vector3 _thrust_type;
      _thrust_type thrust;

    RateThrust():
      header(),
      angular_rates(),
      thrust()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->angular_rates.serialize(outbuffer + offset);
      offset += this->thrust.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->angular_rates.deserialize(inbuffer + offset);
      offset += this->thrust.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mav_msgs/RateThrust"; };
    const char * getMD5(){ return "119c5bf883bef42350d52ce5a7927c7c"; };

  };

}
#endif
