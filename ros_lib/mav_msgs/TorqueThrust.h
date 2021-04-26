#ifndef _ROS_mav_msgs_TorqueThrust_h
#define _ROS_mav_msgs_TorqueThrust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace mav_msgs
{

  class TorqueThrust : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _torque_type;
      _torque_type torque;
      typedef geometry_msgs::Vector3 _thrust_type;
      _thrust_type thrust;

    TorqueThrust():
      header(),
      torque(),
      thrust()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->torque.serialize(outbuffer + offset);
      offset += this->thrust.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->torque.deserialize(inbuffer + offset);
      offset += this->thrust.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mav_msgs/TorqueThrust"; };
    const char * getMD5(){ return "81293743ae52683b61e39c21bc0b30db"; };

  };

}
#endif
