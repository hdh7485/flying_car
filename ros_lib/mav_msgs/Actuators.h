#ifndef _ROS_mav_msgs_Actuators_h
#define _ROS_mav_msgs_Actuators_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mav_msgs
{

  class Actuators : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t angles_length;
      typedef float _angles_type;
      _angles_type st_angles;
      _angles_type * angles;
      uint32_t angular_velocities_length;
      typedef float _angular_velocities_type;
      _angular_velocities_type st_angular_velocities;
      _angular_velocities_type * angular_velocities;
      uint32_t normalized_length;
      typedef float _normalized_type;
      _normalized_type st_normalized;
      _normalized_type * normalized;

    Actuators():
      header(),
      angles_length(0), angles(NULL),
      angular_velocities_length(0), angular_velocities(NULL),
      normalized_length(0), normalized(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->angles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles_length);
      for( uint32_t i = 0; i < angles_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angles[i]);
      }
      *(outbuffer + offset + 0) = (this->angular_velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angular_velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angular_velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angular_velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocities_length);
      for( uint32_t i = 0; i < angular_velocities_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_velocities[i]);
      }
      *(outbuffer + offset + 0) = (this->normalized_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->normalized_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->normalized_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->normalized_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->normalized_length);
      for( uint32_t i = 0; i < normalized_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->normalized[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t angles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angles_length);
      if(angles_lengthT > angles_length)
        this->angles = (float*)realloc(this->angles, angles_lengthT * sizeof(float));
      angles_length = angles_lengthT;
      for( uint32_t i = 0; i < angles_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_angles));
        memcpy( &(this->angles[i]), &(this->st_angles), sizeof(float));
      }
      uint32_t angular_velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angular_velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angular_velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angular_velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angular_velocities_length);
      if(angular_velocities_lengthT > angular_velocities_length)
        this->angular_velocities = (float*)realloc(this->angular_velocities, angular_velocities_lengthT * sizeof(float));
      angular_velocities_length = angular_velocities_lengthT;
      for( uint32_t i = 0; i < angular_velocities_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_angular_velocities));
        memcpy( &(this->angular_velocities[i]), &(this->st_angular_velocities), sizeof(float));
      }
      uint32_t normalized_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      normalized_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      normalized_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      normalized_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->normalized_length);
      if(normalized_lengthT > normalized_length)
        this->normalized = (float*)realloc(this->normalized, normalized_lengthT * sizeof(float));
      normalized_length = normalized_lengthT;
      for( uint32_t i = 0; i < normalized_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_normalized));
        memcpy( &(this->normalized[i]), &(this->st_normalized), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "mav_msgs/Actuators"; };
    const char * getMD5(){ return "25741daf38ed25442e3a66a855ee8d9c"; };

  };

}
#endif
