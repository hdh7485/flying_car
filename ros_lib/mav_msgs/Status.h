#ifndef _ROS_mav_msgs_Status_h
#define _ROS_mav_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mav_msgs
{

  class Status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _vehicle_name_type;
      _vehicle_name_type vehicle_name;
      typedef const char* _vehicle_type_type;
      _vehicle_type_type vehicle_type;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef const char* _rc_command_mode_type;
      _rc_command_mode_type rc_command_mode;
      typedef bool _command_interface_enabled_type;
      _command_interface_enabled_type command_interface_enabled;
      typedef float _flight_time_type;
      _flight_time_type flight_time;
      typedef float _system_uptime_type;
      _system_uptime_type system_uptime;
      typedef float _cpu_load_type;
      _cpu_load_type cpu_load;
      typedef const char* _motor_status_type;
      _motor_status_type motor_status;
      typedef bool _in_air_type;
      _in_air_type in_air;
      typedef const char* _gps_status_type;
      _gps_status_type gps_status;
      typedef int32_t _gps_num_satellites_type;
      _gps_num_satellites_type gps_num_satellites;
      enum { RC_COMMAND_ATTITUDE = "attitude_thrust" };
      enum { RC_COMMAND_ATTITUDE_HEIGHT = "attitude_height" };
      enum { RC_COMMAND_POSITION = "position" };
      enum { MOTOR_STATUS_RUNNING = "running" };
      enum { MOTOR_STATUS_STOPPED = "stopped" };
      enum { MOTOR_STATUS_STARTING = "starting" };
      enum { MOTOR_STATUS_STOPPING = "stopping" };
      enum { GPS_STATUS_LOCK = "lock" };
      enum { GPS_STATUS_NO_LOCK = "no_lock" };

    Status():
      header(),
      vehicle_name(""),
      vehicle_type(""),
      battery_voltage(0),
      rc_command_mode(""),
      command_interface_enabled(0),
      flight_time(0),
      system_uptime(0),
      cpu_load(0),
      motor_status(""),
      in_air(0),
      gps_status(""),
      gps_num_satellites(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_vehicle_name = strlen(this->vehicle_name);
      varToArr(outbuffer + offset, length_vehicle_name);
      offset += 4;
      memcpy(outbuffer + offset, this->vehicle_name, length_vehicle_name);
      offset += length_vehicle_name;
      uint32_t length_vehicle_type = strlen(this->vehicle_type);
      varToArr(outbuffer + offset, length_vehicle_type);
      offset += 4;
      memcpy(outbuffer + offset, this->vehicle_type, length_vehicle_type);
      offset += length_vehicle_type;
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      uint32_t length_rc_command_mode = strlen(this->rc_command_mode);
      varToArr(outbuffer + offset, length_rc_command_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->rc_command_mode, length_rc_command_mode);
      offset += length_rc_command_mode;
      union {
        bool real;
        uint8_t base;
      } u_command_interface_enabled;
      u_command_interface_enabled.real = this->command_interface_enabled;
      *(outbuffer + offset + 0) = (u_command_interface_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command_interface_enabled);
      union {
        float real;
        uint32_t base;
      } u_flight_time;
      u_flight_time.real = this->flight_time;
      *(outbuffer + offset + 0) = (u_flight_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flight_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flight_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flight_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flight_time);
      union {
        float real;
        uint32_t base;
      } u_system_uptime;
      u_system_uptime.real = this->system_uptime;
      *(outbuffer + offset + 0) = (u_system_uptime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_system_uptime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_system_uptime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_system_uptime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->system_uptime);
      union {
        float real;
        uint32_t base;
      } u_cpu_load;
      u_cpu_load.real = this->cpu_load;
      *(outbuffer + offset + 0) = (u_cpu_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load);
      uint32_t length_motor_status = strlen(this->motor_status);
      varToArr(outbuffer + offset, length_motor_status);
      offset += 4;
      memcpy(outbuffer + offset, this->motor_status, length_motor_status);
      offset += length_motor_status;
      union {
        bool real;
        uint8_t base;
      } u_in_air;
      u_in_air.real = this->in_air;
      *(outbuffer + offset + 0) = (u_in_air.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_air);
      uint32_t length_gps_status = strlen(this->gps_status);
      varToArr(outbuffer + offset, length_gps_status);
      offset += 4;
      memcpy(outbuffer + offset, this->gps_status, length_gps_status);
      offset += length_gps_status;
      union {
        int32_t real;
        uint32_t base;
      } u_gps_num_satellites;
      u_gps_num_satellites.real = this->gps_num_satellites;
      *(outbuffer + offset + 0) = (u_gps_num_satellites.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gps_num_satellites.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gps_num_satellites.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gps_num_satellites.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_num_satellites);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_vehicle_name;
      arrToVar(length_vehicle_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_vehicle_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_vehicle_name-1]=0;
      this->vehicle_name = (char *)(inbuffer + offset-1);
      offset += length_vehicle_name;
      uint32_t length_vehicle_type;
      arrToVar(length_vehicle_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_vehicle_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_vehicle_type-1]=0;
      this->vehicle_type = (char *)(inbuffer + offset-1);
      offset += length_vehicle_type;
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      uint32_t length_rc_command_mode;
      arrToVar(length_rc_command_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_rc_command_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_rc_command_mode-1]=0;
      this->rc_command_mode = (char *)(inbuffer + offset-1);
      offset += length_rc_command_mode;
      union {
        bool real;
        uint8_t base;
      } u_command_interface_enabled;
      u_command_interface_enabled.base = 0;
      u_command_interface_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->command_interface_enabled = u_command_interface_enabled.real;
      offset += sizeof(this->command_interface_enabled);
      union {
        float real;
        uint32_t base;
      } u_flight_time;
      u_flight_time.base = 0;
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->flight_time = u_flight_time.real;
      offset += sizeof(this->flight_time);
      union {
        float real;
        uint32_t base;
      } u_system_uptime;
      u_system_uptime.base = 0;
      u_system_uptime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_system_uptime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_system_uptime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_system_uptime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->system_uptime = u_system_uptime.real;
      offset += sizeof(this->system_uptime);
      union {
        float real;
        uint32_t base;
      } u_cpu_load;
      u_cpu_load.base = 0;
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_load = u_cpu_load.real;
      offset += sizeof(this->cpu_load);
      uint32_t length_motor_status;
      arrToVar(length_motor_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_status-1]=0;
      this->motor_status = (char *)(inbuffer + offset-1);
      offset += length_motor_status;
      union {
        bool real;
        uint8_t base;
      } u_in_air;
      u_in_air.base = 0;
      u_in_air.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_air = u_in_air.real;
      offset += sizeof(this->in_air);
      uint32_t length_gps_status;
      arrToVar(length_gps_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_gps_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_gps_status-1]=0;
      this->gps_status = (char *)(inbuffer + offset-1);
      offset += length_gps_status;
      union {
        int32_t real;
        uint32_t base;
      } u_gps_num_satellites;
      u_gps_num_satellites.base = 0;
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gps_num_satellites = u_gps_num_satellites.real;
      offset += sizeof(this->gps_num_satellites);
     return offset;
    }

    const char * getType(){ return "mav_msgs/Status"; };
    const char * getMD5(){ return "e191265664a5f7c1871338dc13be0958"; };

  };

}
#endif
