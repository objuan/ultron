#ifndef _ROS_omni_shot_trajectory_poly_h
#define _ROS_omni_shot_trajectory_poly_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace omni_shot
{

  class trajectory_poly : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t linear_length;
      float st_linear;
      float * linear;
      uint8_t angular_length;
      float st_angular;
      float * angular;
      float time;
      float theta;

    trajectory_poly():
      header(),
      linear_length(0), linear(NULL),
      angular_length(0), angular(NULL),
      time(0),
      theta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = linear_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < linear_length; i++){
      union {
        float real;
        uint32_t base;
      } u_lineari;
      u_lineari.real = this->linear[i];
      *(outbuffer + offset + 0) = (u_lineari.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lineari.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lineari.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lineari.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear[i]);
      }
      *(outbuffer + offset++) = angular_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < angular_length; i++){
      union {
        float real;
        uint32_t base;
      } u_angulari;
      u_angulari.real = this->angular[i];
      *(outbuffer + offset + 0) = (u_angulari.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angulari.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angulari.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angulari.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t linear_lengthT = *(inbuffer + offset++);
      if(linear_lengthT > linear_length)
        this->linear = (float*)realloc(this->linear, linear_lengthT * sizeof(float));
      offset += 3;
      linear_length = linear_lengthT;
      for( uint8_t i = 0; i < linear_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_linear;
      u_st_linear.base = 0;
      u_st_linear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_linear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_linear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_linear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_linear = u_st_linear.real;
      offset += sizeof(this->st_linear);
        memcpy( &(this->linear[i]), &(this->st_linear), sizeof(float));
      }
      uint8_t angular_lengthT = *(inbuffer + offset++);
      if(angular_lengthT > angular_length)
        this->angular = (float*)realloc(this->angular, angular_lengthT * sizeof(float));
      offset += 3;
      angular_length = angular_lengthT;
      for( uint8_t i = 0; i < angular_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_angular;
      u_st_angular.base = 0;
      u_st_angular.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angular.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_angular.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_angular.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_angular = u_st_angular.real;
      offset += sizeof(this->st_angular);
        memcpy( &(this->angular[i]), &(this->st_angular), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time = u_time.real;
      offset += sizeof(this->time);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
     return offset;
    }

    const char * getType(){ return "omni_shot/trajectory_poly"; };
    const char * getMD5(){ return "b6dd9f851a0f6c4cd34afea981568b2f"; };

  };

}
#endif