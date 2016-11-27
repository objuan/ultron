#ifndef _ROS_omni_shot_motors_h
#define _ROS_omni_shot_motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace omni_shot
{

  class motors : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float m1;
      float m2;
      float m3;
      float m4;

    motors():
      header(),
      m1(0),
      m2(0),
      m3(0),
      m4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_m1;
      u_m1.real = this->m1;
      *(outbuffer + offset + 0) = (u_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m1);
      union {
        float real;
        uint32_t base;
      } u_m2;
      u_m2.real = this->m2;
      *(outbuffer + offset + 0) = (u_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m2);
      union {
        float real;
        uint32_t base;
      } u_m3;
      u_m3.real = this->m3;
      *(outbuffer + offset + 0) = (u_m3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m3);
      union {
        float real;
        uint32_t base;
      } u_m4;
      u_m4.real = this->m4;
      *(outbuffer + offset + 0) = (u_m4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_m1;
      u_m1.base = 0;
      u_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m1 = u_m1.real;
      offset += sizeof(this->m1);
      union {
        float real;
        uint32_t base;
      } u_m2;
      u_m2.base = 0;
      u_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m2 = u_m2.real;
      offset += sizeof(this->m2);
      union {
        float real;
        uint32_t base;
      } u_m3;
      u_m3.base = 0;
      u_m3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m3 = u_m3.real;
      offset += sizeof(this->m3);
      union {
        float real;
        uint32_t base;
      } u_m4;
      u_m4.base = 0;
      u_m4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m4 = u_m4.real;
      offset += sizeof(this->m4);
     return offset;
    }

    const char * getType(){ return "omni_shot/motors"; };
    const char * getMD5(){ return "492dbdb298ee2a5efd67096e7383ada3"; };

  };

}
#endif