#ifndef _ROS_omni_shot_way_points_h
#define _ROS_omni_shot_way_points_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "omni_shot/way_point.h"

namespace omni_shot
{

  class way_points : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t path_length;
      omni_shot::way_point st_path;
      omni_shot::way_point * path;
      uint8_t sample;

    way_points():
      header(),
      path_length(0), path(NULL),
      sample(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = path_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < path_length; i++){
      offset += this->path[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->sample >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sample);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t path_lengthT = *(inbuffer + offset++);
      if(path_lengthT > path_length)
        this->path = (omni_shot::way_point*)realloc(this->path, path_lengthT * sizeof(omni_shot::way_point));
      offset += 3;
      path_length = path_lengthT;
      for( uint8_t i = 0; i < path_length; i++){
      offset += this->st_path.deserialize(inbuffer + offset);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(omni_shot::way_point));
      }
      this->sample =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sample);
     return offset;
    }

    const char * getType(){ return "omni_shot/way_points"; };
    const char * getMD5(){ return "1847c8a7cc942338a5e4611b252b7e5a"; };

  };

}
#endif