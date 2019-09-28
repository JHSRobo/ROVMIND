#ifndef _ROS_vector_drive_thrusterPercents_h
#define _ROS_vector_drive_thrusterPercents_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vector_drive
{

  class thrusterPercents : public ros::Msg
  {
    public:
      typedef int32_t _t1_type;
      _t1_type t1;
      typedef int32_t _t2_type;
      _t2_type t2;
      typedef int32_t _t3_type;
      _t3_type t3;
      typedef int32_t _t4_type;
      _t4_type t4;

    thrusterPercents():
      t1(0),
      t2(0),
      t3(0),
      t4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_t1;
      u_t1.real = this->t1;
      *(outbuffer + offset + 0) = (u_t1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t1);
      union {
        int32_t real;
        uint32_t base;
      } u_t2;
      u_t2.real = this->t2;
      *(outbuffer + offset + 0) = (u_t2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t2);
      union {
        int32_t real;
        uint32_t base;
      } u_t3;
      u_t3.real = this->t3;
      *(outbuffer + offset + 0) = (u_t3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t3);
      union {
        int32_t real;
        uint32_t base;
      } u_t4;
      u_t4.real = this->t4;
      *(outbuffer + offset + 0) = (u_t4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_t1;
      u_t1.base = 0;
      u_t1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->t1 = u_t1.real;
      offset += sizeof(this->t1);
      union {
        int32_t real;
        uint32_t base;
      } u_t2;
      u_t2.base = 0;
      u_t2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->t2 = u_t2.real;
      offset += sizeof(this->t2);
      union {
        int32_t real;
        uint32_t base;
      } u_t3;
      u_t3.base = 0;
      u_t3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->t3 = u_t3.real;
      offset += sizeof(this->t3);
      union {
        int32_t real;
        uint32_t base;
      } u_t4;
      u_t4.base = 0;
      u_t4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->t4 = u_t4.real;
      offset += sizeof(this->t4);
     return offset;
    }

    const char * getType(){ return "vector_drive/thrusterPercents"; };
    const char * getMD5(){ return "a7d0e7700b4cbc9f7f9261f014f60680"; };

  };

}
#endif