#ifndef _ROS_tcu_board_msgs_tcu_leds_h
#define _ROS_tcu_board_msgs_tcu_leds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tcu_board_msgs
{

  class tcu_leds : public ros::Msg
  {
    public:
      typedef int8_t _led_routine_type;
      _led_routine_type led_routine;

    tcu_leds():
      led_routine(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_led_routine;
      u_led_routine.real = this->led_routine;
      *(outbuffer + offset + 0) = (u_led_routine.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led_routine);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_led_routine;
      u_led_routine.base = 0;
      u_led_routine.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->led_routine = u_led_routine.real;
      offset += sizeof(this->led_routine);
     return offset;
    }

    const char * getType(){ return "tcu_board_msgs/tcu_leds"; };
    const char * getMD5(){ return "94852ec6bb25041db9fee4bb7cdd9299"; };

  };

}
#endif