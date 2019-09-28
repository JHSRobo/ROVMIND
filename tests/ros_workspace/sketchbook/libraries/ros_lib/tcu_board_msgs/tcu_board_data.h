#ifndef _ROS_tcu_board_msgs_tcu_board_data_h
#define _ROS_tcu_board_msgs_tcu_board_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tcu_board_msgs
{

  class tcu_board_data : public ros::Msg
  {
    public:
      typedef float _tempC_type;
      _tempC_type tempC;
      typedef float _humidity_type;
      _humidity_type humidity;
      typedef float _currentMA_type;
      _currentMA_type currentMA;
      typedef float _voltage_type;
      _voltage_type voltage;

    tcu_board_data():
      tempC(0),
      humidity(0),
      currentMA(0),
      voltage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_tempC;
      u_tempC.real = this->tempC;
      *(outbuffer + offset + 0) = (u_tempC.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tempC.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tempC.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tempC.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tempC);
      union {
        float real;
        uint32_t base;
      } u_humidity;
      u_humidity.real = this->humidity;
      *(outbuffer + offset + 0) = (u_humidity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_humidity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_humidity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_humidity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->humidity);
      union {
        float real;
        uint32_t base;
      } u_currentMA;
      u_currentMA.real = this->currentMA;
      *(outbuffer + offset + 0) = (u_currentMA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentMA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentMA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentMA.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentMA);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_tempC;
      u_tempC.base = 0;
      u_tempC.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tempC.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tempC.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tempC.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tempC = u_tempC.real;
      offset += sizeof(this->tempC);
      union {
        float real;
        uint32_t base;
      } u_humidity;
      u_humidity.base = 0;
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->humidity = u_humidity.real;
      offset += sizeof(this->humidity);
      union {
        float real;
        uint32_t base;
      } u_currentMA;
      u_currentMA.base = 0;
      u_currentMA.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentMA.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentMA.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentMA.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentMA = u_currentMA.real;
      offset += sizeof(this->currentMA);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
     return offset;
    }

    const char * getType(){ return "tcu_board_msgs/tcu_board_data"; };
    const char * getMD5(){ return "bdb48909d88ffd8a6589dd5f63593bf1"; };

  };

}
#endif