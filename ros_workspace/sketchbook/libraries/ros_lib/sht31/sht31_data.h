#ifndef _ROS_sht31_sht31_data_h
#define _ROS_sht31_sht31_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sht31
{

  class sht31_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _tempC_type;
      _tempC_type tempC;
      typedef float _tempF_type;
      _tempF_type tempF;
      typedef float _humidity_type;
      _humidity_type humidity;

    sht31_data():
      header(),
      tempC(0),
      tempF(0),
      humidity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempC);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempF);
      offset += serializeAvrFloat64(outbuffer + offset, this->humidity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempC));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempF));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->humidity));
     return offset;
    }

    const char * getType(){ return "sht31/sht31_data"; };
    const char * getMD5(){ return "e4499b266523b5e72cb5c84b02109a13"; };

  };

}
#endif