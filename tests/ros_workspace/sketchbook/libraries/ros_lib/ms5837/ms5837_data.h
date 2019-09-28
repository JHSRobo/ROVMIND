#ifndef _ROS_ms5837_ms5837_data_h
#define _ROS_ms5837_ms5837_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ms5837
{

  class ms5837_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _tempC_type;
      _tempC_type tempC;
      typedef float _tempF_type;
      _tempF_type tempF;
      typedef float _depth_type;
      _depth_type depth;
      typedef float _altitudeM_type;
      _altitudeM_type altitudeM;

    ms5837_data():
      header(),
      tempC(0),
      tempF(0),
      depth(0),
      altitudeM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempC);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempF);
      offset += serializeAvrFloat64(outbuffer + offset, this->depth);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitudeM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempC));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempF));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->depth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitudeM));
     return offset;
    }

    const char * getType(){ return "ms5837/ms5837_data"; };
    const char * getMD5(){ return "eca2bdcabad4ac8096363838d8496716"; };

  };

}
#endif