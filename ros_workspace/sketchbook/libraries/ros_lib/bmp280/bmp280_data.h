#ifndef _ROS_bmp280_bmp280_data_h
#define _ROS_bmp280_bmp280_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace bmp280
{

  class bmp280_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _tempC_type;
      _tempC_type tempC;
      typedef float _pressureP_type;
      _pressureP_type pressureP;
      typedef float _pressureA_type;
      _pressureA_type pressureA;
      typedef float _altitudeM_type;
      _altitudeM_type altitudeM;

    bmp280_data():
      header(),
      tempC(0),
      pressureP(0),
      pressureA(0),
      altitudeM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempC);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressureP);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressureA);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitudeM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempC));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressureP));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressureA));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitudeM));
     return offset;
    }

    const char * getType(){ return "bmp280/bmp280_data"; };
    const char * getMD5(){ return "c5e4218ec509085711881fab8b571873"; };

  };

}
#endif