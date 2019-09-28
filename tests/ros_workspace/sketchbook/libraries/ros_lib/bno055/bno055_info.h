#ifndef _ROS_bno055_bno055_info_h
#define _ROS_bno055_bno055_info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace bno055
{

  class bno055_info : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _tempC_type;
      _tempC_type tempC;
      typedef uint8_t _accelCalibration_type;
      _accelCalibration_type accelCalibration;
      typedef uint8_t _gyroCalibration_type;
      _gyroCalibration_type gyroCalibration;
      typedef uint8_t _magnoCalibration_type;
      _magnoCalibration_type magnoCalibration;
      typedef uint8_t _sysCalibration_type;
      _sysCalibration_type sysCalibration;

    bno055_info():
      header(),
      tempC(0),
      accelCalibration(0),
      gyroCalibration(0),
      magnoCalibration(0),
      sysCalibration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->tempC);
      *(outbuffer + offset + 0) = (this->accelCalibration >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accelCalibration);
      *(outbuffer + offset + 0) = (this->gyroCalibration >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gyroCalibration);
      *(outbuffer + offset + 0) = (this->magnoCalibration >> (8 * 0)) & 0xFF;
      offset += sizeof(this->magnoCalibration);
      *(outbuffer + offset + 0) = (this->sysCalibration >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sysCalibration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tempC));
      this->accelCalibration =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->accelCalibration);
      this->gyroCalibration =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gyroCalibration);
      this->magnoCalibration =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->magnoCalibration);
      this->sysCalibration =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sysCalibration);
     return offset;
    }

    const char * getType(){ return "bno055/bno055_info"; };
    const char * getMD5(){ return "c1818d2af59dc79e8470c9bf920f48a8"; };

  };

}
#endif