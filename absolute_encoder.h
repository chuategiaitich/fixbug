#ifndef ABSOLUTE_ENCODER_H
#define ABSOLUTE_ENCODER_H
#include "Arduino.h"
class absolute_encoder
{
  public:
    static int rawCount;
    static void read_rawCount();
    static void init();
    float voltage_transfer();
    float angle();
  private:
};

#endif
