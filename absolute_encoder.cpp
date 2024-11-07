#include "absolute_encoder.h"
int absolute_encoder::rawCount = 0;
void absolute_encoder::read_rawCount()
{
  if (PIND & (1 << PD3)) rawCount++;
  else rawCount--;
}
void absolute_encoder::init()
{
  attachInterrupt ( 0, read_rawCount, FALLING);
  DDRD &= (0<<2) | (0<<3);
  PORTD &= (1<<2) | (1<<3);
}
float absolute_encoder::voltage_transfer()
{
  return (abs(rawCount) * (2 * PI / 2000));
}
float absolute_encoder::angle()
{
  return ((rawCount) * (2 * PI / 2000));
}