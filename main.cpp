#include<SimpleFOC.h>
#include "absolute_encoder.h"
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10); // <gọi class MagneticSensorSPI>
BLDCMotor motor = BLDCMotor(11);                              // <gọi class BLDCMotor>
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);           // <gọi class BLDCDriver3PWM>
Commander command = Commander(Serial);                        // <gọi class Commander>
absolute_encoder S;                                           // <gọi class absolute_encoder>

float target_angle = 1; // <mục tiêu góc mong muốn>

//////////////////////////////// hàm gọi command ////////////////////////////////

void change_voltage_limit(char* cmd)  // trỏ giá trị từ serial tới velocity_limit(PID)
{
  command.scalar(&motor.PID_velocity.limit, cmd);
}
//////////////////////////////// hàm gọi command ////////////////////////////////

void setup()
{
  Serial.begin(115200);
  S.init();
  //////////////////////////////// Bật debug ////////////////////////////////
  SimpleFOCDebug::enable(&Serial);
//////////////////////////////// Bật debug ////////////////////////////////

//////////////////////////////// động cơ-encoder-driver ////////////////////////////////
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
//////////////////////////////// động cơ-encoder-driver ////////////////////////////////

//////////////////////////////// loại điều khiển động cơ ////////////////////////////////
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::angle;
//////////////////////////////// loại điều khiển động cơ ////////////////////////////////

//////////////////////////////// PID - điện áp - bộ lọc ////////////////////////////////
  motor.PID_current_q.P=0.1;
  motor.PID_current_q.I=0.03;
  motor.PID_current_q.D=0.001;

  motor.PID_current_d.P=0.1;
  motor.PID_current_d.P=0.01;
  motor.PID_current_d.P=0.007;

  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 0.06;
  motor.PID_velocity.D = 0.005;

  motor.P_angle.P = 10;
  motor.P_angle.I = 0.7;
  motor.P_angle.D = 0.05;

  motor.LPF_velocity.Tf = 0.01;
  motor.LPF_angle.Tf = 0.01;
  
  motor.voltage_limit=3;
  motor.velocity_limit = 10;
//////////////////////////////// PID - điện áp - bộ lọc ////////////////////////////////
  
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

//////////////////////////////// Gọi command serial ////////////////////////////////
  command.add('S', change_voltage_limit, "motor.PID_velocity.limit");  // S + float
//////////////////////////////// Gọi command serial ////////////////////////////////
  _delay(1000);
}

void loop()
{
//////////////////////////////// Vòng loop điều khiển motor ////////////////////////////////
  motor.loopFOC();
  command.run();
  motor.move(S.angle());
  //float voltage = S.rawCount*(2*PI/2000);
//////////////////////////////// Vòng loop điều khiển motor ////////////////////////////////

//////////////////////////////// Kiểm tra dòng ////////////////////////////////
  // float Pha_a = motor.voltage.q * sin(motor.electrical_angle);
  // float Pha_b = motor.voltage.q * sin(motor.electrical_angle - _2PI/3);
  // float Pha_c = motor.voltage.q * sin(motor.electrical_angle + _2PI/3);
  float b=sin((-1)*S.angle());
  float a=sin(sensor.getAngle());
  Serial.print(">AS5048A: ");Serial.println(a);   // <Vị trí động cơ_đọc bằng AS5048A>
  Serial.print(">C6B2_CWZ6C: ");Serial.println(b);  // <Vị trí encoder_đọc bằng C6B2_CWZ6C>
  //Serial.print(">input_voltage: ");Serial.println(S.voltage_transfer());
  Serial.print(">output_voltage: ");Serial.println(abs(motor.voltage.q));
//////////////////////////////// Kiểm tra dòng ////////////////////////////////
  
}