#include <PID_v1.h>

#include "DCMotorServo.h"

#define dcmoto_encoder_tick_by_turn 6400

DCMotorServo::DCMotorServo(uint8_t pin_dir_1, uint8_t pin_dir_2, uint8_t pin_PWM_output, uint8_t pin_encode1, uint8_t pin_encode2)
{
  _pin_PWM_output = pin_PWM_output;
  _pin_dir_1 = pin_dir_1;
 // _pin_dir_2 = pin_dir_2;

  //Direction and PWM output
  pinMode(_pin_dir_1, OUTPUT);
  //pinMode(_pin_dir_2, OUTPUT);
  pinMode(_pin_PWM_output, OUTPUT);

  _position = new Encoder(pin_encode1, pin_encode2);
  _PWM_output = 0;  
  _pwm_skip = 50;
  _position_accuracy = 30;
  turnCount=0;
  
  _PID_input = _position->read();
  _PID_output = 0;
  _PID_setpoint = _PID_input;
  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint,.1,.2,.1, DIRECT);

  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip-255, 255-_pwm_skip);

  //turn the PID on
  myPID->SetMode(AUTOMATIC);
}

void DCMotorServo::setCurrentPosition(int new_position)
{
  Serial.print("setCurrentPosition");
  Serial.println(new_position);
  
  _position->write(new_position);
  _PID_input = _position->read();
}

void DCMotorServo::setAccuracy(unsigned int range)
{
   Serial.print("setAccuracy");
  Serial.println(range);
  
  _position_accuracy = range;
}

bool DCMotorServo::setPWMSkip(uint8_t range)
{
  if ( 0 <= range && range < 255) {
    _pwm_skip = range;
    return 1;
  }
  else
    return 0;
}

//void DCMotorServo::SetPIDTunings(double Kp, double Ki, double Kd)
//{
//	myPID->SetTunings(Kp, Ki, Kd);
//}

bool DCMotorServo::finished()
{
  if (abs(_PID_setpoint - _PID_input) < _position_accuracy && _PWM_output == 0)
  {
      Serial.println("finished");
      return 1;
  }
  return 0;
 
}

void DCMotorServo::move(int new_rela_position)
{
   
  //use _PID_setpoint so that we don't introduce errors of _position_accuracy
  _PID_setpoint = _PID_setpoint + new_rela_position;

   Serial.print("SET POS");
    Serial.print(new_rela_position);
     Serial.print("->");
      Serial.println(_PID_setpoint);
}

void DCMotorServo::moveTo(int new_position)
{
  Serial.print("moveTo");
  Serial.println(new_position);
  
  _PID_setpoint = new_position;
}

int DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

int DCMotorServo::getActualPosition()
{
  return _position->read();
}

int lastTime=0;

// lanciata ad ogni loop
void DCMotorServo::run() {
  _PID_input = _position->read();

  // DEBUG
  turnCount = _PID_input / dcmoto_encoder_tick_by_turn;

  int t = millis() / 1000;
  if (t != lastTime) // ogni secondo
  {
    lastTime=t;
   Serial.print("ENC:");
    Serial.print(t);
    Serial.print(" - ");
   Serial.println(_PID_input);
   Serial.print(" - ");
   Serial.println(turnCount);
  }

  //_PWM_output = 153; // 60 RPM -> 1 giro al secondo
 // _PWM_output = 141; // 60 RPM -> 1 giro al secondo
  //_PWM_output = 153/2; // 60 RPM -> 1/2 giro al secondo

   
  myPID->Compute();

  _PWM_output = abs(_PID_output) + _pwm_skip;
  if (abs(_PID_setpoint - _PID_input) < _position_accuracy)
  {
    myPID->SetMode(MANUAL);
    _PID_output = 0;
    _PWM_output = 0;
  }
  else
  {
    myPID->SetMode(AUTOMATIC);
  }

 // Serial.print("OUT");
//  Serial.println(_PWM_output);
  
  //_PWM_output=0;
  _pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
}

void DCMotorServo::stop() {
   Serial.println("STOP");
  myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  analogWrite(_pin_PWM_output, _PWM_output);
}

void DCMotorServo::_pick_direction() {
  if (_PID_output < 0)
  {
    digitalWrite(_pin_dir_1, HIGH);
  //  digitalWrite(_pin_dir_2, HIGH);
  }
  else
  {

    digitalWrite(_pin_dir_1, LOW);
   // digitalWrite(_pin_dir_2, LOW);
  }
  
}

