#include <PID_v1.h>

#include "DCMotorServo.h"

#define dcmoto_encoder_tick_by_turn 6400.0
double kp=2,ki=0.5,kd=2;
//double kp=10,ki=0,kd=0;
//double kp=200,ki=10,kd=0;
double ScaleFactor = 1;

DCMotorServo::DCMotorServo(uint8_t pin_dir_1,  uint8_t pin_PWM_output, uint8_t pin_encode1, uint8_t pin_encode2)
{
  _pin_PWM_output = pin_PWM_output;
  _pin_dir_1 = pin_dir_1;
 // _pin_dir_2 = pin_dir_2;

  //Direction and PWM output
  pinMode(_pin_dir_1, OUTPUT);
  pinMode(_pin_PWM_output, OUTPUT);

  _position = new Encoder(pin_encode1, pin_encode2);
  _PWM_output = 0;  
  _pwm_skip = 50;
  _position_accuracy = 30;

  lastTime=0;
  targetRPM = 0;
  currentRPM=0;
  lastPosition=0;
  deltaPos=0;

  Integral=0;
  Last=0;
  
  _PID_input = _position->read();
  _PID_output = 0;

  // Kp, Ki, Kd:
//  _PID_setpoint = _PID_input;
  //myPID = new PID(&currentRPM, &_PID_output, &targetRPM,.1,.2,.1, DIRECT);
  myPID = new PID(&currentRPM, &_PID_output, &targetRPM,kp,ki,kd, DIRECT);

  myPID->SetSampleTime(200);
 // myPID->SetOutputLimits(_pwm_skip-255, 255-_pwm_skip);
  myPID->SetOutputLimits(0,255);

  //turn the PID on
  myPID->SetMode(AUTOMATIC);
}
/*
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
*/
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
   return 1;
//  if (abs(_PID_setpoint - _PID_input) < _position_accuracy && _PWM_output == 0)
//  {
//      Serial.println("finished");
//      return 1;
//  }
//  return 0;
 
}

void DCMotorServo::setSpeed(double rpm)
{
  targetRPM = rpm;

   Serial.print("SET SPEED:");
   Serial.println(rpm);
}
//
//void DCMotorServo::moveTo(int new_position)
//{
//  Serial.print("moveTo");
//  Serial.println(new_position);
//  
//  _PID_setpoint = new_position;
//}

//int DCMotorServo::getRequestedPosition()
//{
//  return _PID_setpoint;
//}

double DCMotorServo::getActualSpeed()
{
  return currentRPM;
}

double IntThresh = 255;

double DCMotorServo::calcolaPID()
{
     double Drive;
    //-------------------- Calculates the PID drive value  --------------------    
      double Actual = currentRPM; 
      double Error = targetRPM - Actual; 
      if (abs(Error) < IntThresh){          // prevent integral 'windup' 
         Integral = Integral + Error;       // accumulate the error integral 
      } 
      else { 
         Integral=0;                         // zero it if out of bounds    
      }    

      // Serial.println(Error);
       
      P = Error*kp;                         // calc proportional term 
      I = Integral*ki;                      // integral term 
      D = (Last-Actual)*kd;                 // derivative term 
      Drive = P + I + D;                    // Total drive = P+I+D 

       Serial.println(Drive);
       
      Drive = Drive*ScaleFactor;            // scale Drive to be in the range 0-255    
//      if (Drive < 0){                       // Check which direction to go.    
//         digitalWrite (Direction,LOW);      // change direction as needed      
//      } 
//      else {                                //   depending on the sign of Error 
//         digitalWrite (Direction,HIGH); 
//      }   
      if (abs(Drive)>255) { 
         Drive=255; 
      } 
      //analogWrite (Motor,Drive);            // send PWM command to motor board 
      Last = Actual;                        // save current value for next time 
      return Drive;
}

// lanciata ad ogni loop
void DCMotorServo::run() {

  double _time = (double)millis() / 1000;
  if (_time - lastTime > 1.0) // ogni secondo
  {
    _PID_input = _position->read();

    double deltaTime = _time - lastTime;

    deltaPos = _PID_input - lastPosition;
  
    // giri al secondo per 60
    currentRPM = 60.0 * (abs(deltaPos) / dcmoto_encoder_tick_by_turn) / deltaTime;

    Serial.print("READ: ");
    Serial.print(_time);
    Serial.print(" - ");
    Serial.print(deltaTime);
    Serial.print(" - ");
    Serial.println(currentRPM);

    lastTime=_time;
    lastPosition = _PID_input;
  }

  //_PWM_output = 153; // 60 RPM -> 1 giro al secondo
  //_PWM_output = 141; // 60 RPM -> 1 giro al secondo
  //_PWM_output = 153/2; // 60 RPM -> 1/2 giro al secondo
 // _PWM_output =50;

   
    myPID->Compute();
    _PWM_output =  abs(_PID_output);

   double pid =  calcolaPID();
   _PWM_output= pid;
  
    Serial.print("OUT:  ");
    Serial.print(_PWM_output);
     Serial.print(" - ");
    Serial.println(currentRPM);
  //     Serial.print(" - ");
  //  Serial.println(pid);
    
 /* _PWM_output = abs(_PID_output) + _pwm_skip;
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
  */

 // Serial.print("OUT");
//  Serial.println(_PWM_output);
  
  _PWM_output=0;
  //_pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
  
}

void DCMotorServo::stop() {
   Serial.println("STOP");
   myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  analogWrite(_pin_PWM_output, _PWM_output);
}

void DCMotorServo::setDirection(bool forward) {
  if (forward)
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

