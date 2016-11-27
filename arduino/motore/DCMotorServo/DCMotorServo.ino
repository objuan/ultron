//Include Encoder and PID_v1 to enable DCMotorServo's usage of them.
//(see: http://stackoverflow.com/questions/6504211/is-it-possible-to-include-a-library-from-another-library-using-the-arduino-ide)

// If you define ENCODER_DO_NOT_USE_INTERRUPTS *before* including
// Encoder, the library will never use interrupts.  This is mainly
// useful to reduce the size of the library when you are using it
// with pins that do not support interrupts.  Without interrupts,
// your program must call the read() function rapidly, or risk
// missing changes in position.
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include "DCMotorServo.h"

#define pin_dcmoto_dir1 4
#define pin_dcmoto_pwm_out 5

#define pin_dcmoto_encode1 3 // hanno l'interrupt
#define pin_dcmoto_encode2 2
//Determined by experimentation, depends on your encoder, and your belt/gearing ratios:
//#define dcmoto_encoder_1_inch 1344
// 10 giri
//#define dcmoto_speed_rpm 151
#define dcmoto_speed_rpm 20


DCMotorServo servo = DCMotorServo(pin_dcmoto_dir1,  pin_dcmoto_pwm_out, pin_dcmoto_encode1, pin_dcmoto_encode2);

void setup() {

  Serial.begin(9600);
  Serial.println("INIT");
 
  //Tune the servo feedback
  //Determined by trial and error
  servo.myPID->SetTunings(0.1,0.15,0.05);
  servo.setPWMSkip(50);
  //servo.setAccuracy(20);
  //Un-necessary, initializes to 0:
  //servo.setCurrentPosition(0);

   servo.setSpeed(dcmoto_speed_rpm);
}


void loop() {
  //
    
  //wait 1s before starting
  static unsigned long motor_timeout = millis() + 1000;
  static bool motor_go = 0;

//  Serial.println("LOPP:");
//  Serial.println(millis() );
 
  servo.run();

 /* if (servo.finished()) {
    if(motor_go) {
      //stop disengages the motor feedback system, in which case if you moved the motor, it would fight you if you didn't "stop" it first (provided you were still running servo.run() operations)
    //  servo.stop();
    //  motor_timeout = millis() + 1500;
      motor_go = 0;
    }
    
    if(motor_timeout < millis()) {
      //Setting a move operation will ensure that servo.finished() no longer returns true
      servo.setSpeed(dcmoto_speed_rpm);
      motor_go = 1;
    }
  }
  */
}
