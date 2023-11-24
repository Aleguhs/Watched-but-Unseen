
//  X-axis joystick pin: A1
//  Y-axis joystick pin: A0
//  Trim potentiometer pin: A2
//  Button pin: 2

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  520 // this is the 'maximum' pulse length count (out of 4096)

uint8_t servonum = 0;

int xval;
int yval;

int lexpulse;
int rexpulse;

int leypulse;
int reypulse;

int uplidpulse;
int lolidpulse;

int trimval;

const int analogInPin = A0;
int sensorValue = 0;
int outputValue = 0;
int switchval = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");
  pinMode(analogInPin, INPUT);
  pinMode(2, INPUT);
 
  pwm.begin();
  
  pwm.setPWMFreq(60);  
  delay(10);
}


void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   
  pulselength /= 60;   
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  
  pulse /= pulselength;
  Serial.println(pulse);
 
}

void loop() {

  xval = analogRead(A1);
    lexpulse = map(xval, 0,1023, 270, 390);
    rexpulse = lexpulse;

    switchval = digitalRead(2);
    
    
  yval = analogRead(A0);
    leypulse = map(yval, 0,1023, 280, 400);
    reypulse = map(yval, 0,1023, 400, 280);

  trimval = analogRead(A2);
    trimval=map(trimval, 320, 580, -40, 40);
      uplidpulse = map(yval, 0, 1023, 280, 420);
        uplidpulse += (trimval-40);
          uplidpulse = constrain(uplidpulse, 280, 400);
      lolidpulse = map(yval, 0, 1023, 410, 280);
        lolidpulse += (trimval/2);
          lolidpulse = constrain(lolidpulse, 280, 400);      
    
    
      pwm.setPWM(0, 0, lexpulse);
      pwm.setPWM(1, 0, leypulse);
      pwm.setPWM(2, 0, rexpulse);
      pwm.setPWM(3, 0, reypulse); 

      if (switchval == HIGH) {
      pwm.setPWM(4, 0, 240);
      pwm.setPWM(5, 0, 240);
      }
      else if (switchval == LOW) {
      pwm.setPWM(4, 0, uplidpulse);
      pwm.setPWM(5, 0, lolidpulse);
      }



          Serial.println(lexpulse);
      
  delay(5);


}
