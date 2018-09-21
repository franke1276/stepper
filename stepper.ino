#include "TimerOne.h"  
const int MOTOR_LEFT_STEP = 4; 
const int MOTOR_LEFT_DIR = 7; 

const int MOTOR_RIGHT_STEP = 3; 
const int MOTOR_RIGHT_DIR = 6; 

const int trigger = 9; 
const int echo = 10; 

const int enPin = 8; 

const int stepsPerCycle = 200 * 8; 
const double d_r = 0.067;
const double d_k = 0.168 + 0.027/2 + 0.005; // try and error
//const double d_k = 0.168 + 0.031; // try and error
//const double d_k = 0.168 ; 
const double u_fullturn = d_k / d_r;
const double u_halfturn = u_fullturn / 2.0;

const double u_one_cm = 1.0 / (d_r * PI * 100.0 );

void setup() {
  pinMode(enPin,OUTPUT);
  pinMode(MOTOR_LEFT_STEP,OUTPUT); 
  pinMode(MOTOR_LEFT_DIR,OUTPUT);
  pinMode(MOTOR_RIGHT_STEP,OUTPUT); 
  pinMode(MOTOR_RIGHT_DIR,OUTPUT);
  
  digitalWrite(enPin,LOW); 
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trigger, HIGH); //Signal abschalten
  pinMode(2, INPUT_PULLUP);
  Serial.begin(57600, SERIAL_8N1);
}


struct motor {
  int state;
  volatile unsigned long steps;
  unsigned long t0;
  double desiredSpeed;
  double currentSpeed;
  bool forwardLeft;
  bool forwardRight;
  int accelerationState;
  double acceleration;
};


volatile struct motor m = {0, 0, micros(), 0, 0, true, true, 0, 60.0};



void toogleStepper() {
  volatile static bool toogle = false;
  if (toogle) {
    PORTD = PORTD | B00011000;
  } else {
    PORTD = PORTD & B11100111;
    m.steps++;
  }
  toogle = !toogle;
}


void motorSpeed( unsigned long now, struct motor* m) {
  const int timeFrame = 10000;
  
  static unsigned long tb = now;
  static int maxSteps = 0;
  static int currentStep = 0;
  static double deltaVPerStep = 0;
  
  switch(m->accelerationState) {
    case 0: // hold speed phase
      if (m->currentSpeed != m->desiredSpeed) {
        double deltaVPerStepStart = m->acceleration / 100.0;
        m->accelerationState = 1;
        double deltaV = m->desiredSpeed - m->currentSpeed;
        maxSteps = abs(deltaV) / deltaVPerStepStart;
        
        if (deltaV > 0) {
          deltaVPerStep = deltaVPerStepStart; 
        } else {
          deltaVPerStep = -1 * deltaVPerStepStart; 
        }
        m->currentSpeed += deltaVPerStep;
        currentStep = 1;
        tb = micros();
      }
      break;
    case 1: // acceleration phase
      if (micros() > (tb + timeFrame)) {       
        m->currentSpeed += deltaVPerStep;
        digitalWrite(MOTOR_LEFT_DIR, m->forwardLeft? HIGH : LOW);
        digitalWrite(MOTOR_RIGHT_DIR, m->forwardRight? LOW: HIGH);
        Timer1.detachInterrupt();
        if (m->currentSpeed > 0) {
          Timer1.initialize((1.0 / (m->currentSpeed/60.0 * stepsPerCycle * 2)) * 1000 * 1000);
          Timer1.attachInterrupt(toogleStepper);
        }
        if (currentStep >= maxSteps) {
          m->currentSpeed = m->desiredSpeed;
          m->accelerationState = 0;
        } else {
          currentStep++;  
          tb = micros();
        }        
      }
      break;
  }
 

}

bool accelerationCompleted() {
  return m.desiredSpeed == m.currentSpeed;
}

long calculateStepsForAcceleration(double currentSpeed, double desiredSpeed) {
  double deltaV = desiredSpeed - currentSpeed;
  double t = deltaV / m.acceleration;
  double result =  1.0 / 2.0 * m.acceleration / 60.0 * stepsPerCycle * t * t + currentSpeed / 60.0 * stepsPerCycle * t;
  return  abs(result);
}


volatile unsigned long echo_start = 0;
volatile double distance = 10000;
void entfernung() {
  noInterrupts();

  switch (digitalRead(2)){
    case HIGH:                                      
      echo_start = micros();                        
      break;
      
    case LOW:      
      distance = (micros() - echo_start)  * 100.0 * 340.0/(2.0 * 1000.0 * 1000.0);
      detachInterrupt(digitalPinToInterrupt(2));
      break;
  }
  interrupts();
}


void loop() {
  static long stepsStart = 0;
  static unsigned long t_main = micros();
  static int mainState = 0;

  unsigned long now = micros();
  motorSpeed(now, &m);


//  switch (mainState) {
//    case 0: // wait for start
//      if ((now > t_main + 3 * 1000000)) {
//        mainState = 1;
//      }
//      break;
//
//    case 1: // start
//      stepsStart = m.steps;
//      m.desiredSpeed=20;
//      m.forwardLeft = true;
//      m.forwardRight = false;
//      mainState = 2;
//      break;  
//    case 2: 
////          Serial.print(u_one_cm);
////      Serial.print(" ");
////      Serial.print(stepsPerCycle * u_one_cm * 15);
////      Serial.print(" ");
////      Serial.print(calculateStepsForAcceleration(m.desiredSpeed, 0));
////      Serial.print(" ");
////      Serial.print(stepsStart);
////      Serial.print(" ");
////      
////      Serial.println((stepsStart - calculateStepsForAcceleration(m.desiredSpeed, 0) + stepsPerCycle * u_one_cm * 15));
//
//      
//
////      Serial.println(stepsStart - calculateStepsForAcceleration(0) + stepsPerCycle * u_one_cm * 15);
//      if (m.steps >= (stepsStart - calculateStepsForAcceleration(m.desiredSpeed, 0) + stepsPerCycle * u_halfturn)) {       
//        m.desiredSpeed=0;
//        mainState = 3;
//        stepsStart = m.steps;
//        Serial.println(calculateStepsForAcceleration(20, 0));
//        
//      }  
//      break;
//    case 3:
//        if (accelerationCompleted()) {
//          mainState = 4;
//          Serial.println(m.steps - stepsStart);
//        }
//      break;
//    case 4:
//      break;
//  }
//  











  
  switch (mainState) {
    case 0: // wait for start
      if ((now > t_main + 3 * 1000000)) {
        mainState = 1;
      }
      break;

    case 1: // start
      m.desiredSpeed=30;
      m.forwardLeft = true;
      m.forwardRight = true;
      mainState = 2;
      break;  
    case 2: // wait for acceleration
      if (accelerationCompleted()) {
        mainState = 3;
        distance = 10000;
        t_main = now;
      }
      break;
    case 3:  // strait forward
      if (now > (t_main + 200 * 1000)) {
        Serial.println(distance);
        digitalWrite(trigger, LOW);
        delayMicroseconds(3);
        noInterrupts();
        digitalWrite(trigger, HIGH); 
        delayMicroseconds(10);
        digitalWrite(trigger, LOW);
        interrupts();
        attachInterrupt(digitalPinToInterrupt(2), entfernung, CHANGE);  
        t_main = now;
      }
      
      if (distance < 10) {
        m.desiredSpeed=0;
        mainState = 4;
      }
      break;
    case 4: // wait for break
      if (accelerationCompleted()) {
        mainState = 5;
        stepsStart = m.steps;
        m.forwardLeft = false;
        m.forwardRight = false;
        m.desiredSpeed = 20;
        stepsStart = m.steps;
      }
      break;  
    case 5:  // wait for 15 cm back
//      Serial.print(u_one_cm);
//      Serial.print(" ");
//      Serial.print(stepsPerCycle * u_one_cm * 15);
//      Serial.print(" ");
//      Serial.println(calculateStepsForAcceleration(0));
      if (m.steps >= (stepsStart - calculateStepsForAcceleration(m.desiredSpeed, 0) + stepsPerCycle * u_one_cm * 20)) {       
        m.desiredSpeed=0;
        mainState = 6;
      }  
      break;
    case 6: // wait for break
      if (accelerationCompleted()) {        
        mainState = 7;
        stepsStart = m.steps;
        m.forwardLeft = false;
        m.forwardRight = true;
        m.desiredSpeed = 20;
      }
      break;  
    case 7:  // wait for turn completed
      if (m.steps >= (stepsStart - calculateStepsForAcceleration(m.desiredSpeed, 0) + stepsPerCycle * u_halfturn)) {       
        m.desiredSpeed=0;
        mainState = 8;
      }  
      break;
    case 8: // wait for break
      if (accelerationCompleted()) {        
        mainState = 1;
      }
      break;
      
  }
}
