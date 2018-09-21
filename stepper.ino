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
const double d_k = 0.168 + 0.027/2; // try and error
const double u_fullturn = d_k / d_r;
const double u_halfturn = u_fullturn / 2.0;

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

unsigned long calculateStepsForAcceleration(double desiredSpeed) {
  double deltaV = desiredSpeed -m.currentSpeed;
  double t = deltaV / m.acceleration;
  return  1.0 / 2.0 * m.acceleration / 60.0 * stepsPerCycle * t * t + m.currentSpeed / 60.0 * stepsPerCycle * t;
}


volatile unsigned long echo_start = 0;
volatile double distance = -1;
volatile int exCounterH = 0;
volatile int exCounterL = 0;
void entfernung() {
  

  switch (digitalRead(2)){
    case HIGH:                                      
      exCounterH++;
      echo_start = micros();                        
      break;
      
    case LOW:      
      exCounterL++;                                 
      distance = (micros() - echo_start)  * 100.0 * 340.0/(2.0 * 1000.0 * 1000.0);
      detachInterrupt(digitalPinToInterrupt(2));
      break;
  }
}


void loop() {
  static unsigned long stepsStart = 0;
  static unsigned long t_main = micros();
  static int mainState = 0;

  unsigned long now = micros();
  motorSpeed(now, &m);
  
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
        t_main = now;
      }
      break;
    case 3:  // strait forward
      if (now > (t_main + 100 * 1000)) {
//        Serial.println(distance);
        
        
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
      
      if (distance < 25) {
        m.desiredSpeed=0;
        mainState = 4;
      }
//      if (now > (t_main + 3 * 1000000)) {
//        m.desiredSpeed=0;
//        mainState = 4;
//      }
      break;
    case 4: // wait for break
      if (accelerationCompleted()) {
        mainState = 5;
        stepsStart = m.steps;
        m.forwardLeft = false;
        m.desiredSpeed = 20;
      }
      break;  
    case 5: // wait for acceleration
      if (accelerationCompleted()) {        
        mainState = 6;
      }
      break;  
    case 6:  // wait for turn completed
      if (m.steps >= (stepsStart - calculateStepsForAcceleration(0) + stepsPerCycle * u_halfturn)) {       
        m.desiredSpeed=0;
        mainState = 7;
      }  
      break;
    case 7: // wait for break
      if (accelerationCompleted()) {        
        mainState = 1;
      }
      break;
      
  }
}
