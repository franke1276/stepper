#include "TimerOne.h"  
const int MOTOR_LEFT_STEP = 2; 
const int MOTOR_LEFT_DIR = 5; 

const int MOTOR_RIGHT_STEP = 3; 
const int MOTOR_RIGHT_DIR = 6; 

const int enPin = 8; 

const int stepsPerCycle = 200 * 8; 
const double d_r = 0.067;
const double d_k = 0.168; 

const double u_fullturn = d_k / d_r;
const int stepsToStop = 136;

void setup() {
  // Sets the two pins as Outputs
  pinMode(enPin,OUTPUT);
  pinMode(MOTOR_LEFT_STEP,OUTPUT); 
  pinMode(MOTOR_LEFT_DIR,OUTPUT);
  pinMode(MOTOR_RIGHT_STEP,OUTPUT); 
  pinMode(MOTOR_RIGHT_DIR,OUTPUT);
  
  digitalWrite(enPin,LOW); 
  Serial.begin(57600, SERIAL_8N1);
  Serial.print("u_fullturn: ");
  Serial.println(u_fullturn);
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
};

//typedef struct motor Motor;

volatile struct motor m = {0, 0, micros(), 0, 0, true, true, 0};

const double a0 = 50;
volatile bool toogle = false;

void toogleStepper() {
  if (toogle) {
    PORTD = PORTD | B00001100;
  } else {
    PORTD = PORTD & B11110011;
    m.steps++;
  }
  toogle = !toogle;
}


void motorSpeed( unsigned long now, double acceleration, struct motor* m) {
  const int timeFrame = 10000;

  
  static unsigned long ta = now;
  static unsigned long tb = now;
  static double a = 0;
  static int maxSteps = 0;
  static int currentStep = 0;
  static double deltaVPerStep = 0;
  static unsigned long untilTime = 0;
  
  int maxCount = 0;
  double deltaVPerStepStart = acceleration / 100.0;
  
  switch(m->accelerationState) {
    case 0: // hold speed phase
      if (m->currentSpeed != m->desiredSpeed) {
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
 
//  if (m.currentSpeed == 0) {
//    return;
//  }
//  
//  
//  switch(m.state) {
//    case 0:
//      untilTime = (1.0 / (m.currentSpeed/60.0 * stepsPerCycle * 2)) * 1000 * 1000;
//      if (micros() >= m.t0 + untilTime) {
//        PORTD = PORTD | B00001100;
//        m.t0 = micros();  
//        m.state = 1;
//      }
//      break;
//     case 1:
//      if (micros() >= m.t0 + untilTime) {
//        PORTD = PORTD & B11110011;
//        m.t0 = micros();  
//        m.state = 0;
//        m.steps++;
//      }
//      break; 
//  }
}


double t = 0;
int mainState = 0;

volatile unsigned long steps = 0;
unsigned long t1 = micros();
unsigned long stepsWhenStopping = 0;

void loop() {
  unsigned long now = micros();
  double acc = 60;
  double t = 0;
  unsigned long stepsForAcc = 0;
  motorSpeed(now, acc, &m);
  
  

  switch (mainState) {
    case 0: // wait for start
      if ((now > t1 + 3 * 1000000)) {
        t1 = now;
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
      if (m.desiredSpeed == m.currentSpeed) {
        
        mainState = 3;
        Serial.print("estimated steps for 3s: ");
        Serial.println(m.currentSpeed/60.0*stepsPerCycle * 3);
        steps = m.steps;
        t1 = now;
      }
      break;
    case 3:  // strait forward
      if (now > (t1 + 3 * 1000000)) {
        int s = m.steps - steps;
        Serial.print("actual steps for 3s: ");
        Serial.println(s);
        m.desiredSpeed=0;
        t1 = now;
        mainState = 4;
      }
      break;
    case 4: // wait for break
      if (m.desiredSpeed == m.currentSpeed) {
        
        mainState = 5;
        steps = m.steps;
        m.forwardLeft = false;
        m.desiredSpeed = 20;
        Serial.print("steps to turn: ");
        Serial.println(stepsPerCycle * u_fullturn / 2.0);
      }
      break;  
    case 5: // wait for acceleration
      if (m.desiredSpeed == m.currentSpeed) {        
        Serial.print("full accelerated: ");
        Serial.println(m.steps - steps);
        t1 = now;
        mainState = 6;
      }
      break;  
    case 6:  // wait for turn complete

      t = m.currentSpeed / acc;
      
      stepsForAcc = -1.0/2.0 * acc/60.0*stepsPerCycle * t*t + m.currentSpeed/60.0*stepsPerCycle * t;
      
      if (m.steps >= (steps - stepsForAcc + (stepsPerCycle * u_fullturn / 2.0))) {       
        Serial.print("t: ");
        Serial.print(t);
        Serial.print(" steps: ");
        Serial.print(stepsForAcc);
        Serial.print(", steps done: ");
        Serial.println(m.steps - steps);
        m.desiredSpeed=0;
        t1 = now;
        mainState = 7;
      }  
      break;
    case 7: // wait for break
      if (m.desiredSpeed == m.currentSpeed) {        
        
        Serial.print("break done: ");
        Serial.print("t: ");
        Serial.print((now - t1) / 1000.0);
        Serial.print(" steps: ");
        Serial.println(m.steps - steps);
        t1 = now;
        mainState = 1;
      }
      break;
      
  }
}
