const int MOTOR_LEFT_STEP = 2; 
const int MOTOR_LEFT_DIR = 5; 

const int MOTOR_RIGHT_STEP = 3; 
const int MOTOR_RIGHT_DIR = 6; 

const int enPin = 8; 

const int stepsPerCycle = 200 * 8; 
const double d_r = 0.068;
const double d_k = 0.169; 
const double u_fullturn = d_k / d_r;

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
  unsigned long steps;
  unsigned long t0;
  double desiredSpeed;
  double currentSpeed;
  bool forwardLeft;
  bool forwardRight;
  int accelerationState;
};

typedef struct motor Motor;

Motor m = {0, 0, micros(), 0, 0, true, true, 0};



const double a0 = 50;

void motorSpeed( unsigned long now, double acceleration, struct motor& m) {
  const int timeFrame = 10000;

  
  static unsigned long ta = now;
  static unsigned long tb = now;
  static double a = 0;
  static int maxSteps = 0;
  static int currentStep = 0;
  static double deltaVPerStep = 0;
  
  int maxCount = 0;
  double deltaVPerStepStart = acceleration / 100.0;
  
  switch(m.accelerationState) {
    case 0: // hold speed phase
      if (m.currentSpeed != m.desiredSpeed) {
        m.accelerationState = 1;
        double deltaV = m.desiredSpeed - m.currentSpeed;
        maxSteps = abs(deltaV) / deltaVPerStepStart;
        
        if (deltaV > 0) {
          deltaVPerStep = deltaVPerStepStart; 
        } else {
          deltaVPerStep = -1 * deltaVPerStepStart; 
        }
        
//        Serial.print("speed change: ");
//        Serial.print(m.currentSpeed);
//        Serial.print(" -> ");
//        Serial.print(m.desiredSpeed);
//        Serial.print(" in ");
//        Serial.print(maxSteps);
//        Serial.print(" steps a ");
//        Serial.println(deltaVPerStep);
        
        m.currentSpeed += deltaVPerStep;
        currentStep = 1;
        tb = now;
        
      }
      break;
    case 1: // acceleration phase
      if (now > (tb + timeFrame)) {
//        Serial.print("acceleration phase: ");
//        Serial.println(m.currentSpeed);
        
        m.currentSpeed += deltaVPerStep;
        
        if (currentStep >= maxSteps) {
          m.currentSpeed = m.desiredSpeed;
          m.accelerationState = 0;
//          Serial.print("desiredSpeed reached: ");
//          Serial.println(m.currentSpeed);
        } else {
          currentStep++;  
          tb = now;
        }
        
      }
      break;
  }
  
  
  if (m.currentSpeed == 0) {
    return;
  }
  unsigned long untilTime = (1.0 / (m.currentSpeed/60.0 * stepsPerCycle * 4)) * 1000 * 1000;
  digitalWrite(MOTOR_LEFT_DIR, m.forwardLeft? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_DIR, m.forwardRight? LOW: HIGH);
  switch(m.state) {
    case 0:
      if (now > m.t0 + untilTime) {
        digitalWrite(MOTOR_LEFT_STEP, HIGH); 
        digitalWrite(MOTOR_RIGHT_STEP, HIGH); 
        m.t0 = micros();  
        m.state = 1;
      }
      break;
     case 1:
      if (now > m.t0 + untilTime) {
        digitalWrite(MOTOR_LEFT_STEP, LOW); 
        digitalWrite(MOTOR_RIGHT_STEP, LOW); 
        m.t0 = micros();  
        m.state = 0;
        m.steps++;
      }
      break; 
  }
}


double t = 0;
int mainState = 0;

unsigned long steps = 0;
unsigned long t1 = micros();

void loop() {
  unsigned long now = micros();
  motorSpeed(now, 60, m);
  


  switch (mainState) {
    case 0: // wait for start
      if ((now > t1 + 3 * 1000000)) {
        t1 = now;
        mainState = 1;
      }
      break;

    case 1: // start
      Serial.print(mainState);
      Serial.print(": ");
      Serial.println("start");
      m.desiredSpeed=30;
      m.forwardLeft = true;
      m.forwardRight = true;
      t1 = now;
      mainState = 2;
      break;  
    case 2: // wait for acceleration
      if (m.desiredSpeed == m.currentSpeed) {
        Serial.print(mainState);
        Serial.print(": ");
        Serial.println("strait forward");
        mainState = 3;
      }
      break;
    case 3:  // strait forward
      if ((now > t1 + 3 * 1000000)) {
        Serial.print(mainState);
        Serial.print(": ");
        Serial.println("stopping");
        m.desiredSpeed=0;
        t1 = now;
        mainState = 4;
      }
      break;
    case 4: // wait for acceleration
      if (m.desiredSpeed == m.currentSpeed) {
        Serial.print(mainState);
        Serial.print(": ");
        Serial.println("stopped. Turn!");
        mainState = 5;
        steps = m.steps;
        m.forwardLeft = false;
        m.desiredSpeed = 20;
      }
      break;  
    case 5:  // turn 
      if ((m.steps >= steps + (stepsPerCycle * u_fullturn / 2) )) {
        Serial.print(mainState);
        Serial.print(": ");
        Serial.println("Turn over! stopping...");
        m.desiredSpeed=0;
        t1 = now;
        mainState = 6;
      }  
      break;
    case 6: // wait for acceleration
      if (m.desiredSpeed == m.currentSpeed) {
        Serial.print(mainState);
        Serial.print(": ");
        Serial.println("Stopped.");
        t1 = now;
        mainState = 1;
      }
      break;
      
  }
}
