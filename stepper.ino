const int MOTOR_LEFT_STEP = 2; 
const int MOTOR_LEFT_DIR = 5; 

const int MOTOR_RIGHT_STEP = 3; 
const int MOTOR_RIGHT_DIR = 6; 

const int enPin = 8; 

const int stepsPerCycle = 200 * 8; 
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(enPin,OUTPUT);
  pinMode(MOTOR_LEFT_STEP,OUTPUT); 
  pinMode(MOTOR_LEFT_DIR,OUTPUT);
  pinMode(MOTOR_RIGHT_STEP,OUTPUT); 
  pinMode(MOTOR_RIGHT_DIR,OUTPUT);
  
  digitalWrite(enPin,LOW); 
  Serial.begin(57600, SERIAL_8N1);
}

int state = 0;
unsigned long steps = 0;
unsigned long t0 = micros();
double speed = 60;


struct motor {
  int state;
  unsigned long steps;
  unsigned long t0;
  double desiredSpeed;
  double currentSpeed;
  bool forwardLeft;
  bool forwardRight;
};

typedef struct motor Motor;

Motor m = {0, 0, micros(), 0, 0, true, true};


unsigned long t1 = micros();
const double a0 = 50;

void motorSpeed(unsigned long now, struct motor& m) {
  static unsigned long ta = now;
  static unsigned long tb = now;
  static double a = 0;
  static int accelerationState = 0;
  int steps = 100;
  static double deltaVPerStep = 0;
  static int count = 0;
  int maxCount = 0;


  switch(accelerationState) {
    case 0: // hold speed phase
      if (m.currentSpeed != m.desiredSpeed) {
        accelerationState = 1;
        deltaVPerStep = (m.desiredSpeed - m.currentSpeed) / steps;
        m.currentSpeed += deltaVPerStep;
        count = 1;
        tb = now;
      }
      break;
    case 1: // acceleration phase
      if (now > (tb + 1000000 / steps)) {
        m.currentSpeed += deltaVPerStep;
        
        if (count == steps) {
          m.currentSpeed = m.desiredSpeed;
          accelerationState = 0;
        } else {
          count++;  
          tb = now;
        }
        Serial.print("1: ");
        Serial.println(m.currentSpeed);
      }
      break;
  }
  
//  if (now > (ta + 10000)) {
//    double t = (now - ta) / (1000.0 * 1000.0 );
//    
//    if (m.currentSpeed > m.desiredSpeed) {
//      a = -a0;
//    } else if (m.currentSpeed < m.desiredSpeed) {
//      a = a0;
//    } else {
//      a = 0;
//    }
//    m.currentSpeed += a * t;
//    ta = now;  
//  }
  
  if (m.currentSpeed == 0) {
    return;
  }
  unsigned long untilTime = (1.0 / (m.currentSpeed/60.0 * stepsPerCycle * 4)) * 1000 * 1000;
  digitalWrite(MOTOR_LEFT_DIR, m.forwardLeft? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_DIR, m.forwardRight? LOW: HIGH);
  switch(m.state) {
    case 0:
      if (now > t0 + untilTime) {
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
void loop() {
  unsigned long now = micros();
  motorSpeed(now, m);
  

  

  if ((now > t1 + 3000000)  && m.desiredSpeed == 0 ) {
    m.desiredSpeed=30;
    t1 = now;
  }
  
  if ((now > t1 + 3000000) && m.desiredSpeed == 30) {
    m.desiredSpeed=0;
    t1 = now;
  }
  
 
}
