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
  int stepPin;
  int dirPin;
  int state;
  unsigned long steps;
  unsigned long t0;
  double speed;
  bool forward;
};

typedef struct motor Motor;

Motor motorLeft = {MOTOR_LEFT_STEP, MOTOR_LEFT_DIR, 0, 0, micros(), 60, true};
Motor motorRight = {MOTOR_RIGHT_STEP, MOTOR_LEFT_DIR, 0, 0, micros(), 60, true};

unsigned long t1 = micros();

void motorSpeed(struct motor& m) {
  if (m.speed == 0) {
    return;
  }
  unsigned long now = micros();
  unsigned long untilTime = (1.0 / (m.speed/60.0 * stepsPerCycle * 4)) * 1000 * 1000;
  switch(m.state) {
    case 0:
      if (now > t0 + untilTime) {
        digitalWrite(m.dirPin, m.forward? HIGH : LOW);
        digitalWrite(m.stepPin, HIGH); 
        m.t0 = micros();  
        m.state = 1;
      }
      break;
     case 1:
      if (now > m.t0 + untilTime) {
        digitalWrite(m.dirPin, m.forward? HIGH : LOW);
        digitalWrite(m.stepPin, LOW); 
        m.t0 = micros();  
        m.state = 0;
        m.steps++;
      }
      break; 
  }
}

void loop() {
  unsigned long now = micros();
  motorSpeed(motorLeft);
  motorSpeed(motorRight);

  if (now > t1 + 1000000) {
    Serial.println(steps);
    t1 = now;
  }
 
}
