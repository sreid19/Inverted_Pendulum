volatile boolean dir;
unsigned long starttime, endtime = 0;
volatile unsigned int temp, counter = 512; //This variable will increase or decrease depending on the rotation of encoder
const byte numChars = 32;

//volatile float integral, derivative, proportional = 0;
volatile float integral = 0;
volatile float current_time = 0;
volatile float I0 = 75 * (0.4 * 1E-5);
volatile float D0 = 1 * (1 * 1E5);
volatile float P0 = 85;
volatile int runner = 0;
volatile float prev_angle = 0;
volatile float PID = 0;
volatile float current_angle = 0;
volatile float delta_t = 0;
volatile float proportional = 0;
volatile float derivative = 0;
volatile float current_integral = 0;
volatile int factor = 1;
volatile int pastDelay = 0;
volatile int count = 0;
float integrals[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float derivatives[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
int samples = 20;
float pid0 = 200;

#define EN 8

//Direction pin
#define X_DIR 7

//Step pin
#define X_STP 4

#define Z_PULSE 3

//A498
int delayTime = 601;
int stps=1;


void pid1() {
  //float starter = micros();
  //Serial.println(PID);
  current_angle = ((counter%1024)/1024.0*360.0);
  //Serial.println(current_angle)
  delta_t = micros() - current_time;
  current_time = micros();
  float error_value = 180 - current_angle;
  proportional = error_value;
  //Serial.println(current_angle);

  derivatives[count] = ((current_angle - prev_angle) / delta_t);

  current_integral = error_value * delta_t;
  //integrals[count] = current_integral;
  //integral = integrals[0] + integrals[1] + integrals[2] + integrals[3] + integrals[4] + integrals[5] + integrals[6] + integrals[7] + integrals[8] + integrals[9] + integrals[10] + integrals[11] + integrals[12] + integrals[13] + integrals[14] + integrals[15] + integrals[16] + integrals[17] + integrals[18] + integrals[19];
  integral = integral + current_integral;
  if (integral > 400){
    integral = 400;
  }
  else if(integral < -400){
    integral = -400;
  }
  //Serial.println(I0 * integral);
  //Serial.println(PID);
  //Serial.println(PID);
  //PID = proportional;

  //float ender = micros() - starter;
  //Serial.println(ender);
  }

void pid2(){
  //float starter = micros();
  derivative = (derivatives[0] + derivatives[1] + derivatives[2] + derivatives[3] + derivatives[4] + derivatives[5] + derivatives[6] + derivatives[7] + derivatives[8] + derivatives[9] + derivatives[10] + derivatives[11] + derivatives[12] + derivatives[13] + derivatives[14] + derivatives[15] + derivatives[16] + derivatives[17] + derivatives[18] + derivatives[19]) / 20;
  
  PID = P0 * proportional + I0 * integral + D0 * derivative;
  factor = abs(PID) / PID;
  PID = abs(PID);

  if(PID > 25){
    PID = 25;
  }
  else if(PID < 1.25){
    PID = 1.25;
  }

  PID = pid0 * PID;

  delayTime = 5390 - PID;
  //Serial.println(delayTime);

  //Serial.print("Proportional = ");
  //Serial.println(P0 * proportional);
  //Serial.print("Integral = ");
  //Serial.println(I0 * integral);
  //Serial.print("Derivative = ");
  //Serial.println(D0 * derivative);
  //Serial.print("PID = ");
  //Serial.println(PID);
  //float ender = micros() - starter;
  //Serial.println(ender);

  //float ender = micros() - starter;
  //Serial.println(ender);
  if (count < samples){
    count = count + 1;
  }
  else{
    count = 0;
  }
  prev_angle = current_angle;
  //Serial.println(current_angle);
}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(5)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  counter = 517;
  //Serial.println("Here");
}

void step(byte dirPin, byte stepperPin, int steps)
{
  if(factor > 0 ){
    dir =true;}
  if(factor < 0 ){
    dir = false;
  }
  //Serial.println(factor);
  
  //Serial.println(delayTime);
  digitalWrite(dirPin, dir);
  for (int i = 0; i< steps; i++)
  {
    //Serial.println(PID);
    digitalWrite(stepperPin, HIGH);
    
    starttime = micros();
    pid1();
    //Serial.println(delayTime - (micros()-starttime));
    delayMicroseconds(delayTime - (micros()-starttime));
    
    digitalWrite(stepperPin, LOW);
    
    starttime = micros();
    pid2();
    delayMicroseconds(delayTime - (micros()-starttime));
    //Serial.println(delayTime);
  }
}


void setup() {
  Serial.begin (38400);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
//Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, HIGH);


  pinMode(X_DIR, OUTPUT); pinMode(X_STP,OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN,LOW);
  //Serial.println("<Arduino is ready>");
  //Serial.println(0);
  current_time = micros();
  prev_angle = ((counter%1024)/1024.0*360.0);
  pid1();
  pid2();
}


void loop() {
  step(X_DIR, X_STP, stps);
  }
