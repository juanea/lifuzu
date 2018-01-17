#include <PID_v1.h>

int dirY = 6;
int stepY = 3;
int dirX = 5;
int stepX = 2;
int del = 600;

#define pull  A2//2                       // PWM outputs to L298N H-Bridge motor driver module
#define push  A1//3
#define enablePID  A3//9

#define ledPin 9

#define magnetX 12
#define magnetY 13
#define enableMOT 8

double kp = 20, ki = 0, kd = 0;             // modify for optimal performance
//double kp = 50, ki = 10, kd = 10;             // modify for optimal performance
double setPoint, input, output;       //setPoint is set in setUp by user
const int SampleTime = 1;
const int minPWM = -255;
const int maxPWM = 255;
int ledIntensity = 255;
long temp;

PID myPID(&input, &output, &setPoint, kp, ki, kd, DIRECT);

void setup() {
 Serial.begin(115200);
 pinMode(dirX, OUTPUT);
 pinMode(stepX, OUTPUT);
 pinMode(dirY, OUTPUT);
 pinMode(stepY, OUTPUT);
 pinMode(enableMOT,OUTPUT);
 pinMode(magnetX,INPUT);
 pinMode(magnetY,INPUT);
 digitalWrite(enableMOT,LOW);
  analogWrite(ledPin,ledIntensity);
  
 stepMagnet(true,dirY,stepY,magnetY);
 stepMagnet(true,dirX,stepX,magnetX);  

  TCCR1B = TCCR1B & 0b11111000 | 1; //  Set 31KHz PWM to prevent motor noise
  input = analogRead(A0);
  setPoint = 300;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(SampleTime);
  myPID.SetOutputLimits(minPWM, maxPWM);
  digitalWrite(enablePID, HIGH);

  pinMode(pull, OUTPUT);  //When HIGH the actuator push
  pinMode(push, OUTPUT);  //When HIGH the actuatot pull

  setPoint = 990;
  //  readSerial();
}

void oneStep(long stepPin)
{
   digitalWrite(stepPin, HIGH);
   delayMicroseconds(del);
   digitalWrite(stepPin, LOW);
   delayMicroseconds(del);
}

void step(boolean dir,long steps, int dirPin,int stepPin){
 digitalWrite(dirPin,dir);
 delay(50);
 long x=0;
 for(long i=0;i<steps;i++)
    oneStep(stepPin);
}

void stepMagnet(boolean dir,int dirPin,int stepPin,int magnet)
{
 digitalWrite(dirPin,dir);
 delay(50);
 bool val= (bool)digitalRead(magnet);
 while(!val)
 {
  oneStep(stepPin);
  val= (bool)digitalRead(magnet);
 }
  while(val)
 {
   oneStep(stepPin);
   val= (bool)digitalRead(magnet);
 }
 digitalWrite(dirPin,!dir);
 
 while(!val)
 {
  oneStep(stepPin);
  val= (bool)digitalRead(magnet);
 }
  long p=0;
 while(val)
 {
   p++;
   oneStep(stepPin);
   val= (bool)digitalRead(magnet);
 }
 
 digitalWrite(dirPin,dir);
 while(!val)
 {
   oneStep(stepPin);
   val= (bool)digitalRead(magnet);
 }
   p/=2;
 while(p)
 {
    p--;
    oneStep(stepPin);
    val= (bool)digitalRead(magnet);
 }
}
 //Data feedback between 45 and 945
void loop()
{
  input = analogRead(A0); // data from encoder
  //Serial.print("Feedback : ");
  //Serial.println(input);  // monitor motor position
  myPID.Compute();  // calculate new OUTPUT
  pwmOut(output); // drive L298N H-Bridge module
}

void readSerial()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    setPoint = Serial.parseFloat();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(setPoint, DEC);
  }
}
//Signal to the H-Bridge board
void pwmOut(int out)
{
  if (out > 0)
  {
    analogWrite(pull, out);   //Pulling linear actuator
    analogWrite(push, LOW);
  }
  else
  {
    analogWrite(pull, LOW);
    analogWrite(push, abs(out));  //Pushing linear actuator
  }
}
