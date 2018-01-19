// https://www.dimensionengineering.com/products/sabertooth2x12
//i2c Master Code(UNO)
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
bool evenOdd = false;
// Sabertooth Related
SoftwareSerial SWSerial(31, 27);  // RX on pin 2 (unused), TX on pin 3 (to S1).
Sabertooth ST(128, SWSerial);     // Address 128, and use SWSerial as the serial port.

// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit

const int slaveSelectEnc1 = 9;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;

signed int distance1;
signed int distance2;

const int Trig_pin =  10;   // pin for triggering pulse
const int Echo_pin = 11;     // pin for recieving echo
long duration;

int sabPower = 65;


void initSabertooth() {
  SWSerial.begin(9600);
  ST.autobaud();
}

void initEncoders() {

  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);

  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);

  SPI.begin();

  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation
}

void initUtrasonic(){
   // initialize the pulse pin as output:
  pinMode(Trig_pin, OUTPUT);
  // initialize the echo_pin pin as an input:
  pinMode(Echo_pin, INPUT);
}

long readEncoder(int encoder) {

  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;

  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
  }

  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
  }

  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  return count_value;
}

long readUltrasonic(){
  digitalWrite(Trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_pin, LOW);
  long readUltrasonicRangeFinder = pulseIn(Echo_pin,HIGH);
  return readUltrasonicRangeFinder;
}

void clearEncoderCount() {
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation

  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
}

void setup() {
 Serial.begin(9600);      // Serial com for data output
 Wire.begin();
 initSabertooth();     Serial.println("Sabertooth Initialized...");
 initEncoders();
    Serial.println("Encoders Initialized...");
 initUtrasonic();      Serial.println("Ultrasonic Range Finder Initialized...");
 clearEncoderCount();  Serial.println("Encoders Cleared...");
 //ST.motor(1, sabPower);
 //ST.motor(2, sabPower);

 Wire.beginTransmission(5);
 Wire.write('S');
 Wire.endTransmission();
}

long until = 200;
long pipeLength = 1600;
signed long encDis1=0,encDis2=0;
bool flag = true;

void loop()
{

  encoder1count = readEncoder(1);
  encoder2count = readEncoder(2);

  if(flag == true)
  {
    if(encoder1count+encoder2count == 0)
    {
      ST.motor(1, sabPower);
      ST.motor(2, sabPower);
      Serial.println("start_forward");
    }
    if(encoder1count >= pipeLength || encoder2count >= pipeLength)
    {
      flag = false;
      ST.motor(1, 0);
      ST.motor(2, 0);
      delay(2000);
      ST.motor(1, -sabPower);
      ST.motor(2, -sabPower);
      Serial.println("Start_backwards");
    }
    else if(encoder1count >= until || encoder2count >= until)
    {
       ST.motor(1, 0);
       ST.motor(2, 0);
       delay(3000);
       ST.motor(1,sabPower);
       ST.motor(2, sabPower);
       until+=200;

       Wire.beginTransmission(5);
       if(evenOdd == true)
       Wire.write('L');
       else
       Wire.write('R');
       Wire.endTransmission();

       char wait = 'a';
      while(wait == 'a')
      {
        Wire.requestFrom(5,10);

        while(Wire.available())
          wait = Wire.read();
      }
       evenOdd = !evenOdd;
       Serial.println("Move steppers");
    }
  }
  else{
    if( encoder1count <= 0 ||  encoder2count <= 0){
    //  flag = true;
      ST.motor(1, 0);
      ST.motor(2, 0);
      Serial.println("The end");
  //    delay(2000);
   //   ST.motor(1, sabPower);
   //   ST.motor(2, sabPower);
    }
  }

//  Serial.print("Duration: ");
//  Serial.print(duration);
  Serial.print(" Encoder1: ");
  Serial.print(encoder1count);
  Serial.print(" Encoder2: ");
  Serial.println(encoder2count);

  delay(25);
}
