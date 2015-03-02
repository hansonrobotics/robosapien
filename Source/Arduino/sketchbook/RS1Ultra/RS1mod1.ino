//RS1mod1
//version 1.0 Beta
//Program for arduino pro mini 328 5v, to control Robosapien v1
//also adding functions like pan-tilt servo head,sonar,imu
//Copyright (C) 2015  Mandeep Singh Bhatia
/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*/
#include <SimpleTimer.h>
#include <Wire.h>
#include <Servo.h>
#include <HMC5883L.h>
#include <ADXL345.h>

#define print_on 1
//90 degree motor gives prob
#define MAX_A 90
#define MAX_B 90
#define DIFF_A 0xd
#define DIFF_B 0xd
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

HMC5883L compass;

//servo
Servo panServo,tiltServo;
#define panPin 9
#define tiltPin 10
#define servoAngle 90
//servo
//gyro
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
//gyro

//sonar ir
#define sonarIn 2
#define sonarTrig 13
#define IRout 5
const int tsDelay = 833; // us, as estimated
//#define bitTime 516
volatile unsigned long start_time=0;
volatile unsigned distance=0;
volatile char done=1;
//sonar ir

//baro
#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5; 
//baro
SimpleTimer timer;

//baro begin
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}
/*
void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}
*/
/*
float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
*/
//baro end
//gyro begin
void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
//gyro end

//sonar begin
void sonarDist()
{
  int a=digitalRead(sonarIn);
  if(a){
    start_time=micros();
  }else if (!done){
    unsigned long stop_time=micros();
    unsigned long duration=stop_time-start_time;
    distance = (duration/2) / 29.1;
    if (distance>200)distance=201;
    if (distance<4)distance=3;
    done=1;
    //if(print_on) Serial.print(distance);//if done
    //if(print_on) Serial.println(" cm");
  }
  
}
void trigSonar()
{
  //if(print_on) Serial.println("in trigger");
  if (done){
    done=0;
    digitalWrite(sonarTrig, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
//    delayMicroseconds(200); // Added this line
    digitalWrite(sonarTrig, HIGH);
//  delayMicroseconds(200);// - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(sonarTrig, LOW);
  }
}
//sonar end

//ir begin
void delayTs(unsigned int slices) {
  delayMicroseconds(tsDelay * slices); 
}

// send the whole 8 bits
void RSSendCommand(int cmd) 
{
  //Serial.println(cmd);
  // preamble
  digitalWrite(IRout, LOW);
  delayTs(8);
    
  for(char b = 7; b>=0; b--) {
    digitalWrite(IRout, HIGH);
    delayTs( (cmd & 1<<b) ? 4 : 1 );
    digitalWrite(IRout, LOW);
    delayTs(1);        
  } 
  
  digitalWrite(IRout, HIGH);
}

/*{
  digitalWrite(IRout,LOW);
  delayMicroseconds(8*bitTime);
  for (int i=0;i<8;i++) {
    digitalWrite(IRout,HIGH);  
    delayMicroseconds(bitTime);
    if ((command & 128) !=0) delayMicroseconds(3*bitTime);
    digitalWrite(IRout,LOW);
    delayMicroseconds(bitTime);
    command <<= 1;
  }
  digitalWrite(IRout,HIGH);
  delay(250); // Give a 1/4 sec before next
}*/
//IR end
//compass begin
void setupHMC5883L(){
  //Setup the HMC5883L, and check for errors
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println("*C1");
  //if(print_on) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println("*C2");
  //if(print_on) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}

float getHeading(){
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  return heading * RAD_TO_DEG; //radians to degrees
}
//compass end
//adxl begin
void setupADXL()
{
 adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625μs per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}

void adxlExample()
{
//Boring accelerometer stuff   
  int xa,ya,za;  
  adxl.readAccel(&xa, &ya, &za); //read the accelerometer values and store them in variables  x,y,z

  // Output x,y,z values - Commented out
  Serial.print("!a");
  Serial.print(xa);
  Serial.print(",");
  Serial.print(ya);
  Serial.print(",");
  Serial.println(za);

/*
  //Fun Stuff!    
  //read interrupts source and look for triggerd actions
  
  //getInterruptSource clears all triggered actions after returning value
  //so do not call again until you need to recheck for triggered actions
   byte interrupts = adxl.getInterruptSource();
  
  // freefall
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    Serial.println("!AF");//freefall
    //add code here to do when freefall is sensed
  } 
  
  //inactivity
  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
    Serial.println("!AI");//"inactivity");
     //add code here to do when inactivity is sensed
  }
  
  //activity
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    Serial.println("!AA");//"activity"); 
     //add code here to do when activity is sensed
  }
  
  //double tap
  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
    Serial.println("!AD");//double tap");
     //add code here to do when a 2X tap is sensed
  }
  
  //tap
  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
    Serial.println("!AT");//"tap");
     //add code here to do when a tap is sensed
  } 
*/
 
}
//adxl end
void baroEx()
{
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  //float atm = pressure / 101325; // "standard atmosphere"
  //float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 

  Serial.print("!b");
  Serial.print(temperature, 2); //display 2 decimal places in deg celcius
  Serial.print(",");

  //Serial.print("Pressure: ");
  Serial.println(pressure, 0); //whole number only. Pascal
  //Serial.println(",");

  //if(print_on) Serial.print("Standard Atmosphere: ");
  //if(print_on) Serial.println(atm, 4); //display 4 decimal places

  //if(print_on) Serial.print("Altitude: ");
  //Serial.println(altitude, 2); //display 2 decimal places
  //if(print_on) Serial.println(" M");

  //if(print_on) Serial.println();//line break
}

void gyroEx()
{
    getGyroValues();
  Serial.print("!g");
  Serial.print(x);

  Serial.print(",");
  Serial.print(y);

  Serial.print(",");
  Serial.println(z);
}

void compassEx()
{
    float heading = getHeading();
    Serial.print("!c");
    Serial.println(heading);
}
void servoZero()
{
  panServo.write((MAX_A+DIFF_A)/2);
  tiltServo.write((MAX_B+DIFF_B)/2);
}
/*
void panSweep()
{
  static int ang=(MAX_A+MIN_A)/2;
  static int dir=1;
  ang=ang+dir*4;
  if (ang>MAX_A) dir=-1;
  if (ang<MIN_A) dir=1;
  tiltServo.write(ang);
  delay(20);
}
*/
void setup()
{
	Wire.begin();
	Serial.begin(9600);
        while(!Serial){;}
	pinMode(sonarIn,INPUT);
	pinMode(sonarTrig,OUTPUT);
	pinMode(IRout,OUTPUT);
        digitalWrite(IRout,HIGH);
	panServo.attach(panPin);
	tiltServo.attach(tiltPin);
	Serial.println("starting up L3G4200D");
        setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
	bmp085Calibration();
        compass = HMC5883L(); //new instance of HMC5883L library
        setupHMC5883L(); //setup the HMC5883L
	setupADXL();
        delay(1500); //wait for the sensor to be ready 
	attachInterrupt(0,sonarDist,CHANGE);
        timer.setInterval(200,trigSonar);
        timer.setInterval(100,gyroEx);
        timer.setInterval(100,compassEx);
        timer.setInterval(1000,baroEx);
        timer.setInterval(200,printDist);
        servoZero();
}
byte hex2byte(byte hexAsc)
{
  //assuming all lower case
  if (hexAsc>='0' and hexAsc<='9'){
    return (hexAsc-'0');
  }else if (hexAsc>='a' and hexAsc<='f'){
    return (10+hexAsc-'a');
  }
  return 0;
}
void printDist()
{
  Serial.print("!d");
  Serial.println(distance);
}
void RSstop()
{
  RSSendCommand(0x8E);
  //if(print_on) 
  Serial.println("!STP");
}
void loop()
{
  byte inByte=0;
  static byte cmd=0;
  static byte panORtilt=0;
  static byte c1b,c2b,t1b,t2b;
  byte tmp=0;
  static byte state=0;
  timer.run();
  if (Serial.available()){
    inByte=Serial.read();
    switch(inByte)
    {
      case 'r':
      cmd=0;
      state=1;//robot command
      break;
      //case '\r':state=0;
      //break;
      case 's':
      cmd=1;
      state=2;//camera command
      break;
      default:
      tmp=hex2byte(inByte);
      switch(state){
        case 1://read hex command nibble1
        c1b=tmp;
        state=3;
        break;
        case 2://read pan tilt hex nibble
        panORtilt=(tmp)?1:0;//1=pan,0=tilt
        state=3;
        break;
        case 3:
        c2b=tmp;
        state=4;
        break;
        case 4:
        t1b=tmp;
        state=5;//read last time nibble
        break;
        case 5:
        t2b=tmp;
        //carry out command
        if (cmd){//camera
              //Serial.print(t1b*0x10+t2b);
              //Serial.println("deg");
              int ang=(t1b*0x10+t2b);
            if(panORtilt){
              if (ang>MAX_A)ang=MAX_A;
              panServo.write(ang+DIFF_A);
            }else{
              if (ang>MAX_B)ang=MAX_B;
              tiltServo.write(ang+DIFF_B);
            }
        }else{//robot
              RSSendCommand(c1b*0x10+c2b);
              if (t1b|t2b){
                //Serial.print((t1b*0x10+t2b)*10);
                //Serial.println(" ms");
                timer.setTimeout((t1b*0x10+t2b)*10,RSstop);
              }
        }
        state=0;
        break;
        default:
        break;
      }//switch
    }//switch
  }
  //if (done){if(print_on) Serial.print(distance);if(print_on) Serial.println(" cm");}
  //panSweep();
  adxlExample();
}
