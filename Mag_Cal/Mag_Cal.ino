
#include "Wire.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"



#define MAG_ODR_1HZ       0b1001
#define MAG_ODR_10HZ      0b1010
#define MAG_ODR_20HZ      0b1011
#define MAG_ODR_50HZ      0b1100
#define MAG_ODR_100HZ     0b1101
#define MAG_ODR_200HZ     0b1110
#define MAG_ODR_1000HZ    0b1111

#define MAG_ADDR      0x30
#define MAG_RATE      MAG_ODR_200HZ

#define LSB_PER_GAUSS   4096


float xmax, ymax, zmax;
float xmin, ymin, zmin;
float xoffs, yoffs, zoffs;
float xscale, yscale, zscale;

float xalign[3] = {0, 0, 0};
float yalign[3] = {0, 0, 0};
float zalign[3] = {0, 0, 0};

float x, y, z;
int16_t rx, ry, rz;



void setup(){

  Serial.begin(115200);
  Wire.begin();

  writeReg(MAG_ADDR, 0x09, 0b00100001);
  writeReg(MAG_ADDR, 0x0A, 0b00000000);
  writeReg(MAG_ADDR, 0x0B, 0b10110000);//| MAG_ODR_200HZ); //set/reset pulse every 100 samples
}



void loop(){

  if(Serial.available()){
   
    if(Serial.read() == 'x')
      printStats();
          
    while(Serial.available())
      Serial.read();
  }

  readMagRaw(rx, ry, rz);
  getFloatMag(rx, ry, rz, x, y, z);

  if(abs(x) > xalign[0]){
    xalign[0] = x;
    xalign[1] = y;
    xalign[2] = z;
  }
  if(abs(y) > yalign[1]){
    yalign[0] = x;
    yalign[1] = y;
    yalign[2] = z;
  }
  if(abs(x) > zalign[2]){
    zalign[0] = x;
    zalign[1] = y;
    zalign[2] = z;
  }


  bool flag = false;

  if(x < xmin){
    xmin = x;
    flag = true;
  }
  else if(x > xmax){
    xmax = x;
    flag = true;
  }

  if(y < ymin){
    ymin = y;
    flag = true;
  }
  else if(y > ymax){
    ymax = y;
    flag = true;
  }

  if(z < zmin){
    zmin = z;
    flag = true;
  }
  else if(z > zmax){
    zmax = z;
    flag = true;
  }

  if(flag){
    Serial.print("Xmin: "); Serial.print(xmin); 
    Serial.print("\tYmin: "); Serial.print(ymin); 
    Serial.print("\tZmin: "); Serial.println(zmin); 
  
    Serial.print("Xmax: "); Serial.print(xmax); 
    Serial.print("\tYmax: "); Serial.print(ymax); 
    Serial.print("\tZmax: "); Serial.println(zmax);
  }

  delay(50);

}



void printStats(){

  Serial.print("Xmin: "); Serial.print(xmin); 
  Serial.print("\tYmin: "); Serial.print(ymin); 
  Serial.print("\tZmin: "); Serial.println(zmin); 

  Serial.print("Xmax: "); Serial.print(xmax); 
  Serial.print("\tYmax: "); Serial.print(ymax); 
  Serial.print("\tZmax: "); Serial.println(zmax);

  Serial.print("Xoffs: "); Serial.print(xoffs);
  Serial.print("Yoffs: "); Serial.print(yoffs);
  Serial.print("Zoffs: "); Serial.println(zoffs);

  Serial.print("XScale: "); Serial.print(xscale);
  Serial.print("YScale: "); Serial.print(yscale);
  Serial.print("ZScale: "); Serial.println(zscale);
}


void getFloatMag(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){
  
    xf = (float)(x)/LSB_PER_GAUSS;  
    yf = (float)(y)/LSB_PER_GAUSS;  
    zf = (float)(z)/LSB_PER_GAUSS;  
}



void readMagRaw(int16_t &x, int16_t &y, int16_t &z){

  writeReg(MAG_ADDR, 0x09, 0b00001000);
  delay(5);
  writeReg(MAG_ADDR, 0x09, 0b00010000);  
  delay(5);
  writeReg(MAG_ADDR, 0x09, 0b00000001);  
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);
  x = ((Wire.read() << 8) | Wire.read()) - 32768;
  y = ((Wire.read() << 8) | Wire.read()) - 32768;
  z = ((Wire.read() << 8) | Wire.read()) + 32768;
////  x = Wire.read() | (Wire.read() << 8);
////  y = Wire.read() | (Wire.read() << 8);
////  z = Wire.read() | (Wire.read() << 8);
  Serial.print("X: "); Serial.println(x);
  Serial.print("Y: "); Serial.println(y);
  Serial.print("Z: "); Serial.println(z);

//  Wire.beginTransmission(MAG_ADDR);
//  Wire.write(0x00);
//  Wire.endTransmission();
//  Wire.requestFrom(MAG_ADDR, 2);
//  Serial.print("X: ");
//  Serial.print(Wire.read(), BIN);
//  Serial.print("\t");
//  Serial.println(Wire.read(), BIN);
//  Wire.beginTransmission(MAG_ADDR);
//  Wire.write(0x02);
//  Wire.endTransmission();
//  Wire.requestFrom(MAG_ADDR, 2);
//  Serial.print("X: ");
//  Serial.print(Wire.read(), BIN);
//  Serial.print("\t");
//  Serial.println(Wire.read(), BIN);
//  Wire.beginTransmission(MAG_ADDR);
//  Wire.write(0x04);
//  Wire.endTransmission();
//  Wire.requestFrom(MAG_ADDR, 2);
//  Serial.print("X: ");
//  Serial.print(Wire.read(), BIN);
//  Serial.print("\t");
//  Serial.println(Wire.read(), BIN);
}



uint8_t readReg(uint8_t addr, uint8_t reg){

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  return Wire.read();
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val){

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
