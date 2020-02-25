
/*
 * Mag_Cal.ino
 * 
 * Hyperbolic Labs 02/25/20
 * 
 * Calibration test for MMC5983MA magnetometer sensor. This program 
 * calculates offset coefficients for hard-iron compensation as well 
 * as sensor spherical gain compensation for soft-iron proximity effects.
 * To get the most accurate calibration coefficients, rotate the sensor 
 * in a figure-eight pattern. 
 * 
 */


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


float xmax, ymax, zmax;         //max recorded field for each axis
float xmin, ymin, zmin;         //min recorded field for each axis
float xoffs, yoffs, zoffs;      //calculated offset for each axis
float xscale, yscale, zscale;   //calculated gain multiplier for each axis

float xalign[3] = {0, 0, 0};    //sensor reading most aligned with x axis
float yalign[3] = {0, 0, 0};    //sensor reading most aligned with y axis
float zalign[3] = {0, 0, 0};    //sensor reading most aligned with z axis

float x, y, z;                  //temporary variables
int16_t rx, ry, rz;             



void setup(){

  Serial.begin(115200);
  Wire.begin();

  writeReg(MAG_ADDR, 0x0A, 0b00000000);     //enable all axes, 100Hz bandwidth (8msec conversion)
  writeReg(MAG_ADDR, 0x0B, 0b10110000);     //set/reset pulse every 100 samples
  writeReg(MAG_ADDR, 0x09, 0b00100001);     //enable auto set/reset feature, send measurement request 
}



void loop(){

  if(Serial.available()){       //check if character has been sent
   
    if(Serial.read() == 'x'){   //if character is 'x', conclude calibration routine and print results
      printStats();
      while(true);
    }
          
    while(Serial.available())   //clear out any remaining characters
      Serial.read();
  }

  readMagRaw(rx, ry, rz);             //get raw 16-bit magnetometer values for each axis
  getFloatMag(rx, ry, rz, x, y, z);   //convert raw measurements to gauss


  //check if current sample is aligned with x, y or z axis

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


  //check for any min/max conditions

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

  if(flag){   //if min/max has changed, then print to Serial monitor
    
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

  writeReg(MAG_ADDR, 0x09, 0b00001000);   //perform set pulse
  delay(5);
  writeReg(MAG_ADDR, 0x09, 0b00010000);   //perform reset pulse
  delay(5);
  writeReg(MAG_ADDR, 0x09, 0b00000001);   //request new ADC measurement
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);                       //start at X axis MSB
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);          //read until Z axis LSB
  x = ((Wire.read() << 8) | Wire.read()) - 32768;   //16-bit unsigned format, shifted
  y = ((Wire.read() << 8) | Wire.read()) - 32768;   //16-bit unsigned format, shifted
  z = ((Wire.read() << 8) | Wire.read()) + 32768;   //Z axis seems to be inverted, not sure why

//  Serial.print("X: "); Serial.println(x);
//  Serial.print("Y: "); Serial.println(y);
//  Serial.print("Z: "); Serial.println(z);
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
