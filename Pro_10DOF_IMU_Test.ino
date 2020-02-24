

#include "Wire.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#define ODR_12HZ      0b00010000
#define ODR_26HZ      0b00100000
#define ODR_52HZ      0b00110000
#define ODR_104HZ     0b01000000
#define ODR_208HZ     0b01010000
#define ODR_416HZ     0b01100000
#define ODR_833HZ     0b01110000
#define ODR_1666HZ    0b10000000
#define ODR_3333HZ    0b10010000
#define ODR_6666HZ    0b10100000

#define ACCEL_2G      0b0000
#define ACCEL_4G      0b1000
#define ACCEL_8G      0b1100
#define ACCEL_16G     0b0100

#define GYRO_125DPS   0b0010
#define GYRO_250DPS   0b0000
#define GYRO_500DPS   0b0100
#define GYRO_1000DPS  0b1000
#define GYRO_2000DPS  0b1100

#define DEV_ADDR      0x6B
#define ACCEL_RANGE   ACCEL_4G
#define GYRO_RANGE    GYRO_1000DPS
#define UPDATE_RATE   400

#define INVERT_X_AXIS   true
#define INVERT_Y_AXIS   true
#define INVERT_Z_AXIS   false
//#define VERBOSE

#define LSB_PER_G     (float)(32768.0 / 4.0)
#define LSB_PER_DPS   (float)(32768.0 / 1000.0)
#define LSB_PER_C     (float)(256.0)


float axf, ayf, azf, gxf, gyf, gzf, tempf;
float yaw, pitch, roll;
int16_t ax, ay, az, gx, gy, gz, temp;


//Mahony filter;
Madgwick filter;


void setup(){

  Serial.begin(230400);
  Wire.begin();
  filter.begin(UPDATE_RATE);

  Serial.print("Device ID: ");
  Serial.println(readReg(0x0F), HEX);

  writeReg(0x10, ODR_416HZ | ACCEL_RANGE); 
  writeReg(0x11, ODR_416HZ | GYRO_RANGE); 
}



void loop(){

  uint32_t us = micros();

  readGyroRaw(gx, gy, gz);
  readAccelRaw(ax, ay, az);
  readTempRaw(temp); 

  getFloatAccel(ax, ay, az, axf, ayf, azf);
  getFloatGyro( gx, gy, gz, gxf, gyf, gzf);
  getFloatTemp(temp, tempf);

  filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);
  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();

#ifdef VERBOSE
  Serial.print("ax: "); Serial.print(ax); Serial.print("\t");
  Serial.print("ay: "); Serial.print(ay); Serial.print("\t");
  Serial.print("az: "); Serial.print(az); Serial.print("\t\t");

  Serial.print("gx: "); Serial.print(gx); Serial.print("\t");
  Serial.print("gy: "); Serial.print(gy); Serial.print("\t");
  Serial.print("gz: "); Serial.print(gz); Serial.print("\t");

  Serial.print("temp: "); Serial.print(temp); Serial.print("\t");
  Serial.println();
  
  Serial.print("axf: "); Serial.print(axf); Serial.print("\t");
  Serial.print("ayf: "); Serial.print(ayf); Serial.print("\t");
  Serial.print("azf: "); Serial.print(azf); Serial.print("\t");

  Serial.print("gxf: "); Serial.print(gxf); Serial.print("\t");
  Serial.print("gyf: "); Serial.print(gyf); Serial.print("\t");
  Serial.print("gzf: "); Serial.print(gzf); Serial.print("\t");

  Serial.print("temp: "); Serial.print(tempf); Serial.print("\t");
  Serial.println();
  
  Serial.print("Yaw: ");   Serial.print(yaw);   Serial.print("\t");
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print("\t");
  Serial.print("Roll: ");  Serial.print(roll);  Serial.print("\t");
#else

  Serial.print("Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
#endif

  

  delayMicroseconds((1000000/UPDATE_RATE)-(micros()-us));  
}


void getFloatAccel(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  if(INVERT_X_AXIS)
    xf = -(float)(x)/LSB_PER_G;
  else
    xf = (float)(x)/LSB_PER_G;
  if(INVERT_Y_AXIS)
    yf = -(float)(y)/LSB_PER_G;
  else
    yf = (float)(y)/LSB_PER_G;
  if(INVERT_Z_AXIS)
    zf = -(float)(z)/LSB_PER_G;
  else
    zf = (float)(z)/LSB_PER_G;
}

void getFloatGyro(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  if(INVERT_X_AXIS)
    xf = -(float)(x)/LSB_PER_DPS;
  else
    xf = (float)(x)/LSB_PER_DPS;
  if(INVERT_Y_AXIS)
    yf = -(float)(y)/LSB_PER_DPS;
  else
    yf = (float)(y)/LSB_PER_DPS;
  if(INVERT_Z_AXIS)
    zf = -(float)(z)/LSB_PER_DPS;
  else
    zf = (float)(z)/LSB_PER_DPS;
}

void getFloatTemp(int16_t t, float &tf){

  tf = 25.0 + (float)(t)/LSB_PER_C;
}

void readAccelRaw(int16_t &x, int16_t &y, int16_t &z){

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 6);
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

void readGyroRaw(int16_t &x, int16_t &y, int16_t &z){

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x22);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 6);
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

void readTempRaw(int16_t &t){

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(0x20);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 2);
  t = Wire.read() | (Wire.read() << 8);
}

uint8_t readReg(uint8_t reg){

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DEV_ADDR, 1);
  return Wire.read();
}

void writeReg(uint8_t reg, uint8_t val){

  Wire.beginTransmission(DEV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

