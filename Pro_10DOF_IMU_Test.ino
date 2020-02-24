
#include "Wire.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#define IMU_ODR_12HZ      0b00010000
#define IMU_ODR_26HZ      0b00100000
#define IMU_ODR_52HZ      0b00110000
#define IMU_ODR_104HZ     0b01000000
#define IMU_ODR_208HZ     0b01010000
#define IMU_ODR_416HZ     0b01100000
#define IMU_ODR_833HZ     0b01110000
#define IMU_ODR_1666HZ    0b10000000
#define IMU_ODR_3333HZ    0b10010000
#define IMU_ODR_6666HZ    0b10100000

#define MAG_ODR_1HZ       0b1001
#define MAG_ODR_10HZ      0b1010
#define MAG_ODR_20HZ      0b1011
#define MAG_ODR_50HZ      0b1100
#define MAG_ODR_100HZ     0b1101
#define MAG_ODR_200HZ     0b1110
#define MAG_ODR_1000HZ    0b1111


#define ACCEL_2G      0b0000
#define ACCEL_4G      0b1000
#define ACCEL_8G      0b1100
#define ACCEL_16G     0b0100

#define GYRO_125DPS   0b0010
#define GYRO_250DPS   0b0000
#define GYRO_500DPS   0b0100
#define GYRO_1000DPS  0b1000
#define GYRO_2000DPS  0b1100

#define IMU_ADDR      0x6B
#define MAG_ADDR      0x30

#define ACCEL_RANGE   ACCEL_4G
#define GYRO_RANGE    GYRO_1000DPS
#define IMU_RATE      IMU_ODR_416HZ
#define MAG_RATE      MAG_ODR_200HZ
#define MAG_HIGH_RES_MODE


#define FILTER_RATE   100

#define IMU_INVERT_X_AXIS   true
#define IMU_INVERT_Y_AXIS   true
#define IMU_INVERT_Z_AXIS   false
#define MAG_INVERT_X_AXIS   false
#define MAG_INVERT_Y_AXIS   false
#define MAG_INVERT_Z_AXIS   true
//#define VERBOSE

#define LSB_PER_G       (float)(32768.0 / 4.0)
#define LSB_PER_DPS     (float)(32768.0 / 1000.0)
#define LSB_PER_C       256
#define LSB_PER_GAUSS   4096


float axf, ayf, azf, gxf, gyf, gzf, mxf, myf, mzf, tempf;
float yaw, pitch, roll;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz, temp;


//Mahony filter;
Madgwick filter;


void setup(){

  pinMode(13, OUTPUT);

  Serial.begin(230400);
  Wire.begin();
  filter.begin(FILTER_RATE);

  writeReg(IMU_ADDR, 0x10, IMU_ODR_416HZ | ACCEL_RANGE); 
  writeReg(IMU_ADDR, 0x11, IMU_ODR_416HZ | GYRO_RANGE); 

  writeReg(MAG_ADDR, 0x0B, MAG_ODR_200HZ);
}



void loop(){

  uint32_t us = micros();
  digitalWrite(13, LOW);

  readGyroRaw( gx, gy, gz);
  readAccelRaw(ax, ay, az);
  readMagRaw(  mx, my, mz);
  readIMUTempRaw(temp); 

  getFloatAccel(ax, ay, az, axf, ayf, azf);
  getFloatGyro( gx, gy, gz, gxf, gyf, gzf);
  getFloatMag(  mx, my, mz, mxf, myf, mzf);
  getIMUFloatTemp(temp, tempf);

  filter.update(gxf, gyf, gzf, axf, ayf, azf, mxf, myf, mzf);
  //filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);
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

  Serial.print("mx: "); Serial.print(mx); Serial.print("\t");
  Serial.print("my: "); Serial.print(my); Serial.print("\t");
  Serial.print("mz: "); Serial.print(mz); Serial.print("\t");

  Serial.print("temp: "); Serial.print(temp); Serial.print("\t");
  Serial.println();
  
  Serial.print("axf: "); Serial.print(axf); Serial.print("\t");
  Serial.print("ayf: "); Serial.print(ayf); Serial.print("\t");
  Serial.print("azf: "); Serial.print(azf); Serial.print("\t");

  Serial.print("gxf: "); Serial.print(gxf); Serial.print("\t");
  Serial.print("gyf: "); Serial.print(gyf); Serial.print("\t");
  Serial.print("gzf: "); Serial.print(gzf); Serial.print("\t");

  Serial.print("mxf: "); Serial.print(mxf); Serial.print("\t");
  Serial.print("myf: "); Serial.print(myf); Serial.print("\t");
  Serial.print("mzf: "); Serial.print(mzf); Serial.print("\t");

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

  
  digitalWrite(13, HIGH);
  delayMicroseconds((1000000/IMU_RATE)-(micros()-us));  
}


void getFloatAccel(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  if(IMU_INVERT_X_AXIS)
    xf = -(float)(x)/LSB_PER_G;
  else
    xf = (float)(x)/LSB_PER_G;
  if(IMU_INVERT_Y_AXIS)
    yf = -(float)(y)/LSB_PER_G;
  else
    yf = (float)(y)/LSB_PER_G;
  if(IMU_INVERT_Z_AXIS)
    zf = -(float)(z)/LSB_PER_G;
  else
    zf = (float)(z)/LSB_PER_G;
}

void getFloatGyro(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  if(IMU_INVERT_X_AXIS)
    xf = -(float)(x)/LSB_PER_DPS;
  else
    xf = (float)(x)/LSB_PER_DPS;
  if(IMU_INVERT_Y_AXIS)
    yf = -(float)(y)/LSB_PER_DPS;
  else
    yf = (float)(y)/LSB_PER_DPS;
  if(IMU_INVERT_Z_AXIS)
    zf = -(float)(z)/LSB_PER_DPS;
  else
    zf = (float)(z)/LSB_PER_DPS;
}


void getFloatMag(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  if(MAG_INVERT_X_AXIS)
    xf = -(float)(x)/LSB_PER_GAUSS;
  else
    xf = (float)(x)/LSB_PER_GAUSS;
  if(MAG_INVERT_Y_AXIS)
    yf = -(float)(y)/LSB_PER_GAUSS;
  else
    yf = (float)(y)/LSB_PER_GAUSS;
  if(MAG_INVERT_Z_AXIS)
    zf = -(float)(z)/LSB_PER_GAUSS;
  else
    zf = (float)(z)/LSB_PER_GAUSS;  
}

void getIMUFloatTemp(int16_t t, float &tf){

  tf = 25.0 + (float)(t)/LSB_PER_C;
}

void readAccelRaw(int16_t &x, int16_t &y, int16_t &z){

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 6);
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

void readGyroRaw(int16_t &x, int16_t &y, int16_t &z){

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x22);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 6);
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

void readMagRaw(int16_t &x, int16_t &y, int16_t &z){

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);
//  x = (Wire.read() << 8) | Wire.read();
//  y = (Wire.read() << 8) | Wire.read();
//  z = (Wire.read() << 8) | Wire.read();
  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);
}

void readIMUTempRaw(int16_t &t){

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x20);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 2);
  t = Wire.read() | (Wire.read() << 8);
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
