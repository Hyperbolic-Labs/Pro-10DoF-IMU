
#include "Wire.h"


#define DEV_ADDR      0x6B
#define LSB_PER_G     (float)(32768.0 / 2.0)
#define LSB_PER_DPS   (float)(32768.0 / 500.0)
#define LSB_PER_C     (float)(256.0)

float axf, ayf, azf, gxf, gyf, gzf, tempf;
int16_t ax, ay, az, gx, gy, gz, temp;


void setup(){

  Serial.begin(115200);
  Wire.begin();


  Serial.print("Device ID: ");
  Serial.println(readReg(0x0F), HEX);

  writeReg(0x10, 0x60); //CTRL1_XL, 416HZ, +-2G
  writeReg(0x11, 0x60); //CTRL1_XL, 416HZ, +-250dps  
}



void loop(){

  readGyroRaw(gx, gy, gz);
  readAccelRaw(ax, ay, az);
  readTempRaw(temp);
  
  Serial.print("ax: "); Serial.print(ax); Serial.print("\t");
  Serial.print("ay: "); Serial.print(ay); Serial.print("\t");
  Serial.print("az: "); Serial.print(az); Serial.print("\t\t");

  Serial.print("gx: "); Serial.print(gx); Serial.print("\t");
  Serial.print("gy: "); Serial.print(gy); Serial.print("\t");
  Serial.print("gz: "); Serial.print(gz); Serial.print("\t");

  Serial.print("temp: "); Serial.print(temp); Serial.print("\t");
  Serial.println();

  getFloatAccel(ax, ay, az, axf, ayf, azf);
  getFloatGyro( gx, gy, gz, gxf, gyf, gzf);
  getFloatTemp(temp, tempf);

  Serial.print("axf: "); Serial.print(axf); Serial.print("\t");
  Serial.print("ayf: "); Serial.print(ayf); Serial.print("\t");
  Serial.print("azf: "); Serial.print(azf); Serial.print("\t");

  Serial.print("gxf: "); Serial.print(gxf); Serial.print("\t");
  Serial.print("gyf: "); Serial.print(gyf); Serial.print("\t");
  Serial.print("gzf: "); Serial.print(gzf); Serial.print("\t");

  Serial.print("temp: "); Serial.print(tempf); Serial.print("\t");
  Serial.println();

  delay(1000);
  
}


void getFloatAccel(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  xf = (float)(x)/LSB_PER_G;
  yf = (float)(y)/LSB_PER_G;
  zf = (float)(z)/LSB_PER_G;
}

void getFloatGyro(int16_t x, int16_t y, int16_t z, float &xf, float &yf, float &zf){

  xf = (float)(x)/LSB_PER_DPS;
  yf = (float)(y)/LSB_PER_DPS;
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
