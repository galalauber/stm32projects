/* MPU6050_light library for Arduino
 * 
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *            simplifications and corrections
 *          Tockn (github.com/tockn)
 *            initial author (v1.5.2)
 */

//#include "Arduino.h"
#include <math.h>
#include "../Inc/MPU6050_light_stm32.h"
#include "stm32l4xx_hal.h"

I2C_HandleTypeDef *HI2C;
float gyroXoffset, gyroYoffset, gyroZoffset;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
long preInterval;
float accCoef, gyroCoef;


void MPU6050_init(I2C_HandleTypeDef *hi2c){
  HI2C = hi2c;
  accCoef = 1.0-DEFAULT_GYRO_COEFF;
  gyroCoef = DEFAULT_GYRO_COEFF;

  MPU6050_begin();
}

void MPU6050_init2(I2C_HandleTypeDef *hi2c, float aC, float gC){
HI2C = hi2c;
  accCoef = aC;
  gyroCoef = gC;

  MPU6050_begin();
}

void MPU6050_begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  MPU6050_update();
  angleX = MPU6050_getAccAngleX();
  angleY = MPU6050_getAccAngleY();
  preInterval = HAL_GetTick();
}

void writeMPU6050(uint8_t reg, uint8_t data){
  uint8_t pacote[2];
  pacote[0] = reg;
  pacote[1] = data;

  if(HAL_I2C_Master_Transmit(HI2C, MPU6050_ADDR << 1, pacote, 2, 5) == HAL_ERROR) {
	  error_led();
  }
}

uint8_t readMPU6050(uint8_t reg) {
  uint8_t data;
  HAL_I2C_Master_Transmit(HI2C, (MPU6050_ADDR << 1) | 1, &reg, 1, 5);
  if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR << 1) | 1, &data, 1, 5) == HAL_ERROR) {
	  error_led();
  }

  return data;
}

uint8_t readMPU6050_n(uint8_t reg, uint8_t *data, uint8_t size) {

  HAL_I2C_Master_Transmit(HI2C, (MPU6050_ADDR << 1) | 1, &reg, 1, 5);
  if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR << 1) | 1, data, size, 5) == HAL_ERROR) {
	  error_led();
  }

  return 0;
}


void MPU6050_setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050_calcGyroOffsets(){
  float xyz[3] = {0,0,0};
  uint8_t offset[6];
  int16_t b;
  
  for(int i = 0; i < GYRO_OFFSET_NB_MES; i++) {
	  readMPU6050_n(0x43, offset, 6);
	for(int j=0;j<3;j++){
		b  = offset[j*2] << 8;
		b |= offset[j*2+1];
		xyz[j] += ((float)b) / GYRO_LSB_2_DEGSEC;
	}
  }
  gyroXoffset = xyz[0] / GYRO_OFFSET_NB_MES;
  gyroYoffset = xyz[1] / GYRO_OFFSET_NB_MES;
  gyroZoffset = xyz[2] / GYRO_OFFSET_NB_MES;
}

void MPU6050_update(){

uint8_t raw[14];
  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  readMPU6050_n(0x3B, raw, 14);

  for(int i=0;i<7;i++){
	rawData[i]  = raw[i*2] << 8;
    rawData[i] |= raw[i*2+1];
  }

  accX = ((float)rawData[0]) / ACC_LSB_2_G;
  accY = ((float)rawData[1]) / ACC_LSB_2_G;
  accZ = ((float)rawData[2]) / ACC_LSB_2_G;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / GYRO_LSB_2_DEGSEC - gyroXoffset;
  gyroY = ((float)rawData[5]) / GYRO_LSB_2_DEGSEC - gyroYoffset;
  gyroZ = ((float)rawData[6]) / GYRO_LSB_2_DEGSEC - gyroZoffset;
  
  float sgZ = (accZ>=0)-(accZ<0);
  angleAccX = atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG;
  angleAccY = - atan2(accX, sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;

  unsigned long Tnew = HAL_GetTick();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  angleX = (gyroCoef * (angleX + gyroX*dt)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY*dt)) + (accCoef * angleAccY);
  angleZ += gyroZ*dt;

}


float MPU6050_getTemp(){ return temp; };

float MPU6050_getAccX(){ return accX; };
float MPU6050_getAccY(){ return accY; };
float MPU6050_getAccZ(){ return accZ; };

float MPU6050_getGyroX(){ return gyroX; };
float MPU6050_getGyroY(){ return gyroY; };
float MPU6050_getGyroZ(){ return gyroZ; };

float MPU6050_getGyroXoffset(){ return gyroXoffset; };
float MPU6050_getGyroYoffset(){ return gyroYoffset; };
float MPU6050_getGyroZoffset(){ return gyroZoffset; };

float MPU6050_getAccAngleX(){ return angleAccX; };
float MPU6050_getAccAngleY(){ return angleAccY; };

float MPU6050_getAngleX(){ return angleX; };
float MPU6050_getAngleY(){ return angleY; };
float MPU6050_getAngleZ(){ return angleZ; };


void error_led() {
	while(1) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(200);
	}
}
