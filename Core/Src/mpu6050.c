/*
 * mpu6050.c
 *
 *  Created on: May 24, 2025
 *      Author: gonza
 */

#include "MPU6050.h"
#include "main.h"
#include <stdlib.h>

static uint8_t *mpu6050_RxCplt = NULL;
static void (*memWrite)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type) = NULL;
static void (*memReadDMA)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type) = NULL;

// Variables convertidas a unidades físicas con escala ×100 (2 decimales fijos)
int16_t ax_real; // Aceleración en X [centésimas de m/s²]
int16_t ay_real;
int16_t az_real;

int16_t gx_real; // Velocidad angular en X [centésimas de grados/segundo]
int16_t gy_real;
int16_t gz_real;

// Variables RAW leídas directamente del sensor (int16_t = complemento a dos)
static int32_t ax, ay, az, gx, gy, gz;
static uint8_t mpu_state = 1;

void mpu6050_ADC_ConfCpltCallback(volatile uint8_t *PtrRx){
	mpu6050_RxCplt = (uint8_t *)PtrRx;
}

void mpu6050_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type)){
	memWrite = PtrRx;
}

void mpu6050_Attach_MemReadDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type)){
	memReadDMA = PtrRx;
}


//Send init command
void mpu6050_WriteData(uint8_t *byte, uint8_t type) {
	memWrite(MPU6050_ADDR, byte, 1, type);
	//HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

void mpu6050_ReadDataDMA(uint8_t* buffer, size_t size, uint8_t type) {
	memReadDMA(MPU6050_ADDR, buffer, size, type);
	//HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data,14, 1000);
}


void mpu6050_Init(void)
{
    uint8_t data;

    // Salir del modo de bajo consumo (modo sleep)
    // Escritura en el registro PWR_MGMT_1 (0x6B)
    data = 0x00;
    //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
    mpu6050_WriteData(&data, PWR_MGMT_1_REG);

    // Habilitar el Digital Low Pass Filter (DLPF) a ~44Hz
	data = 0x04;
	mpu6050_WriteData(&data, CONFIG_REG);

    // Configurar acelerómetro con rango ±2g (registro ACCEL_CONFIG = 0x1C, valor 0x00)
    data = 0x00;
    //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
    mpu6050_WriteData(&data, ACCEL_CONFIG_REG);

    //Configurar giroscopio con rango ±250°/s (registro GYRO_CONFIG = 0x1B, valor 0x00)
    //data = 0x00;
    //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
    mpu6050_WriteData(&data, GYRO_CONFIG_REG);

}

void mpu6050_Reset_State(void) {
	mpu_state = 1;
	if (mpu6050_RxCplt != NULL) {
		*mpu6050_RxCplt = 0;
	}
}

char mpu6050_Read(void)
{
	static uint8_t Rec_Data[14];

	if (*mpu6050_RxCplt || mpu_state == 1) {
		*mpu6050_RxCplt = 0;  // Reset completion flag
		switch (mpu_state) {
		case 1:
			mpu_state = 2;
			// Leer 14 bytes desde ACCEL_XOUT_H (registro 0x3B)
			mpu6050_ReadDataDMA(Rec_Data, 14, ACCEL_XOUT_H_REG);
			break;
		case 2:
			mpu_state = 1;
			// Combinar bytes altos y bajos
			ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
			ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
			az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

			gx = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
			gy = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
			gz = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

			return 1;
			break;
		}
	}
	return 0;
}
void mpu6050_GetData(int16_t *ax_out, int16_t *ay_out, int16_t *az_out, int16_t *gx_out, int16_t *gy_out, int16_t *gz_out) {
    // Pasamos las variables RAW estáticas directamente al main
    if (ax_out) *ax_out = ax;
    if (ay_out) *ay_out = ay;
    if (az_out) *az_out = az;

    if (gx_out) *gx_out = gx;
    if (gy_out) *gy_out = gy;
    if (gz_out) *gz_out = gz;
}

