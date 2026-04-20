/**
 * @file   [FILENAME]
 * @author [AUTHOR]
 * @date   [DATE]
 *
 * [DESCRIPTION OF FILE/LIBRARY]
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>  // Para usar tipos estándar como uint8_t, int16_t

// ------------------------------------------------------
// ➤ Direcciones de registros internos del MPU6050
// ------------------------------------------------------

// Dirección del dispositivo (con AD0 = 0 → 0x68 << 1 = 0xD0 para escritura)
#define MPU6050_ADDR         0xD0

#define WHO_AM_I_REG         0x75  // Registro de identidad (debe devolver 0x68)
#define PWR_MGMT_1_REG       0x6B  // Registro para salir del modo de bajo consumo
#define CONFIG_REG           0x1A  // Registro de configuración (DLPF)
#define GYRO_CONFIG_REG      0x1B  // Registro de configuración del giroscopio
#define ACCEL_CONFIG_REG     0x1C  // Registro de configuración del acelerómetro
#define ACCEL_XOUT_H_REG     0x3B  // Dirección base de lectura del acelerómetro
#define GYRO_XOUT_H_REG      0x43  // Dirección base de lectura del giroscopio

// ------------------------------------------------------
// ➤ Offsets digitales para calibración (medidos en reposo)
// ------------------------------------------------------
// Se aplican para compensar el ruido o desvío de fábrica

#define OFFSET_AX  450
#define OFFSET_AY  450
#define OFFSET_AZ  450//20000  // en reposo el Z mide 1g → ≈ 16384 + margen

#define OFFSET_GX  450
#define OFFSET_GY  350
#define OFFSET_GZ  350

// ------------------------------------------------------
// ➤ Parámetros de escala
// ------------------------------------------------------

// Gravedad terrestre en m/s²
#define GRAVEDAD            9.81

// Factor de multiplicación para convertir a escala fija (2 decimales)
#define MULTIPLICADORFLOAT  100  // Ejemplo: 9.81 × 100 = 981

// ------------------------------------------------------
// ➤ Prototipos de funciones públicas
// ------------------------------------------------------

// Inicializa el sensor: saca del modo sleep y configura escala de medición
void mpu6050_Init(void);

//// Lee el acelerómetro, aplica offset y escala a m/s² ×100
//void mpu6050_Read_Accel(void);
//
//// Lee el giroscopio, aplica offset y escala a °/s ×100
//void mpu6050_Read_Gyro(void);

//Lectura de acelerómetro y giroscopio
char mpu6050_Read();


void mpu6050_Attach_MemReadDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

void mpu6050_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

void mpu6050_ADC_ConfCpltCallback(volatile uint8_t *PtrRx);

void mpu6050_Reset_State(void);

///**
// * @brief  Obtiene la última lectura de aceleración escalada.
// * @param  ax Pointer al entero donde se copiará ax_real.
// * @param  ay Pointer al entero donde se copiará ay_real.
// * @param  az Pointer al entero donde se copiará az_real.
// */
void mpu6050_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
////
/////**
//// * @brief  Obtiene la última lectura de velocidad angular escalada.
//// * @param  gx Pointer al entero donde se copiará gx_real.
//// * @param  gy Pointer al entero donde se copiará gy_real.
//// * @param  gz Pointer al entero donde se copiará gz_real.
//// */
//void mpu6050_GetGyro();
////




#endif /* INC_MPU6050_H_ */
