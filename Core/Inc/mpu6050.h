/**
 * @file   mpu6050.h
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 * @brief  Driver de configuración y adquisición para la Unidad de Medición Inercial MPU-6050.
 * @details Este módulo gestiona la inicialización de la IMU MPU-6050 sobre el bus I2C,
 *          configurando el muestreo interno a 200 Hz y el filtro digital pasabajos (DLPF) a 98 Hz.
 *          La lectura de los 14 registros continuos de aceleración y velocidad angular se realiza
 *          de forma no bloqueante mediante transferencias DMA directas a memoria.
 * @ingroup group_sensors
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @defgroup MPU6050_Registers Mapa de Registros del MPU-6050
 * @ingroup group_sensors
 * @{
 */
#define MPU6050_ADDR         0xD0  /*!< Dirección I2C del sensor en 8 bits (AD0 = 0 -> 0x68 << 1 = 0xD0) */
#define WHO_AM_I_REG         0x75  /*!< Registro de identidad del dispositivo (valor esperado: 0x68) */
#define PWR_MGMT_1_REG       0x6B  /*!< Registro de gestión de energía y modo sleep */
#define SMPLRT_DIV_REG       0x19  /*!< Divisor de la tasa de muestreo interno (Sample Rate Divider) */
#define CONFIG_REG           0x1A  /*!< Configuración del filtro digital pasa-bajos (DLPF) */
#define GYRO_CONFIG_REG      0x1B  /*!< Configuración de escala y rango del giróscopo */
#define ACCEL_CONFIG_REG     0x1C  /*!< Configuración de escala y rango del acelerómetro */
#define ACCEL_XOUT_H_REG     0x3B  /*!< Dirección base de lectura de aceleración (X_HIGH, 14 bytes continuos) */
#define GYRO_XOUT_H_REG      0x43  /*!< Dirección base de lectura de velocidad angular (X_HIGH) */
/** @} */

/**
 * @defgroup MPU6050_Calibration Offsets Digitales y Factores de Escala
 * @ingroup group_sensors
 * @{
 */
#define OFFSET_AX            450   /*!< Offset de calibración estática para acelerómetro eje X */
#define OFFSET_AY            450   /*!< Offset de calibración estática para acelerómetro eje Y */
#define OFFSET_AZ            450   /*!< Offset de calibración estática para acelerómetro eje Z */
#define OFFSET_GX            450   /*!< Offset de calibración estática para giróscopo eje X */
#define OFFSET_GY            350   /*!< Offset de calibración estática para giróscopo eje Y */
#define OFFSET_GZ            350   /*!< Offset de calibración estática para giróscopo eje Z */

#define MULTIPLICADORFLOAT   100   /*!< Factor de escala en punto fijo (x100) para dos cifras decimales enteras */
/** @} */

/**
 * @brief Inicializa el MPU-6050 con los parámetros de funcionamiento del robot.
 * @details Despierta al sensor sacándolo del modo sleep (PWR_MGMT_1 = 0), configura el DLPF
 *          a 98 Hz (CONFIG = 0x02), establece la tasa de muestreo interno a 200 Hz (SMPLRT_DIV = 4),
 *          y fija los rangos dinámicos en $\pm 2g$ para aceleración y $\pm 250^\circ/\text{s}$ para rotación.
 */
void mpu6050_Init(void);

/**
 * @brief Ejecuta el ciclo de lectura no bloqueante de 14 bytes por DMA.
 * @details Implementa una máquina de dos estados:
 *          - Estado 1: Dispara la lectura DMA de 14 bytes contiguos desde `ACCEL_XOUT_H_REG`.
 *          - Estado 2: Recompone los pares de bytes recibidos (Big-Endian) en valores con signo de 16 bits.
 * @retval 1 Lectura completa, datos actualizados y listos.
 * @retval 0 Transferencia DMA en curso o bus no disponible.
 */
char mpu6050_Read(void);

/**
 * @brief Vincula la función de callback que indica la finalización de recepción DMA.
 * @param[in] PtrRx Puntero a la bandera volátil que se activa en la interrupción DMA Rx.
 */
void mpu6050_ADC_ConfCpltCallback(volatile uint8_t *PtrRx);

/**
 * @brief Conecta el puntero a la función de lectura por DMA del bus I2C del microcontrolador.
 * @param[in] PtrRx Puntero a la función despachadora de lectura I2C DMA implementada en el main.
 */
void mpu6050_Attach_MemReadDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Conecta el puntero a la función de escritura en registros I2C.
 * @param[in] PtrRx Puntero a la función de encolado de escrituras I2C.
 */
void mpu6050_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));

/**
 * @brief Reinicia el estado interno del secuenciador de lectura DMA del sensor.
 */
void mpu6050_Reset_State(void);

/**
 * @brief Copia los últimos datos crudos de 16 bits leídos del acelerómetro y giróscopo.
 * @param[out] ax Puntero donde se almacenará la aceleración cruda en eje X.
 * @param[out] ay Puntero donde se almacenará la aceleración cruda en eje Y.
 * @param[out] az Puntero donde se almacenará la aceleración cruda en eje Z.
 * @param[out] gx Puntero donde se almacenará la velocidad angular cruda en eje X.
 * @param[out] gy Puntero donde se almacenará la velocidad angular cruda en eje Y.
 * @param[out] gz Puntero donde se almacenará la velocidad angular cruda en eje Z.
 */
void mpu6050_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* INC_MPU6050_H_ */
