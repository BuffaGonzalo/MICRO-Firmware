/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "util.h"
#include "img.h"
#include "fonts.h"

#include "ssd1306.h"
#include "mpu6050.h"
#include "esp01.h"

#include <stdio.h>
#include <unerPrtcl.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//time
#define	TO10MS				40
#define SSD1306_MAXVER		64
#define	SSD1306_MAXHOR		128
#define	SSD1306_MAXADC		30
#define	SSD1306_MINADC		60
#define SSD1306_FSTCOL		0
#define	SSD1306_SNDCOL		40
#define	SSD1306_TRDCOL		85

#define SSD1306				0
#define MPU6050				1
#define I2CSIZE				16

#define ON					1
#define OFF					0

#define TIM3CP				9999


#define DEBOUNCE             4
#define NUMBUTTONS           1
#define LIMIT                0x0F

#define T100MS				100
#define T1000MS				1000


//banderas
#define ALLFLAGS          	myFlags.bytes
#define IS10MS				myFlags.bits.bit0
#define IS20MS				myFlags.bits.bit1
#define IS100MS				myFlags.bits.bit2

#define HEARTBEAT			myFlags.bits.bit3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t heartBeatMask[] = {0xFFFFFFFF, 0x00000000, 0x55555555, 0x1, 0x2010080, 0x5F, 0x5, 0x28140A00, 0x15F, 0x15, 0x2A150A08, 0x55F};

const char firmware[] = "EX100923v01\n";

//Tiempos
uint8_t time10ms;
uint8_t tmo100ms = 10;
uint8_t tmo20ms = 2;
//ADC
uint16_t adcData[8], adcDataTx[8]; //ADC

//Comunicación
_sTx USBTx, USBRx;
volatile uint8_t buffUSBTx[RXBUFSIZE];
volatile uint8_t buffUSBRx[TXBUFSIZE];
uint8_t nBytesTx = 0;
_uWord myWord;
//Control
volatile _uFlag myFlags;
//Variables pantalla
volatile uint8_t ssd1306_TxCplt = 0;
volatile uint8_t mpu6050_RxCplt = 0;
//mpu6050
int16_t ax=0, ay=0, az=0;
int16_t gx=0, gy=0, gz=0;


//i2c
uint8_t Pila[I2CSIZE] = {};
uint8_t i2cIndex = 0;

typedef enum{
	IDLE = 0,
	DATA_DISPLAY = 1,
	UPD_DISPLAY = 2,
	ONMPU = 3
}_eDMA;

_eDMA myDMA;

uint8_t tmo100 = 5;
uint8_t IS100 = 0;

uint8_t chnl_1, chnl_2, chnl_3, chnl_4; ////REVISAR CAPAZ QUE SE PUEDE USAR uint8_t

uint8_t hbIndex = 0;

//Wifi
uint8_t timerUDP = 0;
uint8_t byteUART_ESP01;
_sESP01Handle esp01Handler;

_sButton myButton;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//USB-Serial Communication
void USBTask();
void USBRxData(uint8_t *buf, uint32_t len);
void decodeCommand(_sTx *dataRx, _sTx *dataTx);
//Time functions
void do10ms();
void do100ms();

//Others
void heartBeatTask();

//Display
//void displayTask();
void ssd1306Data();
void displayMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void displayMemWriteDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
//MPU6050
void mpuMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
void mpuMemReadDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type);
//i2C
void i2cTask();
//boton
/**
 * @brief Función con la cual inicializamos los botones
 * @param _sButton Estructura con los datos del boton
 * @param buttonFunction Puntero a funcion
*/
void initButton(_sButton *button);

/**
 * @brief Función utilizada para actualizar la MEF de los botones
 * @param _sButton Estructura con los datos
*/
uint8_t updateMefTask(_sButton *button);

void buttonTask();

void buttonTimeout10ms(_sButton *button);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//Revisar tiempos de ejecución de esta sección con respecto a la mpu
	for (int i = 0; i < 8; i++) {
		adcDataTx[i] = adcData[i];
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){ //Pantalla
	ssd1306_TxCplt = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){ //MPU
	mpu6050_RxCplt = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) { //250us
		time10ms++;
		if (time10ms == TO10MS) {
			time10ms = 0;
			IS10MS = TRUE;
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData, 8);
	}

	if (htim->Instance == TIM2) { //20ms
		Pila[i2cIndex] = MPU6050;
		i2cIndex++;
		i2cIndex&=(I2CSIZE-1);
		tmo100--;
		if(!tmo100){
			tmo100=5;
			Pila[i2cIndex] = SSD1306;
			i2cIndex++;
			i2cIndex&=(I2CSIZE-1);
		}
	}
}


void USBRxData(uint8_t *buf, uint32_t len) { //Recibimos datos -> Enviamos datos

	for (uint8_t nBytesRx = 0; nBytesRx < len; nBytesRx++) { //Guardamos los datos en el buffer de recepcion
		USBRx.buff[USBRx.indexW++] = buf[nBytesRx];
		USBRx.indexW &= USBRx.mask;
	}

}

void USBTask() {

	if(USBRx.indexR != USBRx.indexW){
		uint8_t sendBuffer[TXBUFSIZE];

		if (unerPrtcl_DecodeHeader(&USBRx)){
			decodeCommand(&USBRx, &USBTx);

		for (uint8_t i = 0; i < USBTx.bytes; i++) { //Paso limpio, error ultima posición
			sendBuffer[i] = USBTx.buff[USBTx.indexData++];
			USBTx.indexData &= USBTx.mask;
		}

		if(ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED)
			ESP01_Send(sendBuffer, 0, USBTx.bytes, TXBUFSIZE);
		else
			CDC_Transmit_FS(sendBuffer, USBTx.bytes);
		}
	}
}

void decodeCommand(_sTx *dataRx, _sTx *dataTx) {

	switch (dataRx->buff[dataRx->indexData]) {
	case ALIVE:
		unerPrtcl_PutHeaderOnTx(dataTx, ALIVE, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case FIRMWARE:
		unerPrtcl_PutHeaderOnTx(dataTx, FIRMWARE, 13);
		unerPrtcl_PutStrOntx(dataTx, firmware);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETMPU:
		unerPrtcl_PutHeaderOnTx(dataTx, GETMPU, 13);
		myWord.i16[0] = ax;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = ay;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = az;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gx;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gy;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.i16[0] = gz;
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETADC:
		unerPrtcl_PutHeaderOnTx(dataTx, GETADC, 17);
		myWord.ui16[0] = adcDataTx[0];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[1];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[2];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[3];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[4];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[5];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[6];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		myWord.ui16[0] = adcDataTx[7];
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[0]);
		unerPrtcl_PutByteOnTx(dataTx, myWord.ui8[1]);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case SETPWM:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWM, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[2]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[3]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        chnl_1 = myWord.i32;
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[2]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[3]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        chnl_2 = myWord.i32;
		break;
	default:
		unerPrtcl_PutHeaderOnTx(dataTx, (_eCmd) dataRx->buff[dataRx->indexData], 2);
		unerPrtcl_PutByteOnTx(dataTx, UNKNOWN);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
}

void do10ms() {

	if (IS10MS) {
		IS10MS = FALSE;
		tmo100ms--;
		tmo20ms--;
		ESP01_Timeout10ms();
		buttonTimeout10ms(&myButton);

		if (!tmo20ms) {
			tmo20ms = 2;
			IS20MS = TRUE;
		}
		if (!tmo100ms) {
			tmo100ms = 10;

			timerUDP++;
			if(timerUDP>=10){
				timerUDP=0;

				// Preparamos un paquete ALIVE (0xF0)
				_sTx paqueteAlive;
				uint8_t bufferTx[32];

				// Armamos cabecera UNER
				unerPrtcl_PutHeaderOnTx(&paqueteAlive, ALIVE, 1);
				// Ponemos un dato dummy (ej. 0x00)
				unerPrtcl_PutByteOnTx(&paqueteAlive, 0x00);
				// Checksum
				unerPrtcl_PutByteOnTx(&paqueteAlive, paqueteAlive.chk);

				// Pasamos el paquete a un buffer lineal
				for(int i=0; i<paqueteAlive.bytes; i++){
					bufferTx[i] = paqueteAlive.buff[paqueteAlive.indexData++];
					paqueteAlive.indexData &= paqueteAlive.mask;
				}

				// Enviamos el saludo a la IP de la PC (Configurada en el Init)
				ESP01_Send(bufferTx, 0, paqueteAlive.bytes, 100);

			}

			IS100MS = TRUE;
			heartBeatTask();

		}
	}
}

void do100ms(){
	if(IS100MS){
		IS100MS=FALSE;
	}
}

void heartBeatTask() {
	static uint8_t times = 0;

	if (~heartBeatMask[hbIndex] & (1 << times)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}

	times++;
	times &= 31;
}

void displayMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write(&hi2c2, address , type, 1, data, size, HAL_MAX_DELAY);
}

void displayMemWriteDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write_DMA(&hi2c2, address , type, 1, data, size);
}

void mpuMemWrite(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Write(&hi2c2, address , type, 1, data, size, HAL_MAX_DELAY);
}

void mpuMemReadDMA(uint8_t address, uint8_t *data, uint8_t size, uint8_t type){
	HAL_I2C_Mem_Read_DMA(&hi2c2, address , type, 1, data, size);
}

void ssd1306Data() {
	char data[8];
	uint8_t y = 0, x = 2;

	ssd1306_Fill(White);

	ssd1306_FillRectangle(30, 0, 32, 64, Black);

	ssd1306_FillRectangle(0, 20, 128, 22, Black);

	ssd1306_FillRectangle(80, 0, 82, 64, Black);

	x = SSD1306_SNDCOL;
	y = 0;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "ACC");
	ssd1306_WriteString(data, Font_11x18, Black);
	x = SSD1306_TRDCOL;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "GYR");
	ssd1306_WriteString(data, Font_11x18, Black);

	x = SSD1306_SNDCOL;
	y += 25;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", ax);
	ssd1306_WriteString(data, Font_7x10, Black);
	x = SSD1306_TRDCOL;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", gx);
	ssd1306_WriteString(data, Font_7x10, Black);

	x = SSD1306_SNDCOL;
	y += 12;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", ay);
	ssd1306_WriteString(data, Font_7x10, Black);
	x = SSD1306_TRDCOL;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", gy);
	ssd1306_WriteString(data, Font_7x10, Black);

	x = SSD1306_SNDCOL;
	y += 12;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", az);
	ssd1306_WriteString(data, Font_7x10, Black);
	x = SSD1306_TRDCOL;
	ssd1306_SetCursor(x, y);
	snprintf(data, sizeof(data), "%d", gz);
	ssd1306_WriteString(data, Font_7x10, Black);

	ssd1306_Line(3, 60, 3,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[0] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(6, 60, 6,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[1] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(9, 60, 9,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[2] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(12, 60, 12,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[3] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(15, 60, 15,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[4] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(18, 60, 18,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[5] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(21, 60, 21,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[6] * SSD1306_MAXADC) / 4090), Black);
	ssd1306_Line(24, 60, 24,
			(SSD1306_MINADC - ((uint32_t)adcDataTx[7] * SSD1306_MAXADC) / 4090), Black);
}

void i2cTask() {
	static uint8_t i = IDLE;
	static uint8_t j = 0;

	switch (i) {
	case IDLE:
		if (j == i2cIndex) { //Sale por que no hay nuevos elementos
			break;
		}

		if (Pila[j]) { //mpu6050
			i = ONMPU;
			j++;
			j &= (I2CSIZE - 1);
			break;
		}
		if (!Pila[j]) {
			i = DATA_DISPLAY;
			j++;
			j &= (I2CSIZE - 1);
		}
		break;
	case DATA_DISPLAY:
		ssd1306Data();
		i = UPD_DISPLAY;
		break;
	case UPD_DISPLAY:
		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			if (ssd1306_UpdateScreenDMA()) {

				ssd1306_TxCplt = FALSE;
				i = IDLE;
			}
		}
		break;
	case ONMPU:
		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			if (mpu6050_Read()) {

				mpu6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
				mpu6050_RxCplt = FALSE;
				i = IDLE;
			}
		}
		break;
	default:
		i = IDLE;
		break;
	}
}

void PWM_Control(){

	// +-------------------------------------------------------+
	// | TABLA DE ESTADOS - CONTROLADOR L9110S                 |
	// +------------+------------+--------------+--------------+
	// | Entrada IA | Entrada IB | Salida Motor | Estado       |
	// +------------+------------+--------------+--------------+
	// |    LOW     |    LOW     |     OFF      | Frenado/Stop |
	// |    HIGH    |    LOW     |    AVANCE    | Giro Horario |
	// |    LOW     |    HIGH    |  RETROCESO   | Giro Antihor.|
	// |    HIGH    |    HIGH    |     OFF      | Frenado/Stop |
	// +------------+------------+--------------+--------------+
	// | * Nota: Las salidas quedan en estado "flotante" si las |
	// |   entradas son iguales (ambas HIGH o ambas LOW).      |
	// +-------------------------------------------------------+

	  // Calcula CCRx = period * percent / 100
	  uint16_t lPulse1  = TIM3CP * chnl_1 / 100UL;
	  uint16_t lPulse3 = TIM3CP * chnl_3 / 100UL;

	  uint16_t rPulse2 = TIM3CP * chnl_2 / 100UL;
	  uint16_t rPulse4 = TIM3CP * chnl_4 / 100UL;


	//Rueda izquierda
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,lPulse1);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,lPulse3);

	//Rueda derecha
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,rPulse2);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,rPulse4);

}


void CHPD_Control(uint8_t state)
{
    /* Assuming CH_PD is on GPIOB Pin 0 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Send one byte out over UART to ESP-01.
 * @param  byte  The data byte to transmit.
 */

int USART_SendByte(uint8_t byte)
{
    /* Assuming huart2 is configured for the ESP01 */
    if(HAL_UART_Transmit_IT(&huart1, &byte, 1) == HAL_OK)
    	return 1;
    else
    	return 0;
}

/**
 * @brief  Forward one received byte into the ESP01 driver’s rx buffer.
 * @param  byte  The byte received from UART ISR.
 */
void FeedRxBuf(uint8_t byte)
{
    ESP01_WriteRX(byte);
}


void DebugESP01_To_USB(const char *msg) {
    // strlen requiere #include <string.h>
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}


// Esta función la llama el driver cuando tiene un byte de datos UDP limpio
void WiFi_Data_Callback(uint8_t byte)
{
    // AQUÍ guardamos el dato en tu estructura de protocolo para que USBTask lo procese
    USBRx.buff[USBRx.indexW++] = byte;
    USBRx.indexW &= USBRx.mask;
}

//BOTONES
void initButton(_sButton *button){
    button->currentState = BUTTON_UP;
    button->stateInput = NO_EVENT;
    button->isPressed = FALSE;
    button->time = 0;
}

uint8_t updateMefTask(_sButton *button){
    uint8_t action=FALSE;

    switch (button->currentState){
        case BUTTON_UP:
            if(button->stateInput==PRESSED)
                button->currentState=BUTTON_FALLING;
        break;
        case BUTTON_FALLING:
            if(button->stateInput==PRESSED){
                button->currentState=BUTTON_DOWN;
                button->isPressed=TRUE;
            }else{
                button->currentState=BUTTON_UP;
            }
        break;
        case BUTTON_DOWN:
            if(button->stateInput==NOT_PRESSED)
                button->currentState=BUTTON_RISING;
        break;
        case BUTTON_RISING:
            if(button->stateInput==NOT_PRESSED){
                button->currentState=BUTTON_UP;
                button->isPressed = FALSE;
                action=TRUE;
            }else{
                button->currentState=BUTTON_DOWN;
            }
        break;
        default:
            button->currentState=BUTTON_UP;
        break;
    }
    return action;
}


void buttonTimeout10ms(_sButton *button){
    static uint8_t timeToDebounce= 0;

    if(button->isPressed){
        button->time += 10;
    } else{
    	button->time = 0;
    }

    // ACTUALIZAMOS EL ESTADO DE LOS PULSADORES
    if(timeToDebounce > DEBOUNCE){
        timeToDebounce = 0;

		if(HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_RESET) {
			myButton.stateInput = PRESSED;
		} else {
			myButton.stateInput = NOT_PRESSED;
		}

    } else {
        timeToDebounce++;
    }
}

void buttonTask(_sButton *button){
	if((!button->isPressed) && (button->time > T100MS) && (button->time<T1000MS)){
		hbIndex = 1;
		button->time = 0; //limpiamos el valor para que no pueda volver a entrar en esta parte
	}
	if((!button->isPressed) && (button->time > T1000MS)){
		hbIndex = 0;
		button->time = 0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CDC_Attach_Rx(USBRxData); //Attach a la función que tenia en el .C

  	HAL_TIM_Base_Start_IT(&htim1);
  	HAL_TIM_Base_Start_IT(&htim2);
  	HAL_TIM_Base_Start_IT(&htim3);

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //Apagamos el LED

  	//Display

  	ssd1306_ADC_ConfCpltCallback(&ssd1306_TxCplt);
  	ssd1306_Attach_MemWrite(displayMemWrite);
  	ssd1306_Attach_MemWriteDMA(displayMemWriteDMA);
  	ssd1306_Init();

  	//mpu6050

  	mpu6050_ADC_ConfCpltCallback(&mpu6050_RxCplt);
  	mpu6050_Attach_MemWrite(mpuMemWrite);
  	mpu6050_Attach_MemReadDMA(mpuMemReadDMA);
  	mpu6050_Init();

  	//esp01

  	esp01Handler.DoCHPD = CHPD_Control;
 	esp01Handler.WriteUSARTByte = USART_SendByte;
 	esp01Handler.WriteByteToBufRX = WiFi_Data_Callback;

  	ESP01_Init(&esp01Handler);

  	ESP01_AttachDebugStr(DebugESP01_To_USB);

  	HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1); //non blocking

  	//ESP01_SetWIFI("FCAL","fcalconcordia.06-2019");
  	//ESP01_SetWIFI("Buffa Family 2.4GHz", "-NixieBulb2022-");
  	ESP01_SetWIFI("ARPAMOVILE","12345678");
  	ESP01_StartUDP("192.168.154.68", 30010, 30001);

  	//Inicializacion de protocolo
  	unerPrtcl_Init(&USBRx, &USBTx, buffUSBRx, buffUSBTx);

  	//Variables
  	ALLFLAGS = RESET;
  	//reversa
  	chnl_1=5;
  	chnl_3=5;
  	//adelante
  	chnl_2=0;
  	chnl_4=0;

    //INICIALIZAMOS BOTONES
    initButton(&myButton);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	do10ms();
	ESP01_Task();
	USBTask();

	PWM_Control();
	i2cTask();

	buttonTask(&myButton);
	updateMefTask(&myButton);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP01_EN_GPIO_Port, ESP01_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW0_Pin */
  GPIO_InitStruct.Pin = SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP01_EN_Pin */
  GPIO_InitStruct.Pin = ESP01_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP01_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Verificamos que la interrupción venga del UART1 (ESP01)
    if (huart->Instance == USART1)
    {
        // 1. Alimentamos al driver ESP01
        FeedRxBuf(byteUART_ESP01);

        // 2. Volvemos a activar la escucha para el siguiente byte
        HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
