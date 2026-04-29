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
#include "wiregfx.h"

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

#define TIM3CP				9999//2500 // Anteriormente el valor era 9999, modificado para tener mas precision

//WebServer - solo necesitamos la primera linea del request HTTP (~80 bytes max)
#define HTTP_BUF_SIZE   	128

#define DEBOUNCE             4
#define NUMBUTTONS           1
#define LIMIT                0x0F

#define PID_SCALE_FACTOR	100 //factor de escala, evitar decimales o AFP (aritmetica de punto fijo)
#define ANG50				50*PID_SCALE_FACTOR
#define ANG45				45*PID_SCALE_FACTOR
#define ANG20				20*PID_SCALE_FACTOR

#define CTRLSPEED			10
//Acelerometro
#define RADTOGRAD			5730
// Giroscopio
#define GYRO_SENSITIVITY    131 // Para configuración de +/- 250 grados/s
#define DT_MS               20
// Filtro Complementario
#define ALPHA_GYRO          98     // 98% de confianza al giroscopio
#define ALPHA_ACC           2      // 2% de confianza al acelerómetro

#define AZ_MIN_VALID  		4000
#define OUTPUT_DEADBAND 	150 // outputs menores a esto → motores apagados

//#define MIN_PWM 			28  // Mínimo para que la rueda empiece a girar, valor de 6 para un TIM3CP de 9999
//#define	MAX_PWM 			25  // Máximo permitido para correcciones

////seguidor de linea
#define SCALE_LINE			1000
#define LINE_THRESHOLD		1500//1800
#define IR_WHITE			1500  // Lectura ADC base sobre superficie blanca (~1800-2000)
#define LINE_LOST_PHASE0  	35
#define LINE_LOST_PHASE1  	70
#define TURNPWM_LEFT		900
#define TURNPWM_RIGHT		830

// Variables del esquivador
#define OBS_TURNPWM_LEFT    1050   // PWM de rotación izquierda
#define OBS_TURNPWM_RIGHT   1000   // PWM de rotación derecha


#define T100MS				100
#define T1000MS				1000

//WIFI
#define NUM_KNOWN_NETWORKS  (sizeof(knownNetworks) / sizeof(knownNetworks[0]))
#define NETWORK_MAX_RETRIES 2            /* Intentos por cada red antes de saltar */
#define SCANTIME			3000//1500

//banderas
#define ALLFLAGS          	myFlags.bytes
#define IS10MS				myFlags.bits.bit0
#define IS20MS				myFlags.bits.bit1
#define IS100MS				myFlags.bits.bit2

#define HEARTBEAT			myFlags.bits.bit3

#define RUN_PID             myFlags.bits.bit4

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
uint32_t heartBeatMask[] = {0x55555555, 0xFFFFFFFF, 0x00000000, 0x1, 0x2010080, 0x5F, 0x5, 0x28140A00, 0x15F, 0x15, 0x2A150A08, 0x55F};

const char firmware[] = "EX100923v01\n";

//Tiempos
uint8_t time10ms;
uint8_t tmo100ms = 10;
uint8_t tmo20ms = 2;
//ADC
uint16_t adcData[8], adcDataTx[8]; //ADC

//Comunicación
_sComm USBTx, USBRx;
_sComm WiFiTx, WiFiRx;
volatile uint8_t buffUSBTx[RXBUFSIZE];
volatile uint8_t buffUSBRx[TXBUFSIZE];
volatile uint8_t buffWiFiTx[RXBUFSIZE];
volatile uint8_t buffWiFiRx[TXBUFSIZE];
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

_eDMA myDMA;

uint8_t tmo100 = 5;
uint8_t IS100 = 0;

uint8_t chnl_1, chnl_2, chnl_3, chnl_4; ////REVISAR CAPAZ QUE SE PUEDE USAR uint8_t

// Variables crudas de 16 bits para el PWM de cada motor (0 a 9999)
uint16_t lPulse1 = 0;
uint16_t rPulse2 = 0;
uint16_t lPulse3 = 0;
uint16_t rPulse4 = 0;

uint8_t hbIndex = 0;

//Wifi
uint8_t timerUDP = 0;
uint8_t udpSilenceCounter = 5; // Comienza en 5 para enviar ALIVEs desde el arranque. Se resetea al recibir comandos WiFi.
uint8_t byteUART_ESP01;
_sESP01Handle esp01Handler;
_sButton myButton;

/* ---- WEBSERVER ---- */
static char    httpBuf[HTTP_BUF_SIZE];
static uint8_t httpBufIdx = 0;          /* 0..N = acumulando | 0xFF = primera linea lista */
static uint8_t isWebserverMode = 0;     /* 1 = modo webserver activo */

/* Buffer unico compartido para todas las respuestas HTTP.
 * Nunca se usan sendHTMLForm y sendHTTPOKPage al mismo tiempo,
 * por lo que un solo buffer alcanza. Tamaño = respuesta mas grande (formulario). */
static uint8_t httpTxBuf[340];

/* ---- Destino UDP (guardado desde el formulario web) ---- */
static char    udpTargetIP[16]   = "172.23.190.89";
static uint16_t udpTargetPort    = 30010;
static uint8_t  udpReadyToStart  = 0;

/* ---- TABLA DE REDES CONOCIDAS ---- */
typedef struct {
	const char *ssid;
	const char *password;
	const char *targetIP;
} _sWiFiNetwork;

static const _sWiFiNetwork knownNetworks[] = {
	{ "FCAL",    "fcalconcordia.06-2019",    "172.23.190.89"  },
	{ "ARPANET", "1969-Apolo_11-2022",       "192.168.0.13"   },
	{ "SA04",    "12345678",                "10.93.92.213"   },
};

static uint8_t  currentNetworkIdx  = 0;   /* Indice de la red que estamos intentando */
static uint8_t  networkScanActive  = 0;   /* 1 = estamos escaneando redes */
static uint8_t  networkRetryCount  = 0;   /* Reintentos por red antes de pasar a la siguiente */
static uint16_t networkScanTimer = SCANTIME;

//////VARIABLES DE LOS SISTEMAS DE CONTROL//////
//variables internas
int32_t acc_angle_hr = 0;
int32_t gyro_delta_hr = 0;
int32_t current_angle_hr = 0;
int32_t error = 0;
int32_t last_error = 0;
int32_t derivative = 0;
int32_t integral = 0;
int32_t output = 0;
int32_t current_angle = 0; // Escala x100 (ej: 150 = 1.5 grados)
//filtros de acc
int32_t ax_filt = 0;
int32_t az_filt = 0;

//Variables externas PID
int16_t Kp_stable = 75;
int16_t Kd_stable = 8;
int16_t Ki_stable = 0;
// Pasan a ser de 16 bits. minPWM centrado en 735.
uint16_t maxPWM    = 9999;
uint16_t minPWM_left  = 1100;
uint16_t minPWM_right = 1060;

// Offsets que garantizan exactamente los 70 puntos de diferencia
int16_t offset_left = 8;
int16_t offset_right = 20; //70

int32_t setpoint = -225; // Angulo unico de trabajo (x100 = 0.5°), ajustable por SETLINECTRL

// Variables del Control de Línea
int16_t Kp_line = 125; // Proporcional para reacción del volante (60 es un buen inicio)
int16_t Kq_line = 25; // Derivativo para suavizar el giro y evitar oscilaciones
int32_t sum_sensors = 0;
int32_t error_linea = 0;
int32_t abs_error = 0;
int32_t linear_term = 0;
int32_t quad_term = 0;
int32_t turn_offset = 0;
int32_t last_line_error = 0;
// Variables escaladas (custom_turn maneja valores escalados x100)
int16_t custom_turn = 20; // 15000 / 100 = 150 de PWM real para giro en búsqueda
int16_t attack_setpoint = -1000; // Inclinación de 2.0° para un avance muy sutil
int16_t brake_angle_div = 4; //Valor menor aumenta la velocidad en curvas

// Fases y tiempos de búsqueda activa en LINE_LOST.
uint8_t line_lost_timer = 0;
uint8_t line_lost_phase = 0;

// Ángulos usando enteros de 8 bits (0 a 255 representa un giro completo)
uint8_t angle_x = 0;
uint8_t angle_y = 0;
uint8_t angle_z = 0;

// Ángulos usando enteros de 8 bits
uint8_t angle_xz = 0; // Rotación 3D clásica
uint8_t angle_yz = 0; // Rotación 3D clásica
uint8_t angle_xw = 0; // Rotación 4D (La magia del hipercubo)

// --- Estado del seguidor de línea ---
typedef enum {
    LINE_SEARCHING,  // Todo blanco
    LINE_FOLLOWING,  // Seguimiento normal
	LINE_LOST,
    LINE_CROSS       // Todo negro (Intersección en T)
} _eLineState;

_eLineState lineState = LINE_SEARCHING;

//Esquivar objeto
// --- Estado del esquivador de objetos ---
typedef enum {
    OBS_IDLE,          // Seguimiento de línea normal, sin obstáculo
    OBS_STOP,          // Obstáculo detectado: para en el lugar
    OBS_ROTATE_1,      // Rota 90° hacia el lado libre
    OBS_SLIDE,         // Avanza paralelo al objeto manteniendo distancia lateral
    OBS_ROTATE_2,      // Rota 90° al terminar el objeto (sensor lateral < 400)
    OBS_FORWARD,       // Avanza manteniendo distancia lateral buscando la línea
    OBS_ROTATE_3,      // Rota -90° al encontrar fin de pared lateral
    OBS_REJOIN         // Avanza buscando la línea con sensor central
} _eObsState;

_eObsState obsState = OBS_IDLE;


uint8_t  obs_timer       = 0;
uint8_t  obs_turn_dir    = 0;   // 1 = gira derecha, 0 = gira izquierda
int16_t  obs_side_sensor = 0;   // Sensor lateral activo durante el deslizamiento

uint16_t obs_detect_dist = 1000;   // Distancia de detección frontal (ADC)
uint16_t obs_lost_dist =  400;   // Distancia mínima lateral: objeto terminó
uint16_t obs_side_dist = 1000;   // Distancia de referencia lateral a mantener
uint16_t obs_stop_cycles =  10;   // Ciclos de pausa antes de rotar (200ms)
uint16_t obs_rotate_cycles =  20;   // Ciclos de rotación de 90° (~400ms, ajustable)

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
void decodeCommand(_sComm *dataRx, _sComm *dataTx);
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

/* ---- WEBSERVER ---- */
void sendHTMLForm(uint8_t connID);
void sendHTTPOKPage(uint8_t connID);
void parseHTTPGetParams(const char *httpReq, char *ssid, char *pass, char *ip, uint16_t *port);
void httpTask(void);
void OnESP01ChangeState(_eESP01STATUS state);


//PID
void PID_ControlTask(void);

void ssd1306_DrawCube(void);

void ssd1306_DrawTesseract(void);

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

void COMMTask(_sComm *dataRx, _sComm *dataTx, uint8_t source) {

	if (dataRx->indexR != dataRx->indexW) {
		uint8_t sendBuffer[TXBUFSIZE];

		if (unerPrtcl_DecodeHeader(dataRx)) {

			// Si recibimos un comando válido por WiFi, la PC está conectada.
			// Resetear el contador de silencio para suprimir los ALIVEs autónomos.
			if (source == WIFI) {
				udpSilenceCounter = 0;
			}

			decodeCommand(dataRx, dataTx);

			for (uint8_t i = 0; i < dataTx->nBytes; i++) { //Paso limpio, error ultima posición
				sendBuffer[i] = dataTx->buff[dataTx->indexData++];
				dataTx->indexData &= dataTx->mask;
			}

			if(source)
				ESP01_Send(ESP01_GetLastConnID(), sendBuffer, 0, dataTx->nBytes, TXBUFSIZE);
			else
				CDC_Transmit_FS(sendBuffer, dataTx->nBytes);
		}
	}
}

void decodeCommand(_sComm *dataRx, _sComm *dataTx) {

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
	case SETPWML:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWML, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        lPulse1 = myWord.ui16[0];
		break;
	case SETPWMR:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMR, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        rPulse2 = myWord.ui16[0];
		break;
	case SETBALANCEKP:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKP, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Kp_stable = myWord.i16[0];
		break;
	case SETBALANCEKD:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKD, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Kd_stable = myWord.i16[0];
		break;
	case SETBALANCEKI:
        unerPrtcl_PutHeaderOnTx(dataTx, SETBALANCEKI, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        Ki_stable = myWord.i16[0];
		break;
	case SETPWMMINR:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMMINR, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        minPWM_right = myWord.ui16[0];
		break;
	case SETPWMMINL:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMMINL, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        minPWM_left = myWord.ui16[0];
		break;
	case SETSETPOINT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSETPOINT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.i8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.i8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		setpoint = (int32_t) myWord.i16[0];
		break;
	case SETBKANG:
		unerPrtcl_PutHeaderOnTx(dataTx, SETBKANG, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		brake_angle_div = myWord.i16[0];
		break;
	case GETINTERNALDATA:
		// Estructura simplificada para sincronización de parámetros (38 bytes de datos + 1 chk)
		unerPrtcl_PutHeaderOnTx(dataTx, GETINTERNALDATA, 39);

		// 1. Bloque PID Balancín (10 bytes: Kp, Ki, Kd, Max, Min)
		int16_t pid_bal[5] = { Kp_stable, Ki_stable, Kd_stable, (int16_t)minPWM_right, (int16_t)minPWM_left};
		for (int i = 0; i < 5; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (pid_bal[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_bal[i] >> 8) & 0xFF));
		}

		// 2. Setpoint (4 bytes - int32)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (setpoint & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 16) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 24) & 0xFF));

		// 3. Bloque Seguimiento y Offsets (14 bytes: KpL, KdL, OffL, OffR, Turn, Attack, Brake)
		int16_t params_ext[7] = { Kp_line, Kq_line, offset_left, offset_right, custom_turn, attack_setpoint, brake_angle_div };
		for (int i = 0; i < 7; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_ext[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_ext[i] >> 8) & 0xFF));
		}

		// 4. Bloque Esquivador de Obstáculos (10 bytes: Front, Side, Lost, Stop, Rotate)
		uint16_t params_obs[5] = { obs_detect_dist, obs_side_dist, obs_lost_dist, obs_stop_cycles, obs_rotate_cycles };
		for (int i = 0; i < 5; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_obs[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_obs[i] >> 8) & 0xFF));
		}

		// Checksum final
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETPIDBALANCE:
			// Tamaño: 1(cmd) + 4 variables * 4 bytes = 17 bytes
			unerPrtcl_PutHeaderOnTx(dataTx, GETPIDBALANCE, 17);

			int32_t pid_telemetry[4] = { error, integral, derivative, output };

			for (int i = 0; i < 4; i++) {
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (pid_telemetry[i] & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 8) & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 16) & 0xFF));
				unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_telemetry[i] >> 24) & 0xFF));
			}

			unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
			break;
	case SETLINEKP:
        unerPrtcl_PutHeaderOnTx(dataTx, SETLINEKP, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK);
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        Kp_line = myWord.i16[0];
        break;
	case SETLINEKD:
        unerPrtcl_PutHeaderOnTx(dataTx, SETLINEKD, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK);
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
        Kq_line = myWord.i16[0];
        break;
	case SETOFFSETL:
		unerPrtcl_PutHeaderOnTx(dataTx, SETOFFSETL, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		offset_left = myWord.i16[0];
		break;
	case SETOFFSETR:
		unerPrtcl_PutHeaderOnTx(dataTx, SETOFFSETR, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		offset_right = myWord.i16[0];
		break;
	case SETCUSTOMTURN:
		unerPrtcl_PutHeaderOnTx(dataTx, SETCUSTOMTURN, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		custom_turn = myWord.i16[0];
		break;
	case SETSPEED:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSPEED, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		attack_setpoint = myWord.i16[0];
		break;
	case SETFRONTDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_detect_dist = myWord.ui16[0];
		break;
	case SETSIDEDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSIDEDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_side_dist = myWord.ui16[0];
		break;
	case SETLOSTDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETLOSTDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_lost_dist = myWord.ui16[0];
		break;
	case SETSTOPCYCLES:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSTOPCYCLES, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_stop_cycles = myWord.ui16[0];
		break;
	case SETROTATECYCLES:
		unerPrtcl_PutHeaderOnTx(dataTx, SETROTATECYCLES, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_rotate_cycles = myWord.ui16[0];
		break;
	default:
		unerPrtcl_PutHeaderOnTx(dataTx, (_eCmd) dataRx->buff[dataRx->indexData],
				2);
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

		/* Lógica de escaneo autónomo y cíclico de redes */
		if (networkScanActive) {
			if (networkScanTimer > 0) {
				networkScanTimer--;
			} else {
				/* Se acabó el tiempo (15 segs). Pasamos a la siguiente red en la lista */
				currentNetworkIdx++;
				if (currentNetworkIdx >= NUM_KNOWN_NETWORKS) {
					currentNetworkIdx = 0; /* Volvemos al inicio de la lista */
				}

				networkScanTimer = SCANTIME; /* Reiniciamos la paciencia: 15 segundos */

				/* Forzamos al ESP01 a probar la nueva red */
				ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid,
						knownNetworks[currentNetworkIdx].password);
			}
		}

		if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
			uint32_t er = huart1.Instance->SR;
			uint32_t dr = huart1.Instance->DR;
			(void) er;
			(void) dr;
			huart1.RxState = HAL_UART_STATE_READY;
			HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
		}

		if (!tmo20ms) {
			tmo20ms = 2;
			IS20MS = TRUE;
		}
		if (!tmo100ms) {
			tmo100ms = 10;

			IS100MS = TRUE;
			heartBeatTask();

			timerUDP++;
			if (timerUDP >= 10) { //Entrar cada 1000ms o 1s
				timerUDP = 0;

				// Incrementar contador de silencio WiFi (saturar en 5)
				if (udpSilenceCounter < 5)
					udpSilenceCounter++;

				/* Enviar ALIVE solo si:
				 * 1. UDP conectado y no en modo webserver
				 * 2. La PC NO está comunicándose activamente (silencio > 3s)
				 * Cuando la PC envía GETMPU/GETADC, udpSilenceCounter se resetea
				 * en COMMTask, suprimiendo los ALIVEs automáticamente.
				 */
				if(!isWebserverMode && ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED
					&& udpSilenceCounter >= 5){
					static uint8_t bufferTx[9] = { 'U', 'N', 'E', 'R', 0x03, ':', ALIVE, ACK, 0x98 };
					ESP01_Send(0, bufferTx, 0, 9, TXBUFSIZE);
				}
			}
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
	static uint32_t mpu_timeout = 0; // NUEVO: Contador de paciencia

	switch (i) {
	case IDLE:
		if (j == i2cIndex) {
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
		//WIREGFX_DrawCube();
		//WIREGFX_DrawTesseract();
		//WIREGFX_Graphics_DrawPyramid();
		//WIREGFX_Graphics_DrawSphere();
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
		// 1. Si el hardware I2C detecta un error eléctrico, forzamos un reinicio inmediato
		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE) {
			mpu_timeout = 1000;
		}

		// 2. Intentamos leer el sensor
		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			if (mpu6050_Read()) {
				mpu6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
				mpu6050_RxCplt = FALSE;
				RUN_PID = TRUE;
				mpu_timeout = 0; // Lectura exitosa: el sensor vive, reseteamos timeout
				i = IDLE;
			} else {
				// Está esperando que el DMA conteste
				mpu_timeout++;
			}
		} else {
			// El bus I2C está BUSY (Ocupado/Trancado)
			mpu_timeout++;
		}

		// 3. --- EL DESFIBRILADOR ---
		// Si pasó mucho tiempo atascado esperando el DMA o en estado BUSY
		if (mpu_timeout > 500) {
			// Apagamos el hardware I2C para limpiar los registros corruptos
			HAL_I2C_DeInit(&hi2c2);

			// Le damos tiempo a los voltajes de los cables para estabilizarse
			HAL_Delay(1);

			// Lo volvemos a encender
			HAL_I2C_Init(&hi2c2);

			// Reconfiguramos los registros del MPU y reseteamos la máquina de estados
			mpu6050_Init();
			mpu6050_Reset_State();

			mpu_timeout = 0;
			i = IDLE; // Volvemos al bucle principal
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
//	  lPulse1  = TIM3CP * chnl_1 / 100UL;
//	  lPulse3 = TIM3CP * chnl_3 / 100UL;
//
//	  rPulse2 = TIM3CP * chnl_2 / 100UL;
//	  rPulse4 = TIM3CP * chnl_4 / 100UL;


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

// Esta función la llama el driver cuando tiene un byte de datos limpio
void WiFi_Data_Callback(uint8_t byte)
{
    if(isWebserverMode){
        /*
         * Solo capturamos la PRIMERA linea del request HTTP (hasta \r\n).
         * Ahi esta todo lo que necesitamos: "GET /set?ssid=X&pass=Y HTTP/1.1"
         * El resto del header se descarta. Buffer: 128 bytes en lugar de ~512.
         *
         * httpBufIdx:
         *   0..N  = acumulando la primera linea
         *   0xFF  = primera linea completa, lista para que httpTask() la procese
         */
        if(httpBufIdx == 0xFF)
            return; /* primera linea ya capturada, descartar el resto */

        if(httpBufIdx < HTTP_BUF_SIZE - 1){
            httpBuf[httpBufIdx++] = (char)byte;
            httpBuf[httpBufIdx]   = '\0';

            /* Detectar fin de primera linea: \r\n */
            if(httpBufIdx >= 2 &&
               httpBuf[httpBufIdx-2] == '\r' &&
               httpBuf[httpBufIdx-1] == '\n'){
                httpBufIdx = 0xFF; /* marcar como listo */
            }
        } else {
            /* Buffer lleno sin \r\n: request corrupto, resetear */
            httpBufIdx = 0;
            httpBuf[0] = '\0';
        }
    } else {
        /* Modo station UDP/TCP: protocolo UNER */
        WiFiRx.buff[WiFiRx.indexW++] = byte;
        WiFiRx.indexW &= WiFiRx.mask;
    }
}

/* ============================================================
 *  WEBSERVER - Funciones HTTP
 * ============================================================ */

/**
 * @brief Envia el formulario HTML con campos SSID, PASS, IP y Puerto
 */
void sendHTMLForm(uint8_t connID)
{
    const char *header = "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
    const char *body   = "<!DOCTYPE html><html><body>"
                         "<form action=/set method=GET>"
                         "SSID:<input name=ssid><br>"
                         "PASS:<input name=pass type=password><br>"
                         "IP PC:<input name=ip value=192.168.0.1><br>"
                         "Puerto:<input name=port value=30010><br>"
                         "<input type=submit value=Conectar>"
                         "</form></body></html>";

    uint16_t len = (uint16_t)snprintf((char*)httpTxBuf, sizeof(httpTxBuf), "%s%s", header, body);
    ESP01_Send(connID, httpTxBuf, 0, len, sizeof(httpTxBuf));
}

/**
 * @brief Envia pagina de confirmacion al navegador
 */
void sendHTTPOKPage(uint8_t connID)
{
    const char *header = "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
    const char *body   = "<!DOCTYPE html><html><body>"
                         "<h2>Conectando...</h2>"
                         "<p>El dispositivo se conectara a la red indicada.</p>"
                         "</body></html>";

    uint16_t len = (uint16_t)snprintf((char*)httpTxBuf, sizeof(httpTxBuf), "%s%s", header, body);
    ESP01_Send(connID, httpTxBuf, 0, len, sizeof(httpTxBuf));
}

/**
 * @brief Extrae ssid, pass, ip y port del GET /set?ssid=X&pass=Y&ip=Z&port=W
 *        Decodificacion basica: '+' → espacio
 */
void parseHTTPGetParams(const char *httpReq, char *ssid, char *pass, char *ip, uint16_t *port)
{
    const char *p;
    uint8_t i;

    ssid[0] = '\0';
    pass[0] = '\0';
    ip[0]   = '\0';
    *port   = 30010; /* valor por defecto */

    /* ---- SSID ---- */
    p = strstr(httpReq, "ssid=");
    if(p){ p += 5;
        for(i=0; i<63 && *p && *p!='&' && *p!=' '; i++,p++)
            ssid[i] = (*p=='+') ? ' ' : *p;
        ssid[i] = '\0';
    }

    /* ---- PASS ---- */
    p = strstr(httpReq, "pass=");
    if(p){ p += 5;
        for(i=0; i<63 && *p && *p!='&' && *p!=' '; i++,p++)
            pass[i] = (*p=='+') ? ' ' : *p;
        pass[i] = '\0';
    }

    /* ---- IP ---- */
    p = strstr(httpReq, "&ip=");
    if(p){ p += 4;
        for(i=0; i<15 && *p && *p!='&' && *p!=' '; i++,p++)
            ip[i] = *p;
        ip[i] = '\0';
    }

    /* ---- PORT ---- */
    p = strstr(httpReq, "&port=");
    if(p){ p += 6;
        uint16_t val = 0;
        while(*p >= '0' && *p <= '9')
            val = (uint16_t)(val * 10 + (*p++ - '0'));
        if(val > 0) *port = val;
    }
}

/**
 * @brief Callback del driver ESP01: se llama cada vez que cambia el estado
 *
 * Cuando el WiFi se conecta (ESP01_WIFI_CONNECTED), arranca el UDP
 * automaticamente usando la IP y puerto guardados desde el formulario.
 */
void OnESP01ChangeState(_eESP01STATUS state)
{
    if(state == ESP01_WIFI_CONNECTED){
        /* ¡Éxito! Se conectó a la red que estábamos evaluando */
        strncpy(udpTargetIP, knownNetworks[currentNetworkIdx].targetIP, 15);
        udpTargetIP[15] = '\0';

        networkScanActive = 0; /* Detenemos el escaneo */
        udpReadyToStart = 1;
    }
    else if(state == ESP01_WIFI_DISCONNECTED){
        /* Si perdemos la conexión en pleno uso, reactivamos la búsqueda */
        if(!isWebserverMode && !networkScanActive){
            networkScanActive = 1;
            networkScanTimer = SCANTIME; /* Le damos 15 segs a la red actual para recuperarse */
            ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid,
                          knownNetworks[currentNetworkIdx].password);
        }
    }
}

/**
 * @brief Tarea principal del webserver: procesa la primera linea HTTP capturada
 */

/* Macro de seguridad: centraliza el reset del buffer HTTP */
#define HTTP_BUF_RESET()  do { httpBufIdx = 0; httpBuf[0] = '\0'; } while(0)

void httpTask(void)
{
    /* Iniciar UDP en cuanto el WiFi este listo (viene del callback) */
    if(udpReadyToStart){
        udpReadyToStart = 0;
        ESP01_StartUDP(udpTargetIP, udpTargetPort, 30001);
        return;
    }

    if(!isWebserverMode)
        return;

    /* 0xFF = primera linea completa y lista para procesar */
    if(httpBufIdx != 0xFF)
        return;

    uint8_t connID = ESP01_GetLastConnID();

    if(strstr(httpBuf, "GET /set?") != NULL){
        char newSSID[64]  = {0};
        char newPASS[64]  = {0};
        char newIP[16]    = {0};
        uint16_t newPort  = 30010;

        parseHTTPGetParams(httpBuf, newSSID, newPASS, newIP, &newPort);

        /* Guardar IP y puerto para usarlos cuando conecte el WiFi */
        if(newIP[0] != '\0'){
            strncpy(udpTargetIP, newIP, 15);
            udpTargetIP[15] = '\0';
        }
        if(newPort > 0)
            udpTargetPort = newPort;

        /* Responder al navegador */
        sendHTTPOKPage(connID);

        /* Salir del modo webserver y conectar como Station */
        isWebserverMode = 0;
        HTTP_BUF_RESET();

        /* ESP01_SetWIFI arranca el flujo Station; OnESP01ChangeState
         * llamara a ESP01_StartUDP cuando el WiFi este listo */
        ESP01_SetWIFI(newSSID, newPASS);

    } else if(strstr(httpBuf, "GET /") != NULL){
        sendHTMLForm(connID);
        HTTP_BUF_RESET();

    } else {
        /* favicon.ico u otras peticiones: descartar */
        HTTP_BUF_RESET();
    }
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

void PID_ControlTask(void) {
    if (RUN_PID == FALSE)
        return;
    RUN_PID = FALSE;

    // =========================================================
    // --- 1. LECTURA E INVERSIÓN DE SENSORES ---
    // =========================================================
    // Sensores de línea (IR invertidos: negro = alto)
    int32_t left_ir   = 4095 - adcData[5];
    int32_t center_ir = 4095 - adcData[3];
    int32_t right_ir  = 4095 - adcData[1];

    if (left_ir   < 0) left_ir   = 0;
    if (center_ir < 0) center_ir = 0;
    if (right_ir  < 0) right_ir  = 0;

    sum_sensors = left_ir + center_ir + right_ir;
    if (sum_sensors == 0) sum_sensors = 1;

    // Sensores de obstáculo (valores crudos ADC, mayor = más cerca)
    int32_t ir_front       = adcData[6]; // Frontal centrado
    int32_t ir_corner_r    = adcData[0]; // Esquina derecha 45°
    int32_t ir_corner_l    = adcData[4]; // Esquina izquierda 45°
    int32_t ir_side_r      = adcData[7]; // Lateral derecho 90°
    int32_t ir_side_l      = adcData[2]; // Lateral izquierdo 90°

    // =========================================================
    // --- 2. FILTROS Y CÁLCULO DE ÁNGULO (IMU) ---
    // =========================================================
    if (ax_filt == 0 && az_filt == 0) {
        ax_filt = ax;
        az_filt = az;
    } else {
        ax_filt = (ax * 5 + ax_filt * 95) / 100;
        az_filt = (az * 5 + az_filt * 95) / 100;
    }

    if (az_filt > AZ_MIN_VALID || az_filt < -AZ_MIN_VALID) {
        int32_t ratio = (ax_filt * 1000) / az_filt;
        acc_angle_hr = (ratio * (int32_t) RADTOGRAD) / 10;
    }

    gyro_delta_hr = (-(int32_t) gy * 200) / 131;
    current_angle_hr = (ALPHA_GYRO * (current_angle_hr + gyro_delta_hr)
            + ALPHA_ACC * acc_angle_hr) / 100;
    current_angle = current_angle_hr / 100;

    // =========================================================
    // --- 3. MÁQUINA DE ESTADOS PRINCIPAL ---
    // =========================================================
    int32_t target_setpoint = setpoint;
    turn_offset = 0;

    uint8_t ir1_active   = (left_ir   > IR_WHITE);
    uint8_t ir3_active   = (center_ir > IR_WHITE);
    uint8_t ir5_active   = (right_ir  > IR_WHITE);
    uint8_t active_count = ir1_active + ir3_active + ir5_active;

    // ---------------------------------------------------------
    // MEF ESQUIVADOR — tiene prioridad sobre el seguidor de línea
    // Cuando obsState != OBS_IDLE el seguidor de línea se congela.
    // ---------------------------------------------------------
    if (obsState != OBS_IDLE) {

        switch (obsState) {

            // -------------------------------------------------
            // OBS_STOP
            // Obstáculo detectado. Para el robot durante
            // OBS_STOP_CYCLES ciclos y decide hacia qué lado girar
            // comparando los sensores de esquina 45°.
            // -------------------------------------------------
            case OBS_STOP:
                target_setpoint = 0;
                turn_offset     = 0;

                obs_timer++;
                if (obs_timer >= obs_stop_cycles) {
                    obs_timer = 0;

                    // Decide dirección: gira hacia el lado con más espacio
                    // (sensor de esquina con menor lectura = más lejos del objeto)
                    if (ir_corner_r <= ir_corner_l) {
                        obs_turn_dir    = 1;       // gira derecha
                        obs_side_sensor = 0;       // usará lateral derecho en SLIDE
                    } else {
                        obs_turn_dir    = 0;       // gira izquierda
                        obs_side_sensor = 1;       // usará lateral izquierdo en SLIDE
                    }
                    obsState = OBS_ROTATE_1;
                }
                break;

            // -------------------------------------------------
            // OBS_ROTATE_1
            // Rota 90° en el lugar hacia el lado decidido.
            // Usa OBS_ROTATE_CYCLES como tiempo de rotación.
            // Se puede refinar con los sensores de 45° si se
            // desea una rotación más precisa en el futuro.
            // -------------------------------------------------
            case OBS_ROTATE_1:
                target_setpoint = 0;

                if (obs_turn_dir == 1) {
                    turn_offset = custom_turn;
                } else {
                    turn_offset = -custom_turn;
                }

                obs_timer++;
                if (obs_timer >= obs_rotate_cycles) {
                    obs_timer = 0;
                    obsState  = OBS_SLIDE;
                }
                break;

            // -------------------------------------------------
            // OBS_SLIDE
            // Avanza paralelo al objeto manteniendo la distancia
            // lateral de referencia (obs_side_dist).
            // Un control proporcional simple ajusta turn_offset
            // para mantener la distancia.
            // Sale cuando el sensor lateral baja de obs_lost_dist
            // indicando que el objeto terminó.
            // -------------------------------------------------
            case OBS_SLIDE: {
                        int32_t lateral = (obs_side_sensor == 0) ? ir_side_r : ir_side_l;
                        int32_t dist_error = lateral - obs_side_dist;
                        int32_t dist_correction = dist_error / 20;

                        if (obs_turn_dir == 1) {
                            turn_offset = -dist_correction;
                        } else {
                            turn_offset = dist_correction;
                        }

                        target_setpoint = attack_setpoint;

                        if (lateral < obs_lost_dist) {
                            obs_timer++;
                            if (obs_timer >= obs_stop_cycles) {
                                obs_timer = 0;
                                obsState  = OBS_ROTATE_2;
                            }
                        } else {
                            obs_timer = 0;
                        }
                        break;
                    }

            // -------------------------------------------------
            // OBS_ROTATE_2
            // Rota en sentido contrario al inicial para volver
            // a mirar hacia el frente del recorrido.
            // -------------------------------------------------
            case OBS_ROTATE_2:

                        if (obs_timer < obs_stop_cycles * 3) {
                            // Fase de avance previo
                            target_setpoint = attack_setpoint;
                            turn_offset     = 0;
                            obs_timer++;
                        } else if (obs_timer < (obs_stop_cycles * 3 + obs_rotate_cycles)) {
                            // Fase de rotación: cuenta desde obs_stop_cycles*3
                            // hasta obs_stop_cycles*3 + obs_rotate_cycles
                            // → exactamente obs_rotate_cycles ciclos, igual que ROTATE_1
                            target_setpoint = 0;
                            if (obs_turn_dir == 1) {
                                turn_offset = -custom_turn;
                            } else {
                                turn_offset = custom_turn;
                            }
                            obs_timer++;
                        } else {
                            obs_timer = 0;
                            obsState  = OBS_FORWARD;
                        }
                        break;
            // -------------------------------------------------
            // OBS_FORWARD
            // Avanza manteniendo la distancia lateral mientras
            // busca el fin de la pared lateral (< obs_lost_dist).
            // Cuando el lateral cae sale a OBS_ROTATE_3.
            // -------------------------------------------------
        case OBS_FORWARD: {
            int32_t lateral = (obs_side_sensor == 0) ? ir_side_r : ir_side_l;
            int32_t dist_error = lateral - obs_side_dist;
            int32_t dist_correction = dist_error / 20;

            if (obs_turn_dir == 1) {
                turn_offset = -dist_correction;
            } else {
                turn_offset = dist_correction;
            }

            target_setpoint = attack_setpoint;

            // Espera un mínimo de ciclos antes de empezar a evaluar
            // el sensor lateral para evitar falsos positivos al entrar al estado
            if (obs_timer < obs_stop_cycles * 2) {
                obs_timer++;
            } else if (lateral < obs_lost_dist) {
                obs_timer = 0;
                obsState  = OBS_ROTATE_3;
            }
            break;
        }

            // -------------------------------------------------
            // OBS_ROTATE_3
            // Rota -90° (sentido contrario al inicial) para
            // quedar orientado hacia la línea nuevamente.
            // -------------------------------------------------
            case OBS_ROTATE_3:
                target_setpoint = 0;

                // Gira en sentido opuesto al inicial
                if (obs_turn_dir == 1) {
                    turn_offset = -custom_turn;
                } else {
                    turn_offset = custom_turn;
                }

                obs_timer++;
                if (obs_timer >= obs_rotate_cycles) {
                    obs_timer = 0;
                    obsState  = OBS_REJOIN;
                }
                break;

            // -------------------------------------------------
            // OBS_REJOIN
            // Avanza buscando la línea con el sensor central.
            // Cuando ir3_active detecta línea vuelve a OBS_IDLE
            // y retoma el seguimiento normal.
            // -------------------------------------------------
            case OBS_REJOIN:
                target_setpoint = attack_setpoint;
                turn_offset     = 0;

                if (ir3_active) {
                    obsState  = OBS_IDLE;
                    lineState = LINE_FOLLOWING;
                }
                break;

            default:
                obsState = OBS_IDLE;
                break;
        }

    } else {
        // ---------------------------------------------------------
        // obsState == OBS_IDLE: seguimiento de línea normal
        // Detección de obstáculo: si el frontal supera obs_detect_dist
        // se congela el seguidor y se activa el esquivador.
        // ---------------------------------------------------------
        if (ir_front >= obs_detect_dist) {
            obs_timer = 0;
            obsState  = OBS_STOP;

        } else {

            // =====================================================
            // MEF SEGUIDOR DE LÍNEA
            // =====================================================
            switch (lineState) {

                case LINE_SEARCHING:
                    if (ir3_active) {
                        lineState = LINE_FOLLOWING;
                    }
                    break;

                case LINE_FOLLOWING:

                    if (active_count == 3 && ir1_active && ir3_active && ir5_active) {
                        lineState = LINE_CROSS;
                        break;
                    }

                    if (active_count == 0) {
                        line_lost_timer = 0;
                        line_lost_phase = 0;
                        lineState = LINE_LOST;
                        break;
                    }

                    error_linea = ((-(1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10;

                    abs_error   = (error_linea > 0) ? error_linea : -error_linea;
                    linear_term = Kp_line * error_linea;
                    quad_term   = (Kq_line * error_linea * abs_error) / SCALE_LINE;
                    turn_offset = (linear_term + quad_term) / 4;

                    {
                        int32_t abs_turn    = (turn_offset > 0) ? turn_offset : -turn_offset;
                        int32_t curve_brake = 0;
                        if (brake_angle_div != 0) {
                            curve_brake = abs_turn / brake_angle_div;
                        }
                        target_setpoint = attack_setpoint + curve_brake;
                        if (target_setpoint > 50) target_setpoint = 50;
                    }

                    last_line_error = error_linea;
                    break;

                case LINE_LOST:

                    if (ir3_active) {
                        lineState = LINE_FOLLOWING;
                        break;
                    }

                    if (line_lost_phase == 0) {
                        turn_offset     = (last_line_error > 0) ? custom_turn : -custom_turn;
                        target_setpoint = 0;
                        line_lost_timer++;
                        if (line_lost_timer >= LINE_LOST_PHASE0) {
                            line_lost_timer = 0;
                            line_lost_phase = 1;
                        }
                    } else if (line_lost_phase == 1) {
                        turn_offset     = (last_line_error > 0) ? -custom_turn : custom_turn;
                        target_setpoint = 0;
                        line_lost_timer++;
                        if (line_lost_timer >= LINE_LOST_PHASE1) {
                            line_lost_timer = 0;
                            line_lost_phase = 2;
                        }
                    } else {
                        turn_offset     = (last_line_error > 0) ? -custom_turn : custom_turn;
                        target_setpoint = 0;
                    }
                    break;

                case LINE_CROSS:
                    if (active_count < 3) {
                        lineState = LINE_FOLLOWING;
                        break;
                    }
                    target_setpoint = attack_setpoint;
                    break;

                default:
                    lineState = LINE_SEARCHING;
                    break;
            }
        }
    }

    // =========================================================
    // --- 4. LAZO PID CENTRAL (Equilibrio) ---
    // =========================================================
    error      = target_setpoint - current_angle;
    derivative = ((error - last_error) * 1000) / DT_MS;

    if (error > -150 && error < 150) {
        integral += error * DT_MS;
        if (integral >  (ANG20 * DT_MS)) integral =  (ANG20 * DT_MS);
        if (integral < -(ANG20 * DT_MS)) integral = -(ANG20 * DT_MS);
    } else {
        integral = (integral * 8) / 10;
    }

    output     = (Kp_stable * error + (Ki_stable * integral) / 1000 + (Kd_stable * derivative)) / 10000;
    last_error = error;

    // =========================================================
    // --- 5. INYECCIÓN INSTANTÁNEA DE FRICCIÓN (minPWM) ---
    // =========================================================
    int32_t final_pwm_left;
    int32_t final_pwm_right;

    if (output > 0) {
        final_pwm_left  = output + minPWM_left;
        final_pwm_right = output + minPWM_right;
    } else if (output < 0) {
        final_pwm_left  = output - minPWM_left;
        final_pwm_right = output - minPWM_right;
    } else {
        final_pwm_left  = 0;
        final_pwm_right = 0;
    }

    if (current_angle > ANG45 || current_angle < -ANG45) {
        final_pwm_left  = 0;
        final_pwm_right = 0;
        integral        = 0;
    }

    // =========================================================
    // --- 6. MIXER DIRECCIONAL Y SATURACIÓN ---
    // =========================================================
    int32_t pwm_left  = final_pwm_left;
    int32_t pwm_right = final_pwm_right;

    if (obsState == OBS_ROTATE_1 || obsState == OBS_ROTATE_2 ||
        obsState == OBS_ROTATE_3 || obsState == OBS_STOP     ||
        lineState == LINE_LOST) {
        // Rotación en el lugar: una rueda adelante, la otra atrás
        if (turn_offset > 0) {
            pwm_left  = -(int32_t) minPWM_left;
            pwm_right =  (int32_t) minPWM_right;
        } else if (turn_offset < 0) {
            pwm_left  =  (int32_t) minPWM_left;
            pwm_right = -(int32_t) minPWM_right;
        } else {
            pwm_left  = 0;
            pwm_right = 0;
        }
    } else {
        if (final_pwm_left != 0) {
            if (final_pwm_left > 0) {
                pwm_left  += (offset_left  - turn_offset);
                pwm_right += (offset_right + turn_offset);
            } else {
                pwm_left  -= (offset_left  - turn_offset);
                pwm_right -= (offset_right + turn_offset);
            }
        }
    }

    if (pwm_left  >  (int32_t) maxPWM) pwm_left  =  (int32_t) maxPWM;
    if (pwm_left  < -(int32_t) maxPWM) pwm_left  = -(int32_t) maxPWM;
    if (pwm_right >  (int32_t) maxPWM) pwm_right =  (int32_t) maxPWM;
    if (pwm_right < -(int32_t) maxPWM) pwm_right = -(int32_t) maxPWM;

    if (current_angle > ANG45 || current_angle < -ANG45) {
		final_pwm_left = 0;
		final_pwm_right = 0;
		integral = 0;
		pwm_left = 0;
		pwm_right = 0;
	}
    // =========================================================
    // --- 7. MAPEO AL HARDWARE ---
    // =========================================================
    // Izquierda: CH4 (Adelante), CH3 (Atrás)
    if (pwm_left > 0) {
        rPulse4 = (uint16_t) pwm_left;
        lPulse3 = 0;
    } else {
        lPulse3 = (uint16_t) (-pwm_left);
        rPulse4 = 0;
    }

    // Derecha: CH2 (Adelante), CH1 (Atrás)
    if (pwm_right > 0) {
        rPulse2 = (uint16_t) pwm_right;
        lPulse1 = 0;
    } else {
        lPulse1 = (uint16_t) (-pwm_right);
        rPulse2 = 0;
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
  	ESP01_AttachChangeState(OnESP01ChangeState); /* Inicia UDP automaticamente al conectar */

  	HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1); //non blocking


  	/* ---- MODO WEBSERVER: el dispositivo levanta un AP para recibir credenciales WiFi ----
  	 * Conectarse con el telefono a la red "MiDispositivo" (pass: 12345678)
  	 * y navegar a 192.168.4.1 para ingresar el SSID y contraseña del router.
  	 * Una vez recibidas las credenciales, el driver llama automaticamente a ESP01_SetWIFI().
  	 * ---- Para volver al modo UDP/TCP comentar esta linea y descomentar las de abajo ---- */
  	isWebserverMode = FALSE;
  	//ESP01_SetWebServer("MICRO", "12345678", 5, 3);

  	/* ---- AUTO-SCAN DE REDES ----
  	 * Intenta conectar a cada red conocida en orden.
  	 * Si falla, el callback OnESP01ChangeState pasa a la siguiente.
  	 * Cuando conecta, carga la IP correspondiente automáticamente. */
  	currentNetworkIdx = 0;
  	networkRetryCount = 0;
  	networkScanTimer = SCANTIME; /* Darle 15 segs a la primera red*/
  	networkScanActive = 1;
  	ESP01_SetWIFI(knownNetworks[currentNetworkIdx].ssid,
  	              knownNetworks[currentNetworkIdx].password);

  	//Inicializacion de protocolo
  	unerPrtcl_Init(&USBRx, &USBTx, buffUSBRx, buffUSBTx);
  	unerPrtcl_Init(&WiFiRx, &WiFiTx, buffWiFiRx, buffWiFiTx);
  	//Variables
  	ALLFLAGS = RESET;

  	lPulse1=0;
  	lPulse3=0;
  	rPulse2=0;
  	rPulse4=0;

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
	httpTask();

	COMMTask(&USBRx, &USBTx, SERIE);
	COMMTask(&WiFiRx, &WiFiTx, WIFI);

	PWM_Control();
	i2cTask();

	PID_ControlTask();

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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // En la familia STM32F1, leer el registro SR y luego el DR limpia el error ORE
        uint32_t er = huart->Instance->SR;
        uint32_t dr = huart->Instance->DR;
        (void)er;
        (void)dr;

        // Forzamos el reinicio de la escucha
        huart->RxState = HAL_UART_STATE_READY;
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
