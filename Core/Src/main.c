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
// Estructura contenedora de credenciales de red conocida
typedef struct {
	const char *ssid;
	const char *password;
	const char *targetIP;
} _sWiFiNetwork;

// MEF de estados de trayectoria de la pista (Seguidor de línea)
typedef enum {
	LINE_SEARCHING,                                   // Estado 0: Pérdida total inicial, buscando línea activamente
	LINE_FOLLOWING,                                   // Estado 1: Seguimiento PID/Lineal normal sobre la pista
	LINE_LOST,                                        // Estado 2: Desvío reciente de la línea, aplicando memoria de búsqueda
	LINE_CROSS                                        // Estado 3: Cruce o bifurcación en T detectada (todo negro)
} _eLineState;

// MEF del control de esquivado de obstáculos
typedef enum {
	OBS_IDLE,                                         // Estado 0: Libre de obstáculos frontalmente
	OBS_APPROACH,                                     // Estado 1: Detección frontal inminente, decidiendo desvío
	OBS_CORNER,                                       // Estado 2: Maniobrando esquinas cerradas alrededor del objeto
	OBS_WALL                                          // Estado 3: Seguimiento paralelo a la pared del obstáculo
} _eObsState;

typedef enum {
    STATE_SWING = 0,          // 1 clic: Solo balanceo estático en el lugar, pantalla off
    STATE_LINE_FOLLOWING = 1, // 2 clics: Seguimiento de línea activo, pantalla off
    STATE_DODGE = 2,          // 3 clics: Esquivado de obstáculos activo, pantalla off
    STATE_FIRST_SCREEN = 3,   // Pulsación 1s: Pantalla RAW (ADC, ACC, GYR), motores off
    STATE_SECOND_SCREEN = 4   // Pulsación 2s: Pantalla Premium (VEL, ACL, ANG, BAL), motores off
} _eRobotMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// =========================================================
// //GENERAL
// =========================================================
#define TO10MS              40       // Umbral para tarea de 10ms (40 x 250us)
#define T100MS              100      // Equivalente a 100ms
#define T1000MS             1000     // Equivalente a 1000ms
#define T2000MS             2000     // Equivalente a 2000ms
#define T3000MS             3000     // Equivalente a 3000ms
#define T400MS              400      // Ventana de espera para multiclic de botones (400ms)
#define ON                  1        // Banderas de estado encendido
#define OFF                 0        // Banderas de estado apagado
#define DEBOUNCE            4        // Ciclos de antirrebote para botones

// =========================================================
// //DISPLAY
// =========================================================
#define SSD1306_MAXADC      30       // Límite superior de ADC para gráficos en pantalla
#define SSD1306_MINADC      60       // Límite inferior de ADC para gráficos en pantalla
#define SSD1306_SNDCOL      40       // Segunda columna del display
#define SSD1306_TRDCOL      85       // Tercera columna del display
#define SSD1306             0        // Identificador de tarea de pantalla en la Pila I2C

// =========================================================
// //MPU
// =========================================================
#define MPU6050             1        // Identificador de tarea del giroscopio en la Pila I2C
#define DT_MS               5        // Delta de tiempo nominal en ms para integración de giroscopio (ajustado a 5ms)
#define DT_US               5000     // Delta de tiempo nominal en us para el lazo PID (ajustado a 5ms)
#define ALPHA_GYRO          980      // Confianza en escala x1000 del filtro complementario en el giroscopio (98.0%)
#define ALPHA_ACC           20       // Confianza en escala x1000 del filtro complementario en el acelerómetro (2.0%)
#define AZ_MIN_VALID        4000     // Mínimo valor absoluto del acelerómetro Z para validar el ángulo
#ifndef MPU6050_ADDR
#define MPU6050_ADDR        (0x68 << 1) // Dirección I2C del giroscopio MPU6050
#endif

// =========================================================
// //I2C
// =========================================================
#define I2CSIZE             16       // Tamaño del buffer de tareas I2C (Pila)

// =========================================================
// //MOTORES
// =========================================================
#define TIM3CP              9999     // Período máximo del PWM de los motores (Timer 3)

// =========================================================
// //WIFI
// =========================================================
#define HTTP_BUF_SIZE       128      // Tamaño máximo de almacenamiento de request HTTP (Webserver)
#define NUM_KNOWN_NETWORKS  (sizeof(knownNetworks) / sizeof(knownNetworks[0])) // Cantidad de redes registradas
#define SCANTIME            3000     // Tiempo en ms para escaneo de redes conocidas
#define HTTP_BUF_RESET()  do { httpBufIdx = 0; httpBuf[0] = '\0'; } while(0) /* Macro de seguridad: centraliza el reset del buffer HTTP */

// =========================================================
// //PID
// =========================================================
#define PID_SCALE_FACTOR    100      // Factor de escala (x100) para evitar uso de floats en el PID
#define ANG45               45*PID_SCALE_FACTOR // Ángulo de caída extrema (45.00°)
#define ANG20               20*PID_SCALE_FACTOR // Ángulo de límite de integración PID (20.00°)
#define ANG18               18*PID_SCALE_FACTOR // Ángulo de inclinación crítica delantera (18.00°)
#define ANG15               15*PID_SCALE_FACTOR // Ángulo de umbral dinámico trasero en curva (15.00°)
#define ANG12               12*PID_SCALE_FACTOR // Ángulo de setpoint de ataque en curva (12.00°)
#define ANG10               10*PID_SCALE_FACTOR // Ángulo de setpoint de ataque estándar (10.00°)
#define ANG7_5              75*PID_SCALE_FACTOR/10 // Ángulo de recuperación amortiguado (7.50°)
#define ANG2                2*PID_SCALE_FACTOR  // Ángulo de caída delantera (2.00°)



// =========================================================
// //SEGUIDOR
// =========================================================
#define SCALE_LINE          1000     // Factor de escala para el término cuadrático de error de línea
#define IR_WHITE            200      // Umbral analógico para considerar superficie blanca
#define IR6_BOX_THRESHOLD   2000     // Umbral analógico para detección de caja (IR6)
#define LINE_LOST_PHASE0    35       // Duración de la primera fase de búsqueda en ciclos
#define LINE_LOST_PHASE1    70       // Duración de la segunda fase de búsqueda en ciclos

// Estados para la sub-MEF de pérdida de línea
#define LINE_LOST_ROT_90      0
#define LINE_LOST_ROT_180     1
#define LINE_LOST_STOPPED     2

// =========================================================
// //LUT
// =========================================================
#define LUT_SIZE            16       // Tamaño de las Look-Up Tables de calibración de sensores
#define lut_l1              LUT_IR1_IZQ           // Alias de tabla de calibración izquierda
#define lut_l2              LUT_IR3_CEN           // Alias de tabla de calibración central
#define lut_l3              LUT_IR5_DER           // Alias de tabla de calibración derecha
#define lut_l4              LUT_PROMEDIO          // Alias de tabla de calibración promedio
#define lut_y               LUT_Y_SCALE           // Alias de escala de salida (0 a 1000)

#define lut_l1_x            LUT_IR1_IZQ         // Mapeo X de tabla de calibración izquierda
#define lut_l1_y            LUT_Y_SCALE         // Mapeo Y de tabla de calibración izquierda
#define lut_l2_x            LUT_IR3_CEN         // Mapeo X de tabla de calibración central
#define lut_l2_y            LUT_Y_SCALE         // Mapeo Y de tabla de calibración central
#define lut_l3_x            LUT_IR5_DER         // Mapeo X de tabla de calibración derecha
#define lut_l3_y            LUT_Y_SCALE         // Mapeo Y de tabla de calibración derecha
#define lut_l4_x            LUT_PROMEDIO        // Mapeo X de tabla de calibración promedio
#define lut_l4_y            LUT_Y_SCALE         // Mapeo Y de tabla de calibración promedio

// =========================================================
// //BANDERAS
// =========================================================
#define ALLFLAGS            myFlags.bytes       // Acceso a bytes completos de las banderas
#define IS10MS              myFlags.bits.bit0   // Bandera de ciclo de 10ms activo
#define IS20MS              myFlags.bits.bit1   // Bandera de ciclo de 20ms activo
#define IS100MS             myFlags.bits.bit2   // Bandera de ciclo de 100ms activo
#define RUN_PID             myFlags.bits.bit4   // Bandera para ejecutar PID balancín
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
volatile _eRobotMode robotMode = STATE_SWING;
volatile _eRobotMode lastMode = (_eRobotMode)-1;

// Variables de botones encapsuladas dentro de la estructura _sButton myButton

// =========================================================
// //GENERAL
// =========================================================
// Nueva máscara ajustada a 20 ranuras de 100ms síncronas (Ciclo de 2.0 segundos)
uint32_t heartBeatMask[] = {
    0x00000001,  // Indice 0: STATE_SWING (1 parpadeo de 100ms)
    0x00000005,  // Indice 1: STATE_LINE_FOLLOWING (2 parpadeos de 100ms)
    0x00000015,  // Indice 2: STATE_DODGE (3 parpadeos de 100ms)
    0x0000001F,  // Indice 3: STATE_FIRST_SCREEN (encendido 500ms, apagado 1.5s)
    0x000003FF   // Indice 4: STATE_SECOND_SCREEN (encendido 1000ms, apagado 1s)
};
const char firmware[] = "EX100923v01\n";             // Versión actual del firmware del microcontrolador
uint8_t hbIndex = 0;                                  // Índice para seleccionar la máscara del LED (Heartbeat) - Inicializado para STATE_SWING (1 parpadeo de 100ms)
uint8_t time10ms;                                     // Temporizador incremental de 250us para llegar a 10ms
uint8_t tmo100ms = 10;                                // Temporizador de paciencia para eventos de 100ms
uint8_t tmo20ms = 2;                                  // Temporizador de paciencia para eventos de 20ms
uint8_t tmo100 = 20;                                  // Divisor de ciclo para intercalar tareas en I2C (ajustado para TIM2 a 5ms para mantener refresco a 100ms)

// =========================================================
// //DISPLAY
// =========================================================
volatile uint8_t ssd1306_TxCplt = 0;                  // Bandera indicadora de fin de transmisión I2C por DMA para pantalla
uint16_t staticOff = 400;                             // Offset estático para dibujo de gráficos
uint16_t movingOff = 300;                             // Offset móvil para dibujo de gráficos interactivos

// =========================================================
// //MPU
// =========================================================
volatile uint8_t mpu6050_RxCplt = 0;                  // Bandera indicadora de recepción I2C por DMA de datos del giroscopio
int16_t ax = 0;                                       // Lectura cruda del acelerómetro en el eje X
int16_t ay = 0;                                       // Lectura cruda del acelerómetro en el eje Y
int16_t az = 0;                                       // Lectura cruda del acelerómetro en el eje Z
int16_t gx = 0;                                       // Lectura cruda del giroscopio en el eje X
int16_t gy = 0;                                       // Lectura cruda del giroscopio en el eje Y
int16_t gz = 0;                                       // Lectura cruda del giroscopio en el eje Z
volatile int32_t speed = 0;                           // Estimación física de la velocidad lineal en mm/s
volatile int32_t dynamic_accel = 0;                   // Aceleración lineal filtrada y compensada
volatile uint16_t calib_cycle = 0;                    // Contador de ciclos de calibración inicial del MPU (para telemetría)

// =========================================================
// //I2C
// =========================================================
uint8_t Pila[I2CSIZE] = {};                           // Cola de tareas I2C pendientes de despacho
uint8_t i2cIndex = 0;                                 // Índice circular actual para despacho en la cola I2C

// =========================================================
// //ADC
// =========================================================
uint16_t adcData[8];                                  // Buffer de DMA que almacena los valores crudos del ADC del micro
uint16_t adcDataTx[8];                                // Copia segura del buffer de ADC para transmisión libre de colisiones

// =========================================================
// //COMUNICACION
// =========================================================
_sComm USBTx;                                         // Estructura de protocolo para el buffer de transmisión USB
_sComm USBRx;                                         // Estructura de protocolo para el buffer de recepción USB
_sComm WiFiTx;                                        // Estructura de protocolo para el buffer de transmisión WiFi (ESP-01)
_sComm WiFiRx;                                        // Estructura de protocolo para el buffer de recepción WiFi (ESP-01)
volatile uint8_t buffUSBTx[RXBUFSIZE];                // Array de memoria física para buffer de transmisión USB
volatile uint8_t buffUSBRx[TXBUFSIZE];                // Array de memoria física para buffer de recepción USB
volatile uint8_t buffWiFiTx[RXBUFSIZE];               // Array de memoria física para buffer de transmisión WiFi
volatile uint8_t buffWiFiRx[TXBUFSIZE];               // Array de memoria física para buffer de recepción WiFi
_uWord myWord;                                        // Unión de propósito general para conversión de tipos de datos (2/4 bytes)

// =========================================================
// //BANDERAS
// =========================================================
volatile _uFlag myFlags;                              // Banderas de ciclo de tareas del sistema en tiempo real

// =========================================================
// //MOTORES
// =========================================================
uint16_t lPulse1 = 0;                                 // Ancho de pulso PWM para Motor Izquierdo Adelante
uint16_t rPulse2 = 0;                                 // Ancho de pulso PWM para Motor Derecho Adelante
uint16_t lPulse3 = 0;                                 // Ancho de pulso PWM para Motor Izquierdo Atrás
uint16_t rPulse4 = 0;                                 // Ancho de pulso PWM para Motor Derecho Atrás

// =========================================================
// //WIFI
// =========================================================
uint8_t timerUDP = 0;                                 // Temporizador para vigilar transmisión periódica UDP
uint8_t udpSilenceCounter = 5;                        // Contador de silencio UDP (manda ALIVEs autónomos en ausencia de comandos)
uint8_t byteUART_ESP01;                               // Byte de almacenamiento de interrupción UART para WiFi
_sESP01Handle esp01Handler;                           // Estructura de llamadas y control del módulo Wi-Fi
_sButton myButton;                                    // Estructura de estado físico del botón de configuración

// =========================================================
// //WEBSERVER
// =========================================================
static char httpBuf[HTTP_BUF_SIZE];                   // Buffer para acumular el request del servidor web local
static uint8_t httpBufIdx = 0;                        // Índice actual en el buffer HTTP (0xFF indica petición lista)
static uint8_t isWebserverMode = 0;                   // Estado bandera del modo Webserver activo (1 = activo, 0 = inactivo)
static uint8_t httpTxBuf[1024];                      // Buffer de transmisión compartido para respuestas HTML y JSON (1024 bytes exactos)
static char udpTargetIP[16] = "192.168.0.10";         // Dirección IP de destino UDP/TCP para envío de telemetría
static uint16_t udpTargetPort = 30010;                // Puerto de destino UDP/TCP de la aplicación de escritorio
static uint8_t udpReadyToStart = 0;                   // Bandera que indica que el socket UDP/TCP está listo para despachar
static char udpTargetProto[4] = "TCP";                // Protocolo de transporte por defecto ("TCP" para pruebas de PID)

// =========================================================
// //REDES
// =========================================================
static const _sWiFiNetwork knownNetworks[] = {
		{ "POCOX8",    "12345678",                "10.168.4.213"   },
	{ "ARPANET", "1969-Apolo_11-2022",       "192.168.0.10"   },
	{ "FCAL-Personal", "fcal-uner+2019",       "172.22.237.227" },
	{ "FCAL",    "fcalconcordia.06-2019",    "172.23.190.89"  },
	{ "InternetPlus_872f10_EXT", "wlan78d0ef", "192.168.1.52" },
};                                                    // Base de datos local de redes Wi-Fi a las cuales autoconectarse

static uint8_t currentNetworkIdx = 0;                 // Red actual en intento de conexión por el escáner
static uint8_t networkScanActive = 0;                 // _eDMA Bandera indicadora de escáner de redes activo
static uint16_t networkScanTimer = SCANTIME;          // Temporizador de permanencia en escaneo de red en milisegundos

// =========================================================
// //SENSORES
// =========================================================
const uint16_t LUT_IR1_IZQ[LUT_SIZE] = {78, 167, 315, 384, 520, 722, 807, 903, 1055, 1327, 1492, 1629, 2213, 2442, 2919, 3682}; // LUT del sensor IR izquierdo
const uint16_t LUT_IR3_CEN[LUT_SIZE] = {149, 298, 459, 622, 868, 1261, 1501, 1720, 2246, 2874, 3261, 3499, 3847, 3875, 3885, 3905}; // LUT del sensor IR central
const uint16_t LUT_IR5_DER[LUT_SIZE] = {102, 237, 375, 518, 705, 1092, 1255, 1490, 1931, 2501, 2865, 3140, 3826, 3853, 3881, 3892}; // LUT del sensor IR derecho
const uint16_t LUT_PROMEDIO[LUT_SIZE] = {109, 234, 383, 508, 697, 1025, 1187, 1371, 1744, 2234, 2539, 2756, 3295, 3390, 3561, 3826}; // LUT de calibración promedio
const uint16_t LUT_Y_SCALE[16] = {0, 67, 133, 200, 267, 333, 400, 467, 533, 600, 667, 733, 800, 867, 933, 1000}; // Escala normalizada de salida (0 = Blanco, 1000 = Negro)

// (LUTs para sensores superiores eliminadas para usar valores crudos directamente)

volatile int16_t cal_left_ir = 0;                     // Lectura calibrada del sensor IR izquierdo (Der-Raw en telemetría)
volatile int16_t cal_center_ir = 0;                   // Lectura calibrada del sensor IR central (Cen-Raw en telemetría)
volatile int16_t cal_right_ir = 0;                    // Lectura calibrada del sensor IR derecho (Izq-Raw en telemetría)

// Lecturas calibradas de sensores superiores (esquivar objeto)
volatile int16_t cal_ir0 = 0;
volatile int16_t cal_ir2 = 0;
volatile int16_t cal_ir4 = 0;
volatile int16_t cal_ir6 = 0;
volatile int16_t cal_ir7 = 0;

// Prototipos de funciones de calibración asociadas
static uint16_t LUT_Interpolate(const uint16_t *x, const uint16_t *lut_y, uint16_t raw);
void NormalizeLineSensors(const uint16_t *adcDataTx_ptr, uint16_t *norm);

// =========================================================
// //PID
// =========================================================
int32_t acc_angle_hr = 0;                             // Ángulo del acelerómetro de alta resolución
int32_t gyro_delta_hr = 0;                            // Incremento angular de alta resolución calculado del giroscopio
int32_t current_angle_hr = 0;                         // Ángulo complementario filtrado de alta resolución
int32_t current_angle = 0;                            // Ángulo complementario de salida del robot en escala x100
int32_t measured_dt_ms = 20;                          // Delta de tiempo real medido en ms de ejecución del bucle
int32_t ax_filt = 0;                                  // Aceleración filtrada en el eje X
int32_t az_filt = 0;                                  // Aceleración filtrada en el eje Z

int32_t error = 0;                                    // Diferencia entre setpoint y ángulo actual
int32_t last_error = 0;                               // Error del ciclo PID inmediatamente anterior para cálculo derivativo
int32_t derivative = 0;                               // Componente derivativo del lazo PID
int32_t integral = 0;                                 // Componente acumulativo integral del lazo PID
int32_t output = 0;                                   // Acción de control total del lazo de equilibrio inyectada a los motores

int16_t Kp_stable = 85;                               // Ganancia proporcional de equilibrio estático (ajustado por el usuario)
int16_t Kd_stable = 2;                                // Ganancia derivativa de equilibrio estático (ajustado por el usuario)
int16_t Ki_stable = 0;                                // Ganancia integral de equilibrio estático (revertido al valor original)

uint16_t maxPWM = 9999;                               // Ciclo de trabajo máximo permitido (100%)
uint16_t minPWM_Left = 800;                           // PWM mínimo que vence la fricción estática de la rueda izquierda
uint16_t minPWM_Right = 1025;                         // PWM mínimo que vence la fricción estática de la rueda derecha
uint16_t PWM_LRot = 880;                              // PWM estático de pivote de giro en búsqueda para rueda izquierda
uint16_t PWM_RRot = 800;                              // PWM estático de pivote de giro en búsqueda para rueda derecha
int16_t offset_left = 0;                              // Offset para compensación de deriva de tracción del motor izquierdo
int16_t offset_right = 0;                             // Offset para compensación de deriva de tracción del motor derecho

int32_t setpoint = -1000;                                // Setpoint de equilibrio estático base (x100 = 0.5°) ajustable por Qt
int16_t attack_setpoint = -1600;                      // Setpoint de inclinación frontal para ataque (configurable por Qt)

/* Variables del Lazo Cascada Externo */
int32_t pwm_filtrado = 0;                             // Esfuerzo de motor filtrado LPF (Lazo Lento)
int32_t integral_esfuerzo = 0;                         // Integral de error de esfuerzo para lazo externo
int16_t angulo_modificador_pi = 0;                     // Corrección de ángulo calculado por PI (x100)

/* Parámetros configurables del Lazo Externo */
int16_t Kp_ext = 40;                                 // Ganancia proporcional de lazo externo (x1000)
int16_t Ki_ext = 200;                                  // Ganancia integral de lazo externo (x10000)
int16_t alfa_lpf = 10;                                // Coeficiente alfa del filtro LPF (0-100)

// =========================================================
// //SEGUIDOR
// =========================================================
int16_t Kp_line = 150;                                // Ganancia proporcional de guiñada para corrección rápida sobre la línea
int16_t Kq_line = 3;                                 // Ganancia derivativa/cuadrática de guiñada para atenuar oscilaciones
int16_t Kp_line_backup = 300;                         // Respaldo de Kp_line al entrar a Swing
int16_t Kq_line_backup = 15;                          // Respaldo de Kq_line al entrar a Swing
int32_t sum_sensors = 0;                              // Suma de lecturas normalizadas de los sensores de línea activos
int32_t error_linea = 0;                              // Desviación calculada de la línea (eje horizontal de error)
int32_t abs_error = 0;                                // Valor absoluto del error de línea
int32_t linear_term = 0;                              // Aporte proporcional del control de dirección
int32_t quad_term = 0;                                // Aporte cuadrático/derivativo del control de dirección
int32_t turn_offset = 0;                              // Fuerza de rotación mezclada con el PID y enviada a los motores (Yaw)
int32_t last_line_error = 0;                          // Error de línea del ciclo anterior
int16_t custom_turn = 350;                            // Intensidad de giro prefijada para fases ciegas de búsqueda
int16_t vel_damp_div = 500;                           // Divisor del término amortiguador de velocidad
int16_t vel_damp_limit = 100;                         // Límite del amortiguador de velocidad
int16_t turn_limit = 1000;                            // Límite superior absoluto del esfuerzo de giro motor (Yaw)

int16_t ax_offset = 0;                                // Offset calibrado de gravedad en reposo del acelerómetro X

_eLineState lineState = LINE_SEARCHING;               // Estado actual de la máquina del seguidor de línea
uint16_t line_lost_timer = 0;                         // Temporizador en ciclos transcurridos desde que se perdió la pista
uint8_t line_lost_phase = 0;                          // Fase de búsqueda secuencial actual (fase 0, 1 o 2)

// =========================================================
// //OBSTACULO
// =========================================================
uint16_t obs_detect_dist = 1000;                      // Distancia frontal de detección en mm
uint16_t obs_corner_dist = 800;                       // Distancia lateral del sensor de 45° para validar esquina
uint16_t obs_lost_dist = 400;                         // Distancia mínima lateral por debajo de la cual la pared terminó
int32_t Kp_pared = 9;                                 // Fuerza principal para mantener distancia lateral (90°)
int32_t Kd_anticipo = 4;                              // Fuerza menor de ayuda/anticipación anticipada (45°)
int32_t Kp_frontal = 3;                               // Control preventivo proporcional sensor central (menor factor)
int32_t Kd_frontal = 6;                               // Anticipación de guiñada (derivada frontal) sensor central
uint16_t obs_side_dist = 1000;                        // Distancia lateral de referencia deseada para seguir la pared
uint16_t obs_stop_cycles = 10;                        // Ciclos de inmovilización previa antes de iniciar rotación evasiva
uint16_t obs_align_dist = 2500;                       // Distancia objetivo del sensor lateral tras rotación de 90°

// =========================================================
// //DODGE ROTATION
// =========================================================
// =========================================================
// //DODGE ROTATION
// =========================================================
typedef enum {
    DODGE_LINE_FOLLOWING,      // Seguimiento de línea con rampa de desaceleración
    DODGE_ROTATING,            // Rotación de 90° con giroscopio
    DODGE_WALL_FOLLOWING,      // Evasión PD continua
    DODGE_STANDBY,             // Standby unificado de frenado (1.0s a +750, 2.0s a -250)
    DODGE_LINE_RECOVER_ROT90   // Rotación de 90° a la derecha (setpoint -250) con gz para retomar la línea
} _eDodgeSubState;


volatile _eDodgeSubState dodgeState = DODGE_LINE_FOLLOWING; // Inicia en seguimiento de línea con esquivado activo
volatile int32_t dodge_yaw = 0;
volatile int32_t total_yaw_hr = 0;                    // Ángulo Yaw total acumulado continuo en milígrados
volatile int32_t dodge_step_target_yaw = 2000; // Micro-paso inicial de 2.0° (2,000 mdeg)
volatile uint32_t dodge_timer = 0;
volatile uint8_t dodge_wall_count = 0;
volatile int16_t gz_offset = 0;
volatile uint8_t dodge_config_mode = 0;       // 0: Siempre Derecha, 1: Siempre Izquierda, 2: Alternado
volatile int8_t dodge_direction = -1;         // -1: Derecha, +1: Izquierda (dirección de la maniobra activa)
static int8_t dodge_alt_next = -1;            // Próxima dirección en modo alternado (-1: Derecha)

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
_eESP01STATUS sendJSONTelemetry(uint8_t connID);
_eESP01STATUS sendHTMLForm(uint8_t connID);
_eESP01STATUS sendHTTPOKPage(uint8_t connID);
void parseHTTPGetParams(const char *httpReq, char *ssid, char *pass, char *ip, uint16_t *port, char *proto);
void httpTask(void);
void OnESP01ChangeState(_eESP01STATUS state);
//PID
void PID_ControlTask(void);
void LineFollowingMEF(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint);
void Speed_IntegrationTask(uint32_t dt_us);
static void WiFi_ScanTick(void);
static void UART_EnforceReceiverActive(void);
static void WiFi_HeartbeatTick(void);

void HandleModeScreenTransition(void);

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

	if (htim->Instance == TIM2) { // 5ms (ajustado para muestreo rápido)
		// En modos de movimiento, encolamos el MPU6050 en cada ciclo de 5ms
		if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
			Pila[i2cIndex] = MPU6050;
			i2cIndex++;
			i2cIndex&=(I2CSIZE-1);
		}

		tmo100--;
		if(!tmo100){
			tmo100=20; // 20 * 5ms = 100ms para mantener el intervalo de actualización de la pantalla
			// En modos de pantalla de telemetría, actualizamos secuencialmente
			if (robotMode == STATE_FIRST_SCREEN || robotMode == STATE_SECOND_SCREEN) {
				Pila[i2cIndex] = MPU6050;
				i2cIndex++;
				i2cIndex&=(I2CSIZE-1);

				Pila[i2cIndex] = SSD1306;
				i2cIndex++;
				i2cIndex&=(I2CSIZE-1);
			}
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
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		{
			int16_t val_L = (int16_t)myWord.ui16[0];
			if (val_L >= 0) {
				rPulse4 = val_L;
				lPulse3 = 0;
			} else {
				lPulse3 = -val_L;
				rPulse4 = 0;
			}
		}
		break;
	case SETPWMR:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMR, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		{
			int16_t val_R = (int16_t)myWord.ui16[0];
			if (val_R >= 0) {
				rPulse2 = val_R;
				lPulse1 = 0;
			} else {
				lPulse1 = -val_R;
				rPulse2 = 0;
			}
		}
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
        minPWM_Right = myWord.ui16[0];
		break;
	case SETPWMMINL:
        unerPrtcl_PutHeaderOnTx(dataTx, SETPWMMINL, 2);
        unerPrtcl_PutByteOnTx(dataTx, ACK );
        unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
        myWord.ui8[0]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        myWord.ui8[1]=unerPrtcl_GetByteFromRx(dataRx,1,0);
        minPWM_Left = myWord.ui16[0];
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
		break;
	case SETSTATICOFF:
		unerPrtcl_PutHeaderOnTx(dataTx, SETSTATICOFF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		staticOff = myWord.ui16[0];
		break;
	case SETMOVINGOFF:
		unerPrtcl_PutHeaderOnTx(dataTx, SETMOVINGOFF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		movingOff = myWord.ui16[0];
		break;
	case GETINTERNALDATA:
		// Estructura simplificada para sincronización de parámetros (86 bytes de datos + 1 chk)
		unerPrtcl_PutHeaderOnTx(dataTx, GETINTERNALDATA, 87);

		// 1. Bloque PID Balancín (10 bytes: Kp, Ki, Kd, Max, Min)
		int16_t pid_bal[5] = { Kp_stable, Ki_stable, Kd_stable, (int16_t)minPWM_Right, (int16_t)minPWM_Left};
		for (int i = 0; i < 5; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (pid_bal[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((pid_bal[i] >> 8) & 0xFF));
		}

		// 2. Setpoint (4 bytes - int32)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (setpoint & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 16) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((setpoint >> 24) & 0xFF));

		// 3. Bloque Seguimiento y Offsets (14 bytes: KpL, KdL, OffL, OffR, Turn, Attack, Kp_pared)
		int16_t params_ext[7] = { Kp_line, Kq_line, offset_left, offset_right, custom_turn, attack_setpoint, (int16_t)Kp_pared };
		for (int i = 0; i < 7; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_ext[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_ext[i] >> 8) & 0xFF));
		}

		// 4. Bloque Esquivador de Obstáculos (12 bytes: Front, Side, Lost, Stop, Corner, Align)
		uint16_t params_obs[6] = { obs_detect_dist, obs_side_dist, obs_lost_dist, obs_stop_cycles, obs_corner_dist, obs_align_dist};
		for (int i = 0; i < 6; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_obs[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_obs[i] >> 8) & 0xFF));
		}

		// 5. Bloque Rotación PWM (8 bytes: PWM_LRot, PWM_RRot, staticOff, movingOff)
		uint16_t params_rot[4] = { PWM_LRot, PWM_RRot, staticOff, movingOff };
		for (int i = 0; i < 4; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (params_rot[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((params_rot[i] >> 8) & 0xFF));
		}

		// 6. Nuevos Parámetros (4 bytes: angle_limit, Kp_ext)
		int16_t new_params[2] = { 0, Kp_ext };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (new_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((new_params[i] >> 8) & 0xFF));
		}

		// 7. Anti-collapse Setpoints (4 bytes: Ki_ext, alfa_lpf)
		int16_t save_params[2] = { Ki_ext, alfa_lpf };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (save_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((save_params[i] >> 8) & 0xFF));
		}

		// 8. current_angle_hr (4 bytes)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (current_angle_hr & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 16) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((current_angle_hr >> 24) & 0xFF));

		// 9. Vel Damp Params (4 bytes: vel_damp_div, vel_damp_limit)
		int16_t damp_params[2] = { vel_damp_div, vel_damp_limit };
		for (int i = 0; i < 2; i++) {
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (damp_params[i] & 0xFF));
			unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((damp_params[i] >> 8) & 0xFF));
		}

		// 10. Turn Limit (2 bytes: turn_limit)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (turn_limit & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((turn_limit >> 8) & 0xFF));

		// 11. Sensores de línea RAW (6 bytes: IR1, IR3, IR5)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[1] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[1] >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[3] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[3] >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (adcDataTx[5] & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((adcDataTx[5] >> 8) & 0xFF));

		// 12. Sensores de línea Calibrados (6 bytes: IR1, IR3, IR5)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_left_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_left_ir >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_center_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_center_ir >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (cal_right_ir & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((cal_right_ir >> 8) & 0xFF));

		// 13. Kd Pared / Anticipo (2 bytes: Kd_anticipo)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (Kd_anticipo & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((Kd_anticipo >> 8) & 0xFF));

		// 14. Kp Frontal / Preventivo Central (2 bytes: Kp_frontal)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (Kp_frontal & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((Kp_frontal >> 8) & 0xFF));

		// 15. Kd Frontal / Anticipación Guiñada Central (2 bytes: Kd_frontal)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) (Kd_frontal & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) ((Kd_frontal >> 8) & 0xFF));

		// 16. Modo Esquivado (2 bytes: dodge_config_mode)
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t) dodge_config_mode);
		unerPrtcl_PutByteOnTx(dataTx, 0);

		// Checksum final
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	case GETPIDBALANCE:
			// Tamaño: 1(cmd) + 7 variables * 4 bytes = 29 bytes
			unerPrtcl_PutHeaderOnTx(dataTx, GETPIDBALANCE, 29);

			int32_t pid_telemetry[7] = { error, integral, derivative, output, current_angle, turn_offset, measured_dt_ms };

			for (int i = 0; i < 7; i++) {
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
	case SETCORNERDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETCORNERDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_corner_dist = myWord.ui16[0];
		break;
	case SETALIGNDIST:
		unerPrtcl_PutHeaderOnTx(dataTx, SETALIGNDIST, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		obs_align_dist = myWord.ui16[0];
		break;
	case SETPWMLROT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMLROT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		PWM_LRot = myWord.ui16[0];
		break;
	case SETPWMRROT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETPWMRROT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		PWM_RRot = myWord.ui16[0];
		break;
	case SET_KP_EXT:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_KP_EXT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kp_ext = myWord.i16[0];
		break;
	case SETWALLKP:
		unerPrtcl_PutHeaderOnTx(dataTx, SETWALLKP, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kp_pared = myWord.i16[0];
		break;
	case SETWALLKD:
		unerPrtcl_PutHeaderOnTx(dataTx, SETWALLKD, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kd_anticipo = myWord.i16[0];
		break;
	case SETFRONTKP:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTKP, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kp_frontal = myWord.i16[0];
		break;
	case SETFRONTKD:
		unerPrtcl_PutHeaderOnTx(dataTx, SETFRONTKD, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Kd_frontal = myWord.i16[0];
		break;
	case SETLIMITANG:
		unerPrtcl_PutHeaderOnTx(dataTx, SETLIMITANG, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		break;
	case SET_KI_EXT:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_KI_EXT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		Ki_ext = myWord.i16[0];
		break;
	case SET_ALFA_LPF:
		unerPrtcl_PutHeaderOnTx(dataTx, SET_ALFA_LPF, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		alfa_lpf = myWord.i16[0];
		break;
	case SETVELDAMPDIV:
		unerPrtcl_PutHeaderOnTx(dataTx, SETVELDAMPDIV, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		vel_damp_div = myWord.i16[0];
		break;
	case SETVELDAMPLIM:
		unerPrtcl_PutHeaderOnTx(dataTx, SETVELDAMPLIM, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		vel_damp_limit = myWord.i16[0];
		break;
	case SETTURNLIMIT:
		unerPrtcl_PutHeaderOnTx(dataTx, SETTURNLIMIT, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		turn_limit = myWord.i16[0];
		break;
	case SETDODGEMODE:
		unerPrtcl_PutHeaderOnTx(dataTx, SETDODGEMODE, 2);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		myWord.ui8[0] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		myWord.ui8[1] = unerPrtcl_GetByteFromRx(dataRx, 1, 0);
		dodge_config_mode = (uint8_t)(myWord.ui16[0] & 0xFF);
		break;
	case EXPORTIRCSV: {
		// Incrementar contador de exportación (persistente en sesión)
		static uint16_t ir_csv_export_count = 0;
		ir_csv_export_count++;
		// Responder: EXPORTIRCSV + ACK + count_hi + count_lo
		unerPrtcl_PutHeaderOnTx(dataTx, EXPORTIRCSV, 4);
		unerPrtcl_PutByteOnTx(dataTx, ACK);
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t)((ir_csv_export_count >> 8) & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, (uint8_t)(ir_csv_export_count & 0xFF));
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
	default:
		unerPrtcl_PutHeaderOnTx(dataTx, (_eCmd) dataRx->buff[dataRx->indexData],
				2);
		unerPrtcl_PutByteOnTx(dataTx, UNKNOWN);
		unerPrtcl_PutByteOnTx(dataTx, dataTx->chk);
		break;
	}
}

void do10ms() {
	if (!IS10MS) return;
	IS10MS = FALSE;

	// --- 1. Tareas Periódicas de 10ms ---
	ESP01_Timeout10ms();
	UART_EnforceReceiverActive();
	WiFi_ScanTick();
	buttonTimeout10ms(&myButton);

	// --- 2. Divisor de Tiempo: 20ms ---
	tmo20ms--;
	if (!tmo20ms) {
		tmo20ms = 2;
		IS20MS = TRUE;
	}

	// --- 3. Divisor de Tiempo: 100ms ---
	tmo100ms--;
	if (!tmo100ms) {
		tmo100ms = 10;
		IS100MS = TRUE;

		// Tarea Periódica de 100ms
		heartBeatTask();

		// Tareas Periódicas de 1s (Encapsuladas en tick de 100ms)
		WiFi_HeartbeatTick();
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
	if (times >= 20) {
		times = 0; // Acotado a ciclo síncrono de exactly 2.0 segundos (20 ranuras x 100ms)
	}
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
	char data[32];
	uint8_t y = 0, x = 2;

	if (robotMode == STATE_FIRST_SCREEN) {
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
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gx);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		x = SSD1306_SNDCOL;
		y += 12;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", ay);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gy);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		x = SSD1306_SNDCOL;
		y += 12;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", az);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);
		x = SSD1306_TRDCOL;
		ssd1306_SetCursor(x, y);
		snprintf(data, sizeof(data), "%d", gz);
		ssd1306_WriteString(data, Font_7x10, (SSD1306_COLOR)Black);

		ssd1306_Line(3, 60, 3,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[0] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(6, 60, 6,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[1] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(9, 60, 9,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[2] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(12, 60, 12,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[3] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(15, 60, 15,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[4] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(18, 60, 18,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[5] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(21, 60, 21,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[6] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
		ssd1306_Line(24, 60, 24,
				(SSD1306_MINADC - ((uint32_t)adcDataTx[7] * SSD1306_MAXADC) / 4090), (SSD1306_COLOR)Black);
	} else {
		// New Screen: premium dark mode
		ssd1306_Fill(Black);

		// Row 1: VEL: <speed> mm/s (speed is integrated in mm/s)
		ssd1306_SetCursor(2, 2);
		ssd1306_WriteString("VEL:", Font_7x10, White);
		ssd1306_SetCursor(36, 2);
		snprintf(data, sizeof(data), "%ld mm/s", speed);
		ssd1306_WriteString(data, Font_7x10, White);

		// Row 2: ACL: <dynamic_accel * 3 / 5> mm/s2 (dynamic_accel is in LSB, 1 LSB ≈ 0.6 mm/s2)
		ssd1306_SetCursor(2, 17);
		ssd1306_WriteString("ACL:", Font_7x10, White);
		ssd1306_SetCursor(36, 17);
		snprintf(data, sizeof(data), "%ld mm/s2", (dynamic_accel * 3) / 5);
		ssd1306_WriteString(data, Font_7x10, White);

		// Row 3: ANG: <current_angle / 100>.<abs(current_angle % 100)>
		ssd1306_SetCursor(2, 32);
		ssd1306_WriteString("ANG:", Font_7x10, White);
		ssd1306_SetCursor(36, 32);
		int32_t ang_whole = current_angle / 100;
		int32_t ang_frac = current_angle % 100;
		if (ang_frac < 0) {
			ang_frac = -ang_frac;
		}
		snprintf(data, sizeof(data), "%ld.%02ld", ang_whole, ang_frac);
		ssd1306_WriteString(data, Font_7x10, White);

		// Row 4: BAL: <BALANCING / CAIDO> (si se esta balanceando o no)
		ssd1306_SetCursor(2, 47);
		ssd1306_WriteString("BAL:", Font_7x10, White);
		ssd1306_SetCursor(36, 47);
		if (current_angle < ANG45 && current_angle > -ANG45) {
			ssd1306_WriteString("BALANCING", Font_7x10, White);
		} else {
			ssd1306_WriteString("CAIDO", Font_7x10, White);
		}

		// Horizontal dividing lines in White
		ssd1306_Line(0, 14, 127, 14, White);
		ssd1306_Line(0, 29, 127, 29, White);
		ssd1306_Line(0, 44, 127, 44, White);
	}
}

void i2cTask() {
	static uint8_t i = IDLE;
	static uint8_t j = 0;
	static uint32_t mpu_timeout = 0; // NUEVO: Contador de paciencia

	// DESBLOQUEO DE SEGURIDAD: Forzar retorno a IDLE si estamos en modo movimiento para no colgar la lectura MPU
	if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
		if (i == DATA_DISPLAY || i == UPD_DISPLAY) {
			i = IDLE;
		}
	}

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

static void format_pitch_deg(char *out, size_t maxlen, int32_t pitch_hr) {
    int32_t abs_val = (pitch_hr < 0) ? -pitch_hr : pitch_hr;
    int32_t integer_part = abs_val / 10000;
    int32_t frac_part = (abs_val % 10000) / 100;
    snprintf(out, maxlen, "%s%ld.%02ld", (pitch_hr < 0 && (integer_part != 0 || frac_part != 0)) ? "-" : "", (long)integer_part, (long)frac_part);
}

static void format_yaw_deg(char *out, size_t maxlen, int32_t yaw_hr) {
    int32_t abs_val = (yaw_hr < 0) ? -yaw_hr : yaw_hr;
    int32_t integer_part = abs_val / 1000;
    int32_t frac_part = (abs_val % 1000) / 10;
    snprintf(out, maxlen, "%s%ld.%02ld", (yaw_hr < 0 && (integer_part != 0 || frac_part != 0)) ? "-" : "", (long)integer_part, (long)frac_part);
}

/**
 * @brief Envia telemetria en formato JSON para auto-refresh del dashboard
 */
_eESP01STATUS sendJSONTelemetry(uint8_t connID)
{
    char strPitch[16] = {0};
    char strYaw[16]   = {0};

    format_pitch_deg(strPitch, sizeof(strPitch), current_angle_hr);
    format_yaw_deg(strYaw, sizeof(strYaw), total_yaw_hr);

    const char *header = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";

    uint16_t len = (uint16_t)snprintf((char*)httpTxBuf, sizeof(httpTxBuf),
        "%s{\"irl\":%d,\"irc\":%d,\"irr\":%d,\"d0\":%d,\"d2\":%d,\"d4\":%d,\"d6\":%d,\"d7\":%d,\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d,\"pitch\":%s,\"yaw\":%s}",
        header,
        cal_left_ir, cal_center_ir, cal_right_ir,
        cal_ir0, cal_ir2, cal_ir4, cal_ir6, cal_ir7,
        ax, ay, az, gx, gy, gz,
        strPitch, strYaw);

    return ESP01_Send(connID, httpTxBuf, 0, len, sizeof(httpTxBuf));
}

/**
 * @brief Envia el Dashboard HTML con telemetria en vivo y formulario de configuracion Wi-Fi
 */
_eESP01STATUS sendHTMLForm(uint8_t connID)
{
    char strPitch[16] = {0};
    char strYaw[16]   = {0};

    format_pitch_deg(strPitch, sizeof(strPitch), current_angle_hr);
    format_yaw_deg(strYaw, sizeof(strYaw), total_yaw_hr);

    uint16_t len = (uint16_t)snprintf((char*)httpTxBuf, sizeof(httpTxBuf),
        "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
        "<pre id=p>IR: %d / %d / %d\n"
        "Frontal: %d\n"
        "Lateral: %d / %d / %d / %d\n"
        "Pitch/Yaw: %s / %s\n"
        "Acel: %d,%d,%d\n"
        "Giro: %d,%d,%d</pre>"
        "<form action=/set>"
        "SSID: <input name=ssid required><br>"
        "Pass: <input name=pass type=password><br>"
        "IP PC: <input name=ip value=192.168.0.10><br>"
        "Puerto: <input name=port value=30010><br>"
        "Proto: <select name=proto><option>UDP<option>TCP</select><br><br>"
        "<input type=submit value=\"Conectar\">"
        "</form>"
        "<script>setInterval(()=>fetch('/data').then(r=>r.json()).then(d=>p.innerText="
        "`IR: ${d.irl} / ${d.irc} / ${d.irr}\\n"
        "Frontal: ${d.d6}\\n"
        "Lateral: ${d.d2} / ${d.d4} / ${d.d0} / ${d.d7}\\n"
        "Pitch/Yaw: ${d.pitch}° / ${d.yaw}°\\n"
        "Acel: ${d.ax},${d.ay},${d.az}\\n"
        "Giro: ${d.gx},${d.gy},${d.gz}`),1000);</script>",
        cal_left_ir, cal_center_ir, cal_right_ir,
        cal_ir6,
        cal_ir2, cal_ir4, cal_ir0, cal_ir7,
        strPitch, strYaw,
        ax, ay, az,
        gx, gy, gz);

    return ESP01_Send(connID, httpTxBuf, 0, len, sizeof(httpTxBuf));
}

/**
 * @brief Envia pagina de confirmacion al navegador
 */
_eESP01STATUS sendHTTPOKPage(uint8_t connID)
{
    const char *header = "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
    const char *body   = "<!DOCTYPE html><html><body>"
                         "<h2>Conectando...</h2>"
                         "<p>El dispositivo se conectara a la red indicada.</p>"
                         "</body></html>";

    uint16_t len = (uint16_t)snprintf((char*)httpTxBuf, sizeof(httpTxBuf), "%s%s", header, body);
    return ESP01_Send(connID, httpTxBuf, 0, len, sizeof(httpTxBuf));
}

/**
 * @brief Extrae ssid, pass, ip y port del GET /set?ssid=X&pass=Y&ip=Z&port=W
 *        Decodificacion basica: '+' → espacio
 */
void parseHTTPGetParams(const char *httpReq, char *ssid, char *pass, char *ip, uint16_t *port, char *proto)
{
    const char *p;
    uint8_t i;

    ssid[0] = '\0';
    pass[0] = '\0';
    ip[0]   = '\0';
    *port   = 30010; /* valor por defecto */
    strcpy(proto, "UDP");

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

    /* ---- PROTO ---- */
    p = strstr(httpReq, "&proto=");
    if(p){ p += 7;
        if(strncmp(p, "TCP", 3) == 0 || strncmp(p, "tcp", 3) == 0)
            strcpy(proto, "TCP");
        else
            strcpy(proto, "UDP");
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

        /* Forzar udpSilenceCounter a 5 para que el primer ALIVE se envie
         * en cuanto el UDP quede conectado, sin esperar 5 ciclos de 1s. */
        udpSilenceCounter = 5;
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
void httpTask(void)
{
    /* Iniciar UDP/TCP en cuanto el WiFi este listo (viene del callback) */
    if(udpReadyToStart){
        udpReadyToStart = 0;
        if(strcmp(udpTargetProto, "TCP") == 0){
            ESP01_StartTCP(udpTargetIP, udpTargetPort, 30001);
        } else {
            ESP01_StartUDP(udpTargetIP, udpTargetPort, 30001);
        }
        return;
    }

    if(!isWebserverMode)
        return;

    /* 0xFF = primera linea completa y lista para procesar */
    if(httpBufIdx != 0xFF)
        return;

    uint8_t connID = ESP01_GetLastConnID();

    if(strstr(httpBuf, "GET /data") != NULL){
        if(sendJSONTelemetry(connID) == ESP01_SEND_READY){
            HTTP_BUF_RESET();
        }

    } else if(strstr(httpBuf, "GET /set?") != NULL){
        char newSSID[64]  = {0};
        char newPASS[64]  = {0};
        char newIP[16]    = {0};
        uint16_t newPort  = 30010;
        char newProto[4]  = "UDP";

        parseHTTPGetParams(httpBuf, newSSID, newPASS, newIP, &newPort, newProto);

        /* Guardar IP, puerto y protocolo para usarlos cuando conecte el WiFi */
        if(newIP[0] != '\0'){
            strncpy(udpTargetIP, newIP, 15);
            udpTargetIP[15] = '\0';
        }
        if(newPort > 0)
            udpTargetPort = newPort;
        if(newProto[0] != '\0')
            strcpy(udpTargetProto, newProto);

        /* Responder al navegador */
        if(sendHTTPOKPage(connID) == ESP01_SEND_READY){
            isWebserverMode = 0;
            HTTP_BUF_RESET();
            ESP01_SetWIFI(newSSID, newPASS);
        }

    } else if(strstr(httpBuf, "GET /") != NULL){
        if(sendHTMLForm(connID) == ESP01_SEND_READY){
            HTTP_BUF_RESET();
        }

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
    button->clickCount = 0;
    button->justReleased = FALSE;
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
                button->justReleased = TRUE; // Capturar flanco de subida de liberación
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
    static uint8_t timeToDebounce = 0;
    static uint8_t release_ticks = 0;
    static uint8_t was_pressed = FALSE;

    // Leer el estado físico instantáneo del pin
    _eEvent current_pin = (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_RESET) ? PRESSED : NOT_PRESSED;

    // Si el botón está actualmente presionado, aplicamos un filtro de liberación lento contra rebotes por vibración
    if (button->isPressed) {
        if (current_pin == NOT_PRESSED) {
            release_ticks++;
            if (release_ticks >= 8) { // Requiere 80ms estables de liberación física para confirmar soltado
                button->stateInput = NOT_PRESSED;
                release_ticks = 0;
            }
        } else {
            button->stateInput = PRESSED;
            release_ticks = 0;
        }
        timeToDebounce = 0; // Sincronizar
    } else {
        // Si no está presionado, se utiliza el debouncer rápido de 50ms original del proyecto
        release_ticks = 0;
        if(timeToDebounce > DEBOUNCE){
            timeToDebounce = 0;
            button->stateInput = current_pin;
        } else {
            timeToDebounce++;
        }
    }

    // Lógica unificada para el acumulador / temporizador "time"
    if (button->isPressed) {
        if (!was_pressed) {
            button->time = 0; // Reiniciar para empezar a cronometrar la pulsación actual desde cero
            was_pressed = TRUE;
        }
        button->time += 10;
    } else {
        was_pressed = FALSE;
        // Si no está presionado y hay clics acumulados, decrementamos la ventana de multiclic
        if (button->clickCount > 0 && button->time > 0) {
            if (button->time >= 10) {
                button->time -= 10;
            } else {
                button->time = 0;
            }
        } else if (button->clickCount == 0) {
            button->time = 0;
        }
    }
}

void buttonTask(_sButton *button) {
	// 1. Evaluar si la ventana de clics múltiples terminó (time llegó a 0) sin eventos de liberación pendientes
	if (!button->isPressed && button->time == 0 && button->clickCount > 0 && !button->justReleased) {
		// Retornar y reintentar en el próximo ciclo si el bus I2C está ocupado, previniendo bloqueos y jitter
		if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
			return;
		}

		switch (button->clickCount) {
			case 1:
				robotMode = STATE_SWING;
				hbIndex = 0; // LED Swing (1 parpadeo de 100ms)
				CHPD_Control(1);

				// Respaldar ganancias antes de hacerlas 0
				Kp_line_backup = Kp_line;
				Kq_line_backup = Kq_line;
				Kp_line = 0;
				Kq_line = 0;
				break;
			case 2:
				robotMode = STATE_LINE_FOLLOWING;
				hbIndex = 1; // LED Line Following (2 parpadeos)
				CHPD_Control(1);

				// Restaurar ganancias de seguimiento de línea si venían de estar en 0 (Swing)
				if (Kp_line == 0 && Kq_line == 0) {
					Kp_line = Kp_line_backup;
					Kq_line = Kq_line_backup;
				}
				break;
			case 3:
				robotMode = STATE_DODGE;
				dodgeState = DODGE_LINE_FOLLOWING; // Inicia en seguimiento de línea
				hbIndex = 2; // LED Dodge (3 parpadeos)
				CHPD_Control(1);

				// Restaurar ganancias de seguimiento de línea si venían de estar en 0 (Swing)
				if (Kp_line == 0 && Kq_line == 0) {
					Kp_line = Kp_line_backup;
					Kq_line = Kq_line_backup;
				}
				break;
			default:
				// Si por algún motivo hace más de 3 clics, vuelve a carrera por defecto
				robotMode = STATE_LINE_FOLLOWING;
				hbIndex = 1;
				CHPD_Control(1);

				// Restaurar ganancias de seguimiento de línea si venían de estar en 0 (Swing)
				if (Kp_line == 0 && Kq_line == 0) {
					Kp_line = Kp_line_backup;
					Kq_line = Kq_line_backup;
				}
				break;
		}
		button->clickCount = 0; // Resetear contador

		// Forzar refresco de pantalla para mantener encendido el texto del modo seleccionado
		lastMode = (_eRobotMode)-1;
	}

	// 2. Solo tomamos decisiones de temporización de pulsación ante un EVENTO DE LIBERACIÓN
	if (button->justReleased) {
		if (button->time >= T3000MS) {
			button->justReleased = FALSE; // Consumir el evento
			button->time = 0; // Sin acción por ahora, limpiar
		}
		else if (button->time >= T2000MS && button->time < T3000MS) {
			// Retornar y reintentar en el próximo ciclo si el bus I2C está ocupado, previniendo bloqueos
			if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				return;
			}
			button->justReleased = FALSE; // Consumir el evento

			// Pulsación >= 2s y < 3s -> STATE_SECOND_SCREEN
			robotMode = STATE_SECOND_SCREEN;
			CHPD_Control(1);
			ssd1306_ResetDMAState(); // SOLUCIÓN AL BUG: Reiniciar la máquina de estados DMA de la pantalla
			ssd1306_SetDisplayOn(1); // Encender pantalla
			hbIndex = 4;             // LED Premium (1000ms encendido)
			button->clickCount = 0;  // Abortar clics cortos pendientes
			button->time = 0;        // Limpiar
		}
		else if (button->time >= T1000MS && button->time < T2000MS) {
			// Retornar y reintentar en el próximo ciclo si el bus I2C está ocupado, previniendo bloqueos
			if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				return;
			}
			button->justReleased = FALSE; // Consumir el evento

			// Pulsación >= 1s y < 2s -> STATE_FIRST_SCREEN
			robotMode = STATE_FIRST_SCREEN;
			CHPD_Control(1);
			ssd1306_ResetDMAState(); // SOLUCIÓN AL BUG: Reiniciar la máquina de estados DMA de la pantalla
			ssd1306_SetDisplayOn(1); // Encender pantalla
			hbIndex = 3;             // LED RAW (500ms encendido)
			button->clickCount = 0;  // Abortar clics cortos pendientes
			button->time = 0;        // Limpiar
		}
		else if (button->time < T1000MS) {
			button->justReleased = FALSE; // Consumir el evento
			// Clic corto detectado (duración < 1s) -> Sumar al contador y arrancar ventana de 400ms reutilizando 'time'
			button->clickCount++;
			button->time = T400MS; 
		}
	}
}

void LineFollowingMEF(int32_t left_ir, int32_t center_ir, int32_t right_ir, int32_t *target_setpoint) {
	static uint16_t line_lost_debounce_count = 0;
	static int32_t last_turn_offset = 0;
	static int32_t line_lost_yaw = 0;
	static int8_t search_direction = 1;
	static int8_t cross_direction = 1;
	static int8_t last_active_sensor = -1; // -1: Sensor derecho en línea (rotar a derecha), +1: Sensor izquierdo (rotar a izquierda)

	uint8_t ir1_active = (left_ir < IR_WHITE);
	uint8_t ir3_active = (center_ir < IR_WHITE);
	uint8_t ir5_active = (right_ir < IR_WHITE);
	uint8_t active_count = ir1_active + ir3_active + ir5_active;

	switch (lineState) {
	case LINE_SEARCHING:
		*target_setpoint = attack_setpoint;
		if (ir3_active) {
			lineState = LINE_FOLLOWING;
		} else {
			int16_t active_turn = (custom_turn > 50) ? custom_turn : 350;
			turn_offset = -active_turn;
			last_turn_offset = turn_offset;
		}
		break;

	case LINE_FOLLOWING:
		// Detección de los 3 sensores en negro: activo EXCLUSIVAMENTE en modo seguimiento de línea (no en esquivar)
		if (robotMode == STATE_LINE_FOLLOWING && active_count == 3 && ir1_active && ir3_active && ir5_active) {
			// Sentido de giro aleatorio (+1: derecha, -1: izquierda) usando el jitter del SysTick
			cross_direction = ((SysTick->VAL & 1) != 0) ? 1 : -1;
			lineState = LINE_CROSS;
			break;
		}

		if (active_count == 0) {
			line_lost_debounce_count++;
			if (line_lost_debounce_count >= 6) {
				line_lost_debounce_count = 0;
				line_lost_timer = 0;
				line_lost_phase = LINE_LOST_ROT_90;
				line_lost_yaw = 0;
				// Iniciar rotación hacia el lado donde estuvo la línea por última vez:
				// Si salió el izquierdo primero (último fue el derecho) -> search_direction = -1 (rotar a la derecha)
				// Si salió el derecho primero (último fue el izquierdo) -> search_direction = +1 (rotar a la izquierda)
				search_direction = last_active_sensor;
				lineState = LINE_LOST;
				break;
			}
			error_linea = last_line_error;
			turn_offset = last_turn_offset;
		} else {
			line_lost_debounce_count = 0;

			// Registrar cuál sensor lateral estuvo sobre la línea por última vez
			if (ir5_active && !ir1_active) {
				last_active_sensor = -1; // Sensor derecho en negro (izquierdo salió primero -> rotar derecha)
			} else if (ir1_active && !ir5_active) {
				last_active_sensor = 1;  // Sensor izquierdo en negro (derecho salió primero -> rotar izquierda)
			} else if (right_ir < left_ir) {
				last_active_sensor = -1; // Analógico: sensor derecho más oscuro (más cerca de la línea)
			} else if (left_ir < right_ir) {
				last_active_sensor = 1;  // Analógico: sensor izquierdo más oscuro
			}

			error_linea = ((-(1000 * left_ir) + (1000 * right_ir)) / sum_sensors) / 10;
			abs_error = (error_linea > 0) ? error_linea : -error_linea;

			int32_t linear_term = (Kp_line * error_linea) / 100;
			int32_t quad_term = (Kq_line * error_linea * abs_error) / SCALE_LINE;

			turn_offset = linear_term + quad_term;
			if (turn_offset > turn_limit)        turn_offset = turn_limit;
			else if (turn_offset < -turn_limit)  turn_offset = -turn_limit;

			last_turn_offset = turn_offset;
		}
		last_line_error = error_linea;

		*target_setpoint = attack_setpoint;
		break;

	case LINE_LOST:
		// Verificar constantemente en cada ciclo si CUALQUIERA de los 3 sensores toca la línea
		if (active_count > 0) {
			lineState = LINE_FOLLOWING;
			break;
		}

		// Integración del ángulo de yaw usando el giroscopio Z (gz) y su offset
		{
			int32_t gz_cal = gz - gz_offset;
			line_lost_yaw += ((int64_t)gz_cal * DT_US) / 131000LL;
		}

		integral = (integral * 7) / 10; // Atenuación de memoria inercial para evitar desestabilización en el giro

		int32_t base_turn = 350;
		int32_t abs_error_lost = (error < 0) ? -error : error;

		switch (line_lost_phase) {
		case LINE_LOST_ROT_90:
			// Inclinación hacia adelante (-12.50°) con rotación y balanceo activo
			*target_setpoint = -1250;

			// Prioridad de balanceo dinámica con par de rotación firme
			if (abs_error_lost > 450) {
				turn_offset = 0;
				integral = 0;
			} else {
				// Rotar 90 grados hacia el lado donde se perdió la línea
				turn_offset = base_turn * search_direction;
			}
			{
				int32_t abs_yaw = (line_lost_yaw < 0) ? -line_lost_yaw : line_lost_yaw;
				if (abs_yaw >= 80000) { // Fin de giro (80° cortado antes para compensar inercia a 90°)
					turn_offset = 0;
					line_lost_yaw = 0;
					line_lost_timer = 0;
					line_lost_phase = LINE_LOST_ROT_180;
				}
			}
			break;

		case LINE_LOST_ROT_180:
			// Inclinación hacia adelante (-12.50°) con rotación y balanceo activo en sentido opuesto
			*target_setpoint = -1250;

			// Prioridad de balanceo dinámica con par de rotación firme
			if (abs_error_lost > 450) {
				turn_offset = 0;
				integral = 0;
			} else {
				// Rotar 180 grados en sentido opuesto
				turn_offset = base_turn * (-search_direction);
			}
			{
				int32_t abs_yaw = (line_lost_yaw < 0) ? -line_lost_yaw : line_lost_yaw;
				if (abs_yaw >= 180000) { // Fin de giro (180°)
					turn_offset = 0;
					line_lost_yaw = 0;
					line_lost_timer = 0;
					line_lost_phase = LINE_LOST_STOPPED;
				}
			}
			break;

		case LINE_LOST_STOPPED:
		default:
			// Si tras las rotaciones de 90° y 180° no se detecta la línea: mantener balanceo en el lugar a -7.50° (-750)
			*target_setpoint = -750;
			turn_offset = 0;
			break;
		}
		break;

	case LINE_CROSS:
		*target_setpoint = 350; // Setpoint erguido (+3.50°) para rotar sobre su propio eje

		// Salida: rotar hasta que quede SOLAMENTE UN sensor midiendo
		if (active_count == 1) {
			turn_offset = 0;
			lineState = LINE_FOLLOWING;
			break;
		}

		// Rotar sobre su eje en sentido aleatorio
		turn_offset = cross_direction * 350;
		break;

	default:
		lineState = LINE_SEARCHING;
		break;
	}
}

void PID_ControlTask(void) {
	if (RUN_PID == FALSE)
		return;
	RUN_PID = FALSE;

	// Variables estáticas persistentes de estado
	static int32_t last_angle = 0;

	measured_dt_ms = DT_MS;

	// =========================================================
	// --- 1. LECTURA DIRECTA DE SENSORES DE LÍNEA ---
	// =========================================================
	uint16_t raw_left   = adcDataTx[1];
	uint16_t raw_center = adcDataTx[3];
	uint16_t raw_right  = adcDataTx[5];

	// Normalización e interpolación directa por LUT
	uint16_t raw_sensors[4] = {
		raw_left,
		raw_center,
		raw_right,
		(uint16_t)((raw_left + raw_center + raw_right) / 3)
	};
	
	uint16_t norm_sensors[4];
	NormalizeLineSensors(raw_sensors, norm_sensors);

	int32_t left_ir   = norm_sensors[0];
	int32_t center_ir = norm_sensors[1];
	int32_t right_ir  = norm_sensors[2];

	cal_left_ir   = (int16_t)left_ir;
	cal_center_ir = (int16_t)center_ir;
	cal_right_ir  = (int16_t)right_ir;

	sum_sensors = left_ir + center_ir + right_ir;
	if (sum_sensors == 0)
		sum_sensors = 1;

	// Lectura directa de sensores de distancia superiores (esquivar objeto) sin normalización
	cal_ir0 = (int16_t)adcDataTx[0];
	cal_ir2 = (int16_t)adcDataTx[2];
	cal_ir4 = (int16_t)adcDataTx[4];
	cal_ir6 = (int16_t)adcDataTx[6];
	cal_ir7 = (int16_t)adcDataTx[7];

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
		acc_angle_hr = (int32_t) ax_filt * 35;
	}

	gyro_delta_hr = (-(int32_t) gy * 50) / 131;

	// Acumulación continua de Yaw
	int32_t gz_calibrated = gz - gz_offset;
	total_yaw_hr += ((int64_t)gz_calibrated * DT_US) / 131000LL;

		// Restauramos el filtro complementario puro. El acelerómetro DEBE
		// permanecer activo para corregir la deriva cruzada del giroscopio.
		current_angle_hr = (ALPHA_GYRO * (current_angle_hr + gyro_delta_hr)
				+ ALPHA_ACC * acc_angle_hr) / 1000;

		// Límites de seguridad extremos por software (+/- 90 grados)
		if (current_angle_hr > 900000)  current_angle_hr = 900000;
		if (current_angle_hr < -900000) current_angle_hr = -900000;

		current_angle = current_angle_hr / 100;
	// =========================================================
	// --- 3. MÁQUINA DE ESTADOS: DIRECCIÓN / DODGE ROTATION ---
	// =========================================================
	int32_t target_setpoint = setpoint; 
	turn_offset = 0;

	switch (robotMode) {
	case STATE_SWING:
		// --- MODO 1: BALANCEO ESTATICO ---
		turn_offset = 0;
		target_setpoint = setpoint;
		break;

	case STATE_LINE_FOLLOWING:
		// --- MODO 2: SEGUIMIENTO DE LÍNEA ---
		LineFollowingMEF(left_ir, center_ir, right_ir, &target_setpoint);
		break;

	case STATE_DODGE:
		static _eDodgeSubState standby_next_state = DODGE_ROTATING;

		switch (dodgeState) {
		case DODGE_LINE_FOLLOWING:
			// 1. Seguimiento de línea normal continuo a velocidad máxima
			LineFollowingMEF(left_ir, center_ir, right_ir, &target_setpoint);

			// Cuando se detecta el obstáculo a la distancia adecuada
			if (cal_ir6 >= 500) {
				standby_next_state = DODGE_ROTATING;
				turn_offset = 0;
				dodge_timer = 0;
				dodgeState = DODGE_STANDBY;

				// Determinar la dirección de esquivado según configuración (0: Der, 1: Izq, 2: Alternado)
				if (dodge_config_mode == 0) {
					dodge_direction = -1; // Siempre DERECHA
				} else if (dodge_config_mode == 1) {
					dodge_direction = 1;  // Siempre IZQUIERDA
				} else { // 2: ALTERNADO
					dodge_direction = dodge_alt_next;
					dodge_alt_next = -dodge_alt_next; // Alternar para la próxima maniobra
				}
			}
			break;

		case DODGE_STANDBY:
			// Frenado estándar y estabilización erguida antes de rotar
			dodge_timer += DT_MS;
			turn_offset = 0; // Frenado recto y balanceo quieto en el lugar

			if (dodge_timer < 1000) {
				target_setpoint = 750;  // Frenado moderado (+7.50°) durante el primer 1.0s
			} else if (dodge_timer < 3000) {
				target_setpoint = -250; // Estabilización inclinada (-2.50°) durante los 2.0s restantes
			} else {
				// Finalizados los 3.0s de standby
				dodge_timer = 0;
				dodge_yaw = 0;
				if (standby_next_state == DODGE_LINE_RECOVER_ROT90) {
					// Si algún sensor ya ve la línea negra, no rotar y pasar directo a seguimiento
					if (left_ir < IR_WHITE || center_ir < IR_WHITE || right_ir < IR_WHITE) {
						turn_offset = 0;
						lineState = LINE_FOLLOWING;
						dodgeState = DODGE_LINE_FOLLOWING;
					} else {
						dodgeState = DODGE_LINE_RECOVER_ROT90;
					}
				} else {
					dodgeState = standby_next_state;
				}
			}
			break;

		case DODGE_ROTATING: {
			// Rotación directa de 80° con giroscopio (la espera y frenado previo se realizaron en DODGE_STANDBY)
			target_setpoint = -250; // Inclinación (-2.50°) durante la rotación para buena adherencia
			integral = (integral * 7) / 10; // Atenuación de memoria inercial

			int32_t gz_calibrated = gz - gz_offset;
			dodge_yaw += ((int64_t)gz_calibrated * DT_US) / 131000LL;
			int32_t abs_yaw = (dodge_yaw < 0) ? -dodge_yaw : dodge_yaw;

			if (abs_yaw >= 75000) { // Fin de giro (75° cortado para compensar inercia a 90°)
				turn_offset = 0;
				dodge_yaw = 0;
				dodge_timer = 0;
				dodgeState = DODGE_WALL_FOLLOWING;
			} else {
				// Prioridad de balanceo dinámica con par de rotación firme
				int32_t base_turn = 350;
				int32_t abs_error = (error < 0) ? -error : error;

				if (abs_error > 450) {
					turn_offset = 0;
					integral = 0;
				} else {
					turn_offset = base_turn * dodge_direction;
				}
			}
			break;
		}

		case DODGE_WALL_FOLLOWING: {
					// Inicialización y sincronización de ciclo previo para anticipación frontal
					static int16_t last_cal_ir6 = 0;
					if (dodge_timer <= DT_MS) {
						last_cal_ir6 = cal_ir6;
					}

					// Sensores de pared: 90° para control principal de distancia, 45° para anticipación anticipada
					int16_t sensor_90 = (dodge_direction == 1) ? cal_ir0 : cal_ir2;
					int16_t sensor_45 = (dodge_direction == 1) ? cal_ir7 : cal_ir4;

					// Distancia objetivo principal a la pared
					int16_t wall_target = 800;

					// 1. Error de distancia del sensor lateral 90° (Control Principal)
					int32_t error_distancia = sensor_90 - wall_target;

					// 2. Error de anticipación del sensor diagonal 45° (Ayuda anticipada diagonal)
					int32_t error_anticipo = sensor_45 - wall_target;

					// 3. Control preventivo frontal proporcional (cal_ir6) con menor factor
					int16_t front_margin = 300;
					int32_t error_frontal = (cal_ir6 > front_margin) ? (cal_ir6 - front_margin) : 0;

					// 4. Anticipación de guiñada del sensor central (Derivada / Tasa de aproximación)
					int32_t deriv_frontal = cal_ir6 - last_cal_ir6;
					last_cal_ir6 = cal_ir6;
					int32_t anticipo_frontal = (deriv_frontal > 0) ? (deriv_frontal * Kd_frontal) : 0;

					// Fusión en el cálculo PD continuo (todos los términos alejan activamente al robot de la pared/obstáculo)
					int32_t calculo_pd = ((error_distancia * Kp_pared) 
					                    + (error_anticipo * Kd_anticipo) 
					                    + (error_frontal * Kp_frontal) 
					                    + anticipo_frontal) / 100;

					turn_offset = calculo_pd * dodge_direction;

					int16_t wall_turn_limit = 500;

					if (turn_offset > wall_turn_limit)        turn_offset = wall_turn_limit;
					else if (turn_offset < -wall_turn_limit)  turn_offset = -wall_turn_limit;

					// --- ARREGLO DE REVERSA Y FRENADO PREVENTIVO CONTRA CHOQUES ---
					// En caso de aproximación crítica detectada por el sensor frontal o de 45°,
					// se interrumpe la inclinación de avance (-1400) e inyecta inclinación positiva
					// para desacelerar o retroceder activamente mientras gira alejándose.
					if (cal_ir6 >= 550 || sensor_45 >= 1100) {
						target_setpoint = 350; // Frenado / retroceso activo (+3.50°) ante proximidad crítica
					} else if (cal_ir6 >= 380 || sensor_45 >= 920) {
						target_setpoint = -600; // Desaceleración suave (-6.00°) ante acercamiento moderado
					} else {
						target_setpoint = -1400; // Crucero normal de seguimiento de pared (-14.00°)
					}

					dodge_timer += DT_MS;

					// Medida de seguridad: Detección de línea negra tras al menos 5 segundos (5000 ms) de seguimiento de pared
					if (dodge_timer >= 5000) {
						if (left_ir < IR_WHITE || center_ir < IR_WHITE || right_ir < IR_WHITE) {
							standby_next_state = DODGE_LINE_RECOVER_ROT90;
							turn_offset = 0;
							dodge_timer = 0;
							dodge_yaw = 0;
							dodgeState = DODGE_STANDBY;
						}
					}
					break;
				}

		case DODGE_LINE_RECOVER_ROT90: {
			// Rotación sobre el lugar manteniendo setpoint de balanceo (-2.50°) igual a DODGE_STANDBY / DODGE_ROTATING
			target_setpoint = -250;
			integral = (integral * 7) / 10; // Atenuación de memoria inercial

			int32_t gz_cal = gz - gz_offset;
			dodge_yaw += ((int64_t)gz_cal * DT_US) / 131000LL;
			int32_t abs_yaw = (dodge_yaw < 0) ? -dodge_yaw : dodge_yaw;

			// Fin de rotación si se alcanzaron los 75° O si se detecta la línea con algún sensor
			if (abs_yaw >= 75000 || left_ir < IR_WHITE || center_ir < IR_WHITE || right_ir < IR_WHITE) {
				turn_offset = 0;
				dodge_yaw = 0;
				dodge_timer = 0;
				lineState = LINE_FOLLOWING;
				dodgeState = DODGE_LINE_FOLLOWING;
			} else {
				// Prioridad de balanceo dinámica con par de rotación firme a la DERECHA (-350)
				int32_t base_turn = 350;
				int32_t abs_error = (error < 0) ? -error : error;

				if (abs_error > 450) {
					turn_offset = 0;
					integral = 0;
				} else {
					turn_offset = base_turn * dodge_direction; // Rotación simétrica de reingreso (Der: -350, Izq: +350)
				}
			}
			break;
		}

		default:
			dodgeState = DODGE_WALL_FOLLOWING;
			break;
		}
		break;

	default:
		// Para otros modos o estados no definidos
		turn_offset = 0;
		target_setpoint = setpoint;
		break;
	}

	// --- CONTROL DE PREVENCIÓN DE CAÍDA TRASERA (Lógica del Gatillo) ---
	static uint8_t backwards_recovery_active = 0;
	if (robotMode == STATE_SWING || (robotMode == STATE_LINE_FOLLOWING && lineState != LINE_LOST)) {
		if (current_angle > 0) { // Si la inclinación trasera supera 0.00° (0)
			backwards_recovery_active = 1;
		} else if (current_angle <= 0) {
			backwards_recovery_active = 0;
		}

		if (backwards_recovery_active) {
			target_setpoint = 0; // Desactivar avance agresivo instantáneamente
		}
	} else if (lineState == LINE_LOST) {
		backwards_recovery_active = 0;
	}

	// --- CONTROL DE PREVENCIÓN DE CAÍDA DELANTERA EN MODO SWING ---
	static uint8_t forwards_recovery_active = 0;
	if (robotMode == STATE_SWING) {
		if (current_angle < -ANG15) { // Si la inclinación delantera en Swing excede -15.00° (-1500)
			forwards_recovery_active = 1;
		} else if (current_angle >= -ANG10) { // Histéresis: se desactiva al volver a -10.00° (-1000)
			forwards_recovery_active = 0;
		}

		if (forwards_recovery_active) {
			target_setpoint = 0; // Elimina el setpoint de avance en picada
		}
	}

	// =========================================================
	// --- LAZO DE CONTROL EN CASCADA EXTERNO (Cada 40ms) ---
	// =========================================================
	if (robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
		static uint8_t slow_loop_counter = 0;
		slow_loop_counter++;
		if (slow_loop_counter >= 8) { // 8 * 5ms = 40ms
			slow_loop_counter = 0;

			// 1. Filtro pasa-bajos sobre la acción de control del motor (output)
			// Nota: 'output' no incluye el minPWM, por lo que su valor base es bajito.
			pwm_filtrado = (alfa_lpf * output + (100 - alfa_lpf) * pwm_filtrado) / 100;

			// 2. Error de esfuerzo (Velocidad deseada es attack_setpoint, esfuerzo real es pwm_filtrado)
			int32_t effort_error = (int32_t)attack_setpoint - pwm_filtrado;

			// 3. Acumulación con límites anti-windup estrictos (+/- 50000)
			integral_esfuerzo += effort_error;
			if (integral_esfuerzo > 50000)  integral_esfuerzo = 50000;
			if (integral_esfuerzo < -50000) integral_esfuerzo = -50000;

			// 4. Salida PI (escalada a x100 para coincidir con el setpoint)
			int32_t prop_term = (Kp_ext * effort_error) / 1000;
			int32_t int_term = (Ki_ext * integral_esfuerzo) / 10000;
			int32_t out_pi = prop_term + int_term;

			// Saturación del modificador a un límite de +/- 15.00 grados (1500 centésimas)
			if (out_pi > 1500)  out_pi = 1500;
			if (out_pi < -1500) out_pi = -1500;

			angulo_modificador_pi = (int16_t)out_pi;
		}

		// Sumar el modificador dinámico al target_setpoint sólo si no estamos en rotación de recuperación
		if (lineState != LINE_LOST) {
			target_setpoint += angulo_modificador_pi;
		} else {
			integral_esfuerzo = 0;
			angulo_modificador_pi = 0;
		}
	} else {
		// Reiniciar variables si no estamos en modo seguidor de línea
		pwm_filtrado = 0;
		integral_esfuerzo = 0;
		angulo_modificador_pi = 0;
	}

	// =========================================================
	// --- 4. LAZO PID CENTRAL (Equilibrio Balancín Puro) ---
	// =========================================================
	error = target_setpoint - current_angle;
	// Derivada sobre la medición (Derivative on Measurement) para evitar el Derivative Kick
	derivative = (int32_t)(((int64_t)(last_angle - current_angle) * 1000000LL) / DT_US);

	if (error > -150 && error < 150) {
		integral += (error * (int32_t)DT_US) / 1000;
		if (integral > (ANG20 * 20))  integral = (ANG20 * 20);
		if (integral < -(ANG20 * 20)) integral = -(ANG20 * 20);
	} else {
		integral = (integral * 8) / 10;
	}

	output = (Kp_stable * error + (Ki_stable * integral) / 1000
			+ (Kd_stable * derivative)) / 10000;
	last_error = error;
	last_angle = current_angle;


	// =========================================================
	// --- 5. MEZCLA DE MOTORES (Mezclador de Velocidades y PID) ---
	// =========================================================
	int32_t pwm_left = 0;
	int32_t pwm_right = 0;

	// Detectamos si el robot está en búsqueda pivot sobre propio eje (LINE_CROSS restringido exclusivamente a STATE_LINE_FOLLOWING)
	// LINE_LOST se procesa en el MEZCLADOR GENERAL (idéntico a DODGE_ROTATING) para mantener activo el PID de balanceo (+250) mientras rota
	uint8_t is_rotating_pivot = ((robotMode == STATE_LINE_FOLLOWING && (lineState == LINE_SEARCHING || lineState == LINE_CROSS)) ||
	                             (robotMode == STATE_DODGE && lineState == LINE_SEARCHING));

	if (is_rotating_pivot) {
		// --- ROTACIÓN PIVOT SOBRE PROPIO EJE (Búsqueda de línea) ---
		int32_t raw_L = output + turn_offset;
		int32_t raw_R = output - turn_offset;

		uint16_t rot_min_L = (lineState == LINE_CROSS) ? 770 : PWM_LRot;
		uint16_t rot_min_R = (lineState == LINE_CROSS) ? 750 : PWM_RRot;

		if (raw_L > 0)       pwm_left = raw_L + rot_min_L + offset_left;
		else if (raw_L < 0)  pwm_left = raw_L - rot_min_L - offset_left;

		if (raw_R > 0)       pwm_right = raw_R + rot_min_R + offset_right;
		else if (raw_R < 0)  pwm_right = raw_R - rot_min_R - offset_right;

	} else {
		// --- MEZCLADOR GENERAL DE VELOCIDADES DE MOTORES (DODGE_ROTATING_90, LINE_LOST y Translación) ---
		// Vincula la rotación con el mezclador de velocidades para mantener la respuesta del PID de balanceo (output) alrededor del setpoint (+250)
		uint16_t active_minPWM_Left = minPWM_Left;
		uint16_t active_minPWM_Right = minPWM_Right;

		int32_t base_L = 0;
		int32_t base_R = 0;

		if (output > 0) {
			base_L = output + active_minPWM_Left + offset_left;
			base_R = output + active_minPWM_Right + offset_right;
		} else if (output < 0) {
			base_L = output - active_minPWM_Left - offset_left;
			base_R = output - active_minPWM_Right - offset_right;
		}

		// Mezcla diferencial uniforme de giro sin invertir dirección ni perder el setpoint PID
		if (turn_offset != 0) {
			if (base_L == 0) base_L = (turn_offset < 0) ? active_minPWM_Left : -active_minPWM_Left;
			if (base_R == 0) base_R = (turn_offset < 0) ? -active_minPWM_Right : active_minPWM_Right;

			pwm_left = base_L - turn_offset;
			pwm_right = base_R + turn_offset;
		} else {
			pwm_left = base_L;
			pwm_right = base_R;
		}
	}
	// --- IMPULSO DIRECTO DE RECUPERACIÓN (Bypass del Balanceo) ---
	if (backwards_recovery_active) {
		pwm_left = -3000;  // Impulso directo marcha atrás controlado (30% duty cycle)
		pwm_right = -3000;
		integral = 0;      // Resetear integrador para evitar descontrol al volver a balancear
		last_error = 0;
	} else if (forwards_recovery_active) {
		pwm_left = 3000;   // Impulso directo marcha adelante en modo SWING (30% duty cycle)
		pwm_right = 3000;
		integral = 0;      // Resetear integrador para evitar descontrol al volver a balancear
		last_error = 0;
	}

	// Integración de velocidad (estimación interna de telemetría extraída)
	Speed_IntegrationTask(DT_US);

	// =========================================================
	// --- 6. PROTECCIÓN ABSOLUTA Y APAGADO POR CAÍDA (> 45°) ---
	// =========================================================
	if (current_angle > ANG45 || current_angle < -ANG45) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
		speed = 0;
	}

	// Silenciado de motores durante calibración inicial (primeros 3 segundos)
	if (calib_cycle < 150) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
	}

	// Saturación final al límite de PWM
	if (pwm_left > (int32_t) maxPWM)  pwm_left = (int32_t) maxPWM;
	if (pwm_left < -(int32_t) maxPWM) pwm_left = -(int32_t) maxPWM;
	if (pwm_right > (int32_t) maxPWM)  pwm_right = (int32_t) maxPWM;
	if (pwm_right < -(int32_t) maxPWM) pwm_right = -(int32_t) maxPWM;

	if (robotMode != STATE_SWING && robotMode != STATE_LINE_FOLLOWING && robotMode != STATE_DODGE) {
		pwm_left = 0;
		pwm_right = 0;
		integral = 0;
		speed = 0;
	}

	// =========================================================
	// --- 7. MAPEO AL HARDWARE ---
	// =========================================================
	if (pwm_left > 0) {
		rPulse4 = (uint16_t) pwm_left;
		lPulse3 = 0;
	} else {
		lPulse3 = (uint16_t) (-pwm_left);
		rPulse4 = 0;
	}

	if (pwm_right > 0) {
		rPulse2 = (uint16_t) pwm_right;
		lPulse1 = 0;
	} else {
		lPulse1 = (uint16_t) (-pwm_right);
		rPulse2 = 0;
	}
}

static uint16_t LUT_Interpolate(const uint16_t *x, const uint16_t *lut_y, uint16_t raw)
{
    if(raw <= x[0])
        return lut_y[0];

    // Se asume que LUT_SIZE está definido globalmente (ej. #define LUT_SIZE 16)
    if(raw >= x[LUT_SIZE - 1])
        return lut_y[LUT_SIZE - 1];

    for(int i = 0; i < LUT_SIZE - 1; i++)
    {
        if(raw >= x[i] && raw <= x[i + 1])
        {
            uint32_t diff_x = x[i + 1] - x[i];
            uint32_t diff_y = lut_y[i + 1] - lut_y[i];
            uint32_t offset_x = raw - x[i];

            // Aritmética entera pura.
            // Se suma (diff_x / 2) antes de la división para emular el redondeo (+0.5)
            uint32_t y = lut_y[i] + (((offset_x * diff_y) + (diff_x / 2)) / diff_x);

            return (uint16_t)y;
        }
    }

    return lut_y[LUT_SIZE - 1];
}

/**
 * @brief Normaliza los sensores de línea usando las Look-Up Tables.
 * @param adcDataTx_ptr Puntero al buffer DMA seguro (adcDataTx).
 * @param norm Puntero al array de valores normalizados de salida.
 */
void NormalizeLineSensors(const uint16_t *adcDataTx_ptr, uint16_t *norm)
{
    // Se añade lut_y como parámetro para independizar la función
    norm[0] = LUT_Interpolate(lut_l1_x, lut_l1_y, adcDataTx_ptr[0]);
    norm[1] = LUT_Interpolate(lut_l2_x, lut_l2_y, adcDataTx_ptr[1]);
    norm[2] = LUT_Interpolate(lut_l3_x, lut_l3_y, adcDataTx_ptr[2]);
    norm[3] = LUT_Interpolate(lut_l4_x, lut_l4_y, adcDataTx_ptr[3]);
}



void HandleModeScreenTransition(void) {
	if (robotMode != lastMode) {
		lastMode = robotMode;

		if (robotMode == STATE_SWING || robotMode == STATE_LINE_FOLLOWING || robotMode == STATE_DODGE) {
			// Esperar a que el bus I2C esté listo
			while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
				// Espera activa segura
			}

			ssd1306_SetDisplayOn(1);
			ssd1306_Fill(Black);

			if (robotMode == STATE_SWING) {
				// Centrar ">>>MODO 1 -  BALANCEO<<<"
				// Primera línea: ">>>MODO 1<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 1<<<", Font_7x10, White);
				// Segunda línea: "BALANCEO" (8 caracteres)
				ssd1306_SetCursor((128 - 8 * 7) / 2, 34);
				ssd1306_WriteString("BALANCEO", Font_7x10, White);
			} else if (robotMode == STATE_LINE_FOLLOWING) {
				// Centrar ">>>MODO 2 - SEGUIMIENTO LINEA<<<"
				// Primera línea: ">>>MODO 2<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 2<<<", Font_7x10, White);
				// Segunda línea: "SEGUIMIENTO LINEA" (17 caracteres)
				ssd1306_SetCursor((128 - 17 * 7) / 2, 34);
				ssd1306_WriteString("SEGUIMIENTO LINEA", Font_7x10, White);
			} else if (robotMode == STATE_DODGE) {
				// Centrar ">>>MODO 3 - ESQUIVAR<<<"
				// Primera línea: ">>>MODO 3<<<" (12 caracteres)
				ssd1306_SetCursor((128 - 12 * 7) / 2, 20);
				ssd1306_WriteString(">>>MODO 3<<<", Font_7x10, White);
				// Segunda línea: "ESQUIVAR" (8 caracteres)
				ssd1306_SetCursor((128 - 8 * 7) / 2, 34);
				ssd1306_WriteString("ESQUIVAR", Font_7x10, White);
			}

			ssd1306_UpdateScreen();
		}
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
	ssd1306_SetDisplayOn(0); // Apagar pantalla inicialmente (modo sigue línea activo)

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


  	/* ---- MODO WEBSERVER / SOFTAP (Comentado para fase de pruebas directas en TCP) ----
  	 * Conectarse con el teléfono o PC a la red "MICRO" (contraseña: 12345678)
  	 * y navegar a 192.168.4.1 para ingresar el SSID y contraseña del router.
  	 * Para la versión final se puede reactivar descomentando las 2 líneas siguientes: */
  	// isWebserverMode = TRUE;
  	// ESP01_SetWebServer("MICRO", "12345678", 5, 3);
  	isWebserverMode = FALSE;

  	/* ---- AUTO-SCAN / CONEXIÓN DIRECTA WIFI (Modo pruebas TCP) ----
  	 * Conexión automática directa a las redes guardadas (knownNetworks) usando protocolo TCP. */
  	currentNetworkIdx = 0;
  	networkScanTimer = SCANTIME;
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

    // Habilitar contador de ciclos DWT para medición inercial de alta resolución (microsegundos)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
	COMMTask(&WiFiRx, &WiFiTx, WIFI);

	COMMTask(&USBRx, &USBTx, SERIE);

	PWM_Control();
	i2cTask();

	PID_ControlTask();

	updateMefTask(&myButton);
	buttonTask(&myButton);
	HandleModeScreenTransition();

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
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
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
  htim2.Init.Period = 4999;
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

/**
 * @brief Tarea de integración y estimación inercial de la velocidad y aceleración lineal (Telemetría interna).
 * @param dt_us Delta-time medido en microsegundos de la ejecución del bucle inercial.
 */
void Speed_IntegrationTask(uint32_t dt_us) {
    static int32_t ax_calib_sum = 0;
    static int32_t gz_calib_sum = 0;
    static int32_t last_gz_filt = 0;
    static int32_t gx_filt = 0, gy_filt = 0, gz_filt = 0;
    static int32_t ax_tele_filt = 0;

    // Filtro pasa-bajos para las velocidades angulares (giroscopio)
    if (gx_filt == 0 && gy_filt == 0 && gz_filt == 0) {
        gx_filt = gx;
        gy_filt = gy;
        gz_filt = gz;
    } else {
        gx_filt = (gx * 10 + gx_filt * 90) / 100;
        gy_filt = (gy * 10 + gy_filt * 90) / 100;
        gz_filt = (gz * 10 + gz_filt * 90) / 100;
    }

    // Filtro pasa-bajos rápido de baja latencia para el acelerómetro (telemetría fina)
    if (ax_tele_filt == 0) {
        ax_tele_filt = ax;
    } else {
        ax_tele_filt = (ax * 50 + ax_tele_filt * 50) / 100;
    }

    // Compensación trigonométrica de la aceleración de gravedad debido al ángulo de inclinación
    int32_t ang = current_angle;
    int32_t linear_part = (ang * 286) / 100;
    int64_t ang64 = ang;
    int32_t cubic_part = (int32_t)((ang64 * ang64 * ang64) / 68880962LL);
    int32_t gravity_comp = linear_part - cubic_part;

    // FASE DE CALIBRACIÓN INICIAL E INTEGRACIÓN INERCIAL (Primeros 3 segundos)
    if (calib_cycle < 150) {
        if (calib_cycle < 40) {
            // Acoplamiento rápido inicial de filtros para eliminar el retardo (lag)
            ax_filt = ax;
            az_filt = az;
            gx_filt = gx;
            gy_filt = gy;
            gz_filt = gz;
            last_gz_filt = gz;
            current_angle_hr = (int32_t)ax_filt * 35;
            current_angle = current_angle_hr / 100;
            ang = current_angle;
            linear_part = (ang * 286) / 100;
            ang64 = ang;
            cubic_part = (int32_t)((ang64 * ang64 * ang64) / 68880962LL);
            gravity_comp = linear_part - cubic_part;
            ax_tele_filt = ax;
        }

        if (calib_cycle >= 50) {
            // Promedio móvil para calibración del offset en reposo
            ax_calib_sum += (ax_tele_filt - gravity_comp);
            gz_calib_sum += gz;
        }
        calib_cycle++;
        if (calib_cycle == 150) {
            ax_offset = ax_calib_sum / 100; // Offset guardado
            gz_offset = gz_calib_sum / 100; // Offset de giroscopio Z guardado
        }
        dynamic_accel = 0;
        speed = 0;
        last_gz_filt = gz_filt;
    } else {
        // MODO OPERACIÓN NORMAL (Cálculo tridimensional de fuerzas centrífugas y tangenciales)
        int32_t rot_Y_sq = (int32_t)gy_filt * gy_filt;
        int32_t rot_Z_sq = (int32_t)gz_filt * gz_filt;
        int32_t rot_XY   = (int32_t)gx_filt * gy_filt;
        int32_t alpha_z = gz_filt - last_gz_filt;
        last_gz_filt = gz_filt;

        // Compensación de fuerzas parásitas por desplazamiento físico del MPU
        int32_t rx_centrifugal = (rot_Y_sq + rot_Z_sq) / 2249000;
        int32_t ry_centrifugal = rot_XY / 2249000;
        int32_t ry_tangential  = alpha_z / 6;

        // Aceleración lineal resultante
        dynamic_accel = ax_tele_filt - ax_offset - gravity_comp - rx_centrifugal + ry_centrifugal + ry_tangential;

        // Puerta de ruido (deadband) para evitar deriva inercial remanente
        int32_t accel_for_integration = dynamic_accel;
        if (accel_for_integration > -1300 && accel_for_integration < 1300) {
            accel_for_integration = 0;
        }

        // Integración física en velocidad mm/s
        speed = (speed * 98) / 100 + (int32_t)(((int64_t)accel_for_integration * dt_us * 3LL) / 5000000LL);
    }
}

static void WiFi_ScanTick(void) {
	if (!networkScanActive) return;

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

static void UART_EnforceReceiverActive(void) {
	if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
		uint32_t er = huart1.Instance->SR;
		uint32_t dr = huart1.Instance->DR;
		(void) er;
		(void) dr;
		huart1.RxState = HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, &byteUART_ESP01, 1);
	}
}

static void WiFi_HeartbeatTick(void) {
	timerUDP++;
	if (timerUDP >= 10) { // Entrar cada 10 ciclos de 100ms (1000ms o 1s)
		timerUDP = 0;

		// Incrementar contador de silencio WiFi (saturar en 5)
		if (udpSilenceCounter < 5)
			udpSilenceCounter++;

		/* Enviar ALIVE solo si la PC no esta comunicandose activamente. */
		if (!isWebserverMode && ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED
				&& udpSilenceCounter >= 5) {
			static uint8_t bufferTx[9] = { 'U', 'N', 'E', 'R', 0x03, ':', ALIVE, ACK, 0x98 };
			ESP01_Send(0, bufferTx, 0, 9, TXBUFSIZE);
		}
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
