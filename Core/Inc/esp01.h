/**
 * @file    esp01.h
 * @author  Germán E. Hachmann / Gonzalo M. Buffa
 * @date    04/08/2024
 * @brief   Driver de comunicación por comandos AT para módulo Wi-Fi ESP8266 (ESP-01).
 * @details Este módulo implementa el control asíncrono del transceptor Wi-Fi ESP-01 a través de USART,
 *          gestionando la máquina de estados de conexión a puntos de acceso (Modo Station), la creación
 *          de redes locales (Modo SoftAP), la apertura de sockets UDP/TCP para telemetría y el servidor
 *          HTTP embebido para configuración remota.
 * @ingroup group_comm
 */

#ifndef INC_ESP01_H_
#define INC_ESP01_H_

#include <stdint.h>

/**
 * @brief Estados operativos de la máquina de estados finita del driver ESP01.
 */
typedef enum{
	ESP01_NOT_INIT = -1,           /*!< Driver no inicializado o en proceso de reinicio hardware */
	ESP01_WIFI_DISCONNECTED,       /*!< Desconectado de la red Wi-Fi */
	ESP01_WIFI_NOT_SETED,          /*!< Sin credenciales de red Wi-Fi configuradas */
	ESP01_WIFI_CONNECTING_WIFI,    /*!< Intentando asociarse al punto de acceso (SSID/Password) */
	ESP01_WIFI_CONNECTED,          /*!< Conectado exitosamente al punto de acceso Wi-Fi */
	ESP01_WIFI_NEW_IP,             /*!< Dirección IP asignada por DHCP obtenida */
	ESP01_UDPTCP_DISCONNECTED,     /*!< Socket UDP/TCP cerrado o desconectado */
	ESP01_UDPTCP_CONNECTING,       /*!< Estableciendo socket de comunicación UDP/TCP con el host remoto */
	ESP01_UDPTCP_CONNECTED,        /*!< Socket UDP/TCP conectado y listo para transferir datos */
	ESP01_SEND_BUSY,               /*!< Transmisión de datos en progreso por la USART */
	ESP01_SEND_READY,              /*!< Buffer disponible y módulo listo para recibir nuevo comando de envío */
	ESP01_SEND_OK,                 /*!< Confirmación de paquete enviado exitosamente (`SEND OK`) */
	ESP01_SEND_ERROR               /*!< Fallo o error reportado en la transmisión de datos */
} _eESP01STATUS;

/**
 * @defgroup ESP01_Constants Parámetros de Memoria del Driver ESP-01
 * @ingroup group_comm
 * @{
 */
#define ESP01RXBUFAT		512  /*!< Capacidad en bytes del buffer circular interno de recepción AT (optimizado sin webserver) */
#define ESP01TXBUFAT		512  /*!< Capacidad en bytes del buffer circular interno de transmisión AT (optimizado sin webserver) */
/** @} */

/**
 * @brief Estructura de funciones de bajo nivel requeridas por el driver ESP01.
 * @details Permite la abstracción del hardware para que el driver controle el pin de habilitación
 *          del módulo y realice la lectura/escritura en la USART asignada.
 */
typedef struct{
	void (*DoCHPD)(uint8_t value);          /*!< Puntero a función que comanda el pin CH_PD (Chip Enable) */
	int (*WriteUSARTByte)(uint8_t value);    /*!< Puntero a función que escribe un byte en la USART (retorna 1 si tuvo éxito) */
	void (*WriteByteToBufRX)(uint8_t value);/*!< Puntero a función que almacena un byte entrante en el buffer de la aplicación */
} _sESP01Handle;

/**
 * @brief Inicializa las estructuras internas y timers del driver ESP01.
 * @param[in] hESP01 Puntero a la estructura de manejadores de hardware `_sESP01Handle`.
 * @note Esta función debe ejecutarse antes de cualquier otra interacción con el driver.
 */
void ESP01_Init(_sESP01Handle *hESP01);

/**
 * @brief Configura las credenciales de la red Wi-Fi e inicia el proceso de asociación.
 * @details Si existía una conexión previa activa, el driver la cierra antes de conectar
 *          al nuevo SSID. El estado puede verificarse con `ESP01_StateWIFI()`.
 * @param[in] ssid Cadena con el identificador de red (SSID).
 * @param[in] password Cadena con la clave WPA/WPA2 de la red.
 */
void ESP01_SetWIFI(const char *ssid, const char *password);

/**
 * @brief Inicia una conexión de transporte UDP hacia un endpoint IP y puerto remotos.
 * @param[in] RemoteIP Dirección IP de destino en formato texto (ej: `"192.168.0.10"`).
 * @param[in] RemotePORT Puerto UDP de destino en el equipo remoto.
 * @param[in] LocalPORT Puerto UDP de escucha local en el ESP-01.
 * @return Estado resultante de la solicitud de apertura (`_eESP01STATUS`).
 */
_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);

/**
 * @brief Inicia una conexión de transporte TCP cliente hacia un servidor remoto.
 * @param[in] RemoteIP Dirección IP del servidor TCP remoto.
 * @param[in] RemotePORT Puerto del servidor TCP remoto.
 * @param[in] LocalPORT Puerto local de enlace.
 * @return Estado resultante de la solicitud de conexión (`_eESP01STATUS`).
 */
_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);

/**
 * @brief Devuelve el protocolo actualmente activo en el driver ("UDP" o "TCP").
 * @return Puntero a cadena con el nombre del protocolo.
 */
const char *ESP01_GetProtocol(void);

/**
 * @brief Cierra la conexión activa de socket UDP o TCP (`AT+CIPCLOSE`).
 */
void ESP01_CloseUDPTCP(void);

/**
 * @brief Consulta el estado actual de la conexión a la infraestructura Wi-Fi.
 * @return Estado actual según `_eESP01STATUS`.
 */
_eESP01STATUS ESP01_StateWIFI(void);

/**
 * @brief Obtiene la dirección IP local asignada al ESP-01 en la red Wi-Fi.
 * @return Puntero a la cadena que contiene la IP local, o `NULL` si no posee IP asignada.
 */
char *ESP01_GetLocalIP(void);

/**
 * @brief Consulta el estado de enlace del socket UDP o TCP.
 * @return Estado del socket (`_eESP01STATUS`).
 */
_eESP01STATUS ESP01_StateUDPTCP(void);

/**
 * @brief Transmite un bloque de datos del buffer circular a través del socket activo.
 * @param[in] connID Identificador del canal/conexión (0 para modo Station único; ID de cliente en WebServer).
 * @param[in] buf Puntero al buffer de memoria que almacena los datos.
 * @param[in] irRingBuf Índice de lectura actual en el buffer circular.
 * @param[in] length Cantidad de bytes a transmitir.
 * @param[in] sizeRingBuf Tamaño total del buffer circular para gestión de wrap-around.
 * @retval ESP01_SEND_READY Transmisión encolada correctamente hacia el módulo.
 * @retval ESP01_SEND_BUSY Transmisor ocupado con un envío previo.
 * @retval ESP01_SEND_ERROR Fallo al procesar el comando AT de envío.
 */
_eESP01STATUS ESP01_Send(uint8_t connID, uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf);

/**
 * @brief Configura el módulo en modo Dual (Station + SoftAP) e inicia un servidor TCP.
 * @details El módulo crea un punto de acceso inalámbrico propio en el canal especificado
 *          y abre un socket servidor TCP en el puerto indicado (por defecto 80) para permitir
 *          la configuración de credenciales (SSID;PASS) mediante software de terminal (Hercules).
 * @param[in] apSSID Nombre (SSID) de la red inalámbrica que emitirá el robot.
 * @param[in] apPass Contraseña del SoftAP (mínimo 8 caracteres, o NULL/"" para red abierta).
 * @param[in] ch Canal de radiofrecuencia Wi-Fi (1 a 13).
 * @param[in] enc Tipo de autenticación: 0=Abierta, 2=WPA, 3=WPA2, 4=WPA/WPA2.
 * @param[in] port Puerto de escucha del servidor TCP (ej: 80 o 8080). Si es 0, usa 80.
 */
void ESP01_SetSoftAP(const char *apSSID, const char *apPass, uint8_t ch, uint8_t enc, uint16_t port);

/* Macro de compatibilidad */
#define ESP01_SetWebServer(ssid, pass, ch, enc) ESP01_SetSoftAP(ssid, pass, ch, enc, 80)

/**
 * @brief Retorna el identificador de conexión (`connID`) del último cliente TCP recibido (`+IPD`).
 * @return ID numérico de conexión (rango 0 a 4 en multiplexado `CIPMUX=1`).
 */
uint8_t ESP01_GetLastConnID(void);

/**
 * @brief Base de tiempo periódica de 10 ms para la gestión de timeouts del módulo.
 * @details Debe ser llamada periódicamente cada 10 milisegundos desde el planificador del sistema.
 */
void ESP01_Timeout10ms(void);

/**
 * @brief Tarea principal de procesamiento de la máquina de estados AT.
 * @details Debe ser llamada continuamente en el bucle principal (`main`) para analizar
 *          las respuestas recibidas por USART (`OK`, `ERROR`, `+IPD`, etc.) y gestionar envíos.
 */
void ESP01_Task(void);

/**
 * @brief Inyecta un byte recibido por interrupción USART en el buffer del driver.
 * @param[in] value Byte recibido desde el módulo ESP-01 por hardware USART.
 */
void ESP01_WriteRX(uint8_t value);

/**
 * @brief Registra una función de notificación para cambios de estado en el driver.
 * @param[in] aESP01ChangeState Puntero a función callback que recibe el nuevo `_eESP01STATUS`.
 */
void ESP01_AttachChangeState(void (*aESP01ChangeState)(_eESP01STATUS esp01State));

/**
 * @brief Registra una función callback para imprimir trazas de depuración de texto.
 * @param[in] aESP01DbgStr Puntero a función que recibe cadenas de depuración terminadas en `\0`.
 */
void ESP01_AttachDebugStr(void (*aESP01DbgStr)(const char *dbgStr));

/**
 * @brief Consulta si el módulo se encuentra actualmente ejecutando un reset físico por hardware.
 * @retval 1 Hard reset en proceso.
 * @retval 0 Funcionamiento normal.
 */
int ESP01_IsHDRRST(void);

#endif /* INC_ESP01_H_ */
