/**
 * @file   protocol.h
 * @author Gonzalo M. buffa
 * @date   24/05/2025
 *
 * protocol.h
 * File part of the protocol library, based un the UNER protocol. It's designed to simplify the
 * the development of code for the user by reducing the amount of code lines in the main.c
 *
 * User Attention!!!
 * The library automates the configuration/decryption of headers and some data manipulation
 * The user still needs to handle the received data (RxData) and also what to do with the received data.
 */

#ifndef INC_UNERPRTCL_H_
#define INC_UNERPRTCL_H_

#include <stdint.h>

#define RXBUFSIZE           256
#define TXBUFSIZE           256
#define TXBYTES				7

/**
 *
 * @brief Unión ara la descomposición/composición de números mayores a 1 byte
 *
 */
typedef union {
	uint32_t ui32;
	int32_t i32;
	uint16_t ui16[2];
	int16_t i16[2];
	uint8_t ui8[4];
	int8_t i8[4];
} _uWord;

/**
 * @brief estructura para la recepción de datos por puerto serie
 *
 */
//typedef struct {
//	uint8_t *buff; /*!< Puntero para el buffer de recepción*/
//	uint8_t indexR; /*!< indice de lectura del buffer circular*/
//	uint8_t indexW; /*!< indice de escritura del buffer circular*/
//	uint8_t indexData; /*!< indice para identificar la posición del dato*/
//	uint8_t mask; /*!< máscara para controlar el tamaño del buffer*/
//	uint8_t chk; /*!< variable para calcular el checksum*/
//	uint8_t nBytes; /*!< variable para almacenar el número de bytes recibidos*/
//	uint8_t header; /*!< variable para mantener el estado dela MEF del protocolo*/
//	uint8_t timeOut; /*!< variable para resetear la MEF si no llegan más caracteres luego de cierto tiempo*/
//	uint8_t isComannd;
//} _sRx;

/**
 * @brief Estructura para la transmisión de datos por el puerto serie
 *
 */
//typedef struct {
//	uint8_t *buff; /*!< Puntero para el buffer de transmisión*/
//	uint8_t indexR; /*!<indice de lectura del buffer circular*/
//	uint8_t indexW; /*!<indice de escritura del buffer circular*/
//	uint8_t mask; /*!<máscara para controlar el tamaño del buffer*/
//	uint8_t bytes;
//	uint8_t indexData;
//	uint8_t chk; /*!< variable para calcular el checksum*/
//} _sTx;


/**
 * @brief Estructura correspondiente a la comunicacion
 *
 */
typedef struct {
	uint8_t *buff; /*!< Puntero para el buffer de transmisión*/
	uint8_t indexR; /*!<indice de lectura del buffer circular*/
	uint8_t indexW; /*!<indice de escritura del buffer circular*/
	uint8_t mask; /*!<máscara para controlar el tamaño del buffer*/
	uint8_t bytes;
	uint8_t indexData;
	uint8_t chk; /*!< variable para calcular el checksum*/
} _sComm;

/**
 * @brief Enumeración para la maquina de estados
 * que se encarga de decodificar el protocolo
 * de comunicación
 *
 */
typedef enum {
	HEADER_U, HEADER_N, HEADER_E, HEADER_R, NBYTES, TOKEN, ID, PAYLOAD
} _eDecode;

/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 *
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param ID Identificaci?n del comando que se env?a
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisi?n
 */
uint8_t unerPrtcl_PutHeaderOnTx(_sComm  *dataTx, uint8_t ID, uint8_t frameLength);

/**
 * @brief Agrega un byte al buffer de transmisi?n
 *
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisi?n
 */
uint8_t unerPrtcl_PutByteOnTx(_sComm    *dataTx, uint8_t byte);

/**
 * @brief Agrega un String al buffer de transmisi?n
 *
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisi?n
 */
uint8_t unerPrtcl_PutStrOntx(_sComm *dataTx, const char *str);

/**
 * @brief Decodifica la trama recibida
 *
 * @param dataRx Estructura para la recepci?n de datos
 */
uint8_t unerPrtcl_DecodeHeader(_sComm *dataRx);

uint8_t unerPrtcl_GetByteFromRx(_sComm *dataRx, uint8_t start, uint8_t end);

void unerPrtcl_Init(_sComm *Rx, _sComm *Tx, volatile uint8_t *buffRx, volatile uint8_t *buffTx);

#endif /* INC_UNERPRTCL_H_ */
