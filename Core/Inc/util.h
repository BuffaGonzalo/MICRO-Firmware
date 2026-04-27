/**
 * @file   util.h
 * @author Gonzalo M. Buffa
 * @date   24/05/2025
 *
 * file used to declare all enums,structs,unions of the main project
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdint.h>

/**
 * @brief Enumeracion nameless
 *
 */
enum {
	FALSE, TRUE
};

enum {
	SERIE, WIFI
};


/**
 * @brief Mapa de bits para banderas
 *
 */
typedef union{
    struct{
        uint8_t bit7 : 1;
        uint8_t bit6 : 1;
        uint8_t bit5 : 1;
        uint8_t bit4 : 1;
        uint8_t bit3 : 1;
        uint8_t bit2 : 1;
        uint8_t bit1 : 1;
        uint8_t bit0 : 1;
    }bits;
    uint8_t bytes;
}_uFlag;

/**
 * @brief Enumeración de los comandos del protocolo
 */
typedef enum {
	ALIVE = 0xA0,
	FIRMWARE = 0xA1,
	GETMPU = 0xA2,
	GETADC = 0xA3,

	SETPWML = 0xA4,
	SETPWMR = 0xA5,

	SETPWMMINR = 0xA6,
	SETPWMMINL = 0xA7,

	SETBALANCEKP = 0xA8,
	SETBALANCEKD = 0xA9,
	SETBALANCEKI = 0xAA,

	SETSETPOINT = 0xAB,

	SETLINEKP = 0xAC,
	SETLINEKD = 0xAD,

	GETINTERNALDATA = 0xF0,
	GETPIDBALANCE = 0xF1,

	SETOFFSETL = 0xAE,
	SETOFFSETR = 0xAF,

	SETCUSTOMTURN = 0xB0,
	SETSPEED = 0xB1,
	SETBKANG = 0xB2,

	ACK = 0x0D,
	UNKNOWN = 0xFF
} _eCmd;

typedef enum{
	IDLE = 0,
	DATA_DISPLAY = 1,
	UPD_DISPLAY = 2,
	ONMPU = 3
}_eDMA;

typedef enum{
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_RISING,
    BUTTON_FALLING
}_eButtonState;

/**
 * @brief Enumeracion de los estados de los diferentes estados de los botones, como tengo una configuracion PullDown los coloqué de tal forma que me quede el valor de NOT_PRESSED = 0  y PRESSED = 1
*/
typedef enum{
    PRESSED,
    NOT_PRESSED,
    NO_EVENT
}_eEvent;

//ESTRUCTURAS
typedef struct
{
    _eButtonState   currentState;
    _eEvent         stateInput;
    uint8_t        isPressed;
    uint16_t        time;
}_sButton;

#endif /* INC_UTIL_H_ */
