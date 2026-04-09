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
	ALIVE = 0xF0,
	FIRMWARE = 0xF1,
	GETMPU = 0xF2,
	GETADC = 0xF3,
	SETPWM = 0xF4,
	SETPID = 0xF5,
	SETPWMLIMIT = 0XF6,
	SETSETPOINT = 0xF7,
	SETPIDLINE = 0xF8,
	GETINTERNALDATA = 0xF9,
	SETOFFSET = 0xFA,
	SETCUSTOMTURN = 0xFB,
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
