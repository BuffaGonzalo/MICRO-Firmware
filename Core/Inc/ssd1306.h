/**
 * @file   ssd1306.h
 * @author Olivier Van den Eede / Aleksander Alekseev / Gonzalo M. Buffa
 * @brief  Driver de control y primitivas gráficas para display OLED monocromático SSD1306 (128x64).
 * @details Este módulo gestiona el framebuffer de video en memoria RAM (1024 bytes), los comandos
 *          de inicialización y contraste, primitivas 2D (líneas, rectángulos, círculos, bitmaps)
 *          y la transferencia asíncrona no bloqueante de la imagen a través de DMA por bus I2C2.
 * @ingroup group_ui_graphics
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "ssd1306_conf.h"

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

#ifdef SSD1306_X_OFFSET
#define SSD1306_X_OFFSET_LOWER (SSD1306_X_OFFSET & 0x0F)
#define SSD1306_X_OFFSET_UPPER ((SSD1306_X_OFFSET >> 4) & 0x07)
#else
#define SSD1306_X_OFFSET_LOWER 0
#define SSD1306_X_OFFSET_UPPER 0
#endif

/* Configuración I2C */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        (0x3C << 1)
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64  /*!< Alto de la pantalla OLED en píxeles */
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128 /*!< Ancho de la pantalla OLED en píxeles */
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8) /*!< Tamaño en bytes del framebuffer (1024 bytes) */
#endif

/**
 * @brief Paleta de color binaria para pantalla OLED monocromática.
 */
typedef enum {
    Black = 0x00, /*!< Píxel apagado (Fondo negro) */
    White = 0x01  /*!< Píxel encendido (Color emisor blanco/azul según el display) */
} SSD1306_COLOR;

/**
 * @brief Códigos de error retornados por las operaciones gráficas del SSD1306.
 */
typedef enum {
    SSD1306_OK = 0x00, /*!< Operación completada exitosamente */
    SSD1306_ERR = 0x01 /*!< Error de parámetros, desbordamiento o timeout en bus */
} SSD1306_Error_t;

/**
 * @brief Estructura de estado interno del controlador de display.
 */
typedef struct {
    uint16_t CurrentX;   /*!< Coordenada horizontal actual del cursor de texto */
    uint16_t CurrentY;   /*!< Coordenada vertical actual del cursor de texto */
    uint8_t Initialized; /*!< Bandera booleana de inicialización completada */
    uint8_t DisplayOn;   /*!< Estado de encendido de la matriz OLED (1: ON, 0: OFF) */
} SSD1306_t;

/**
 * @brief Vértice de coordenadas bidimensionales para dibujo de polilíneas.
 */
typedef struct {
    uint8_t x; /*!< Coordenada X */
    uint8_t y; /*!< Coordenada Y */
} SSD1306_VERTEX;

/**
 * @brief Estructura descriptora de fuentes tipográficas matriciales.
 */
typedef struct {
	const uint8_t width;                /*!< Ancho nominal del carácter en píxeles */
	const uint8_t height;               /*!< Alto nominal del carácter en píxeles */
	const uint16_t *const data;         /*!< Puntero a la tabla binaria de glifos en memoria FLASH */
    const uint8_t *const char_width;    /*!< Ancho variable para fuentes proporcionales (NULL para monoespaciadas) */
} SSD1306_Font_t;

/**
 * @name Funciones de Enlace y Callbacks del Driver
 * @{
 */
void ssd1306_Attach_MemWrite(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));
void ssd1306_Attach_MemWriteDMA(void(*PtrRx)(uint8_t address, uint8_t *data, uint8_t size, uint8_t type));
void ssd1306_ADC_ConfCpltCallback(volatile uint8_t *PtrRx);
/** @} */

/**
 * @name Primitivas de Inicialización y Control
 * @{
 */
void ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
char ssd1306_UpdateScreenDMA(void);
void ssd1306_ResetDMAState(void);
void ssd1306_SetContrast(const uint8_t value);
void ssd1306_SetDisplayOn(const uint8_t on);
uint8_t ssd1306_GetDisplayOn(void);
/** @} */

/**
 * @name Primitivas Gráficas y Renderizado 2D
 * @{
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, SSD1306_Font_t Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void ssd1306_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
SSD1306_Error_t ssd1306_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);
/** @} */

/**
 * @name Primitivas de Comunicación de Bajo Nivel
 * @{
 */
void ssd1306_Reset(void);
void ssd1306_WriteCommand(uint8_t byte);
void ssd1306_WriteCommandDMA(uint8_t byte);
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
void ssd1306_WriteDataDMA(uint8_t* buffer, size_t buff_size);
SSD1306_Error_t ssd1306_FillBuffer(uint8_t* buf, uint32_t len);
/** @} */

_END_STD_C

#endif // __SSD1306_H__
