/*
 * wiregfx.h
 *
 *  Created on: 28 abr 2026
 *      Author: gonza
 */

#ifndef INC_WIREGFX_H_
#define INC_WIREGFX_H_

#include "ssd1306.h"

// --- Definiciones de centro de pantalla ---
#define CENTER_X 64
#define CENTER_Y 32

// --- Estructuras ---
typedef struct {
    int16_t x, y, z;
} Point3D;

typedef struct {
    int16_t x, y;
} Point2D;

typedef struct {
    int16_t x, y, z, w;
} Point4D;

// --- Prototipos de Funciones ---
/**
 * @brief Dibuja un cubo 3D rotando en la pantalla OLED.
 */
void WIREGFX_Graphics_DrawCube(void);

/**
 * @brief Dibuja un tesseract (hipercubo 4D) con efecto de perspectiva.
 */
void WIREGFX_Graphics_DrawTesseract(void);


#endif /* INC_WIREGFX_H_ */
