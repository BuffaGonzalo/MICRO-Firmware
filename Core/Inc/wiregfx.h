/**
 * @file    wiregfx.h
 * @author  Gonzalo M. Buffa
 * @date    28/04/2026
 * @brief   Motor gráfico wireframe 3D/4D en punto fijo para pantalla OLED SSD1306.
 * @details Este módulo implementa la proyección geométrica en perspectiva y rotaciones
 *          tridimensionales y tetradimensionales (Hipercubo o Teseracto) sobre el framebuffer
 *          monocromático de 128x64 píxeles. Todas las transformaciones se computan mediante
 *          tablas de búsqueda de senos y cosenos (LUT) escaladas a 7 bits con divisiones
 *          por desplazamiento binario (`>> 7`) para prescindir de la FPU.
 * @ingroup group_ui_graphics
 */

#ifndef INC_WIREGFX_H_
#define INC_WIREGFX_H_

#include "ssd1306.h"

/**
 * @defgroup WireGFX_Config Parámetros de Centro y Perspectiva
 * @ingroup group_ui_graphics
 * @{
 */
#define CENTER_X 64 /*!< Coordenada X del centro geométrico en pantalla (128 / 2) */
#define CENTER_Y 32 /*!< Coordenada Y del centro geométrico en pantalla (64 / 2) */
/** @} */

/**
 * @brief Coordenada espacial tridimensional en punto fijo.
 */
typedef struct {
    int16_t x; /*!< Coordenada en eje X */
    int16_t y; /*!< Coordenada en eje Y */
    int16_t z; /*!< Coordenada en eje Z (profundidad) */
} Point3D;

/**
 * @brief Coordenada proyectada sobre el plano bidimensional de la pantalla.
 */
typedef struct {
    int16_t x; /*!< Coordenada horizontal proyectada en píxeles */
    int16_t y; /*!< Coordenada vertical proyectada en píxeles */
} Point2D;

/**
 * @brief Coordenada espacial tetradimensional (4D) para geometrías hiperespaciales.
 */
typedef struct {
    int16_t x; /*!< Coordenada espacial X */
    int16_t y; /*!< Coordenada espacial Y */
    int16_t z; /*!< Coordenada espacial Z */
    int16_t w; /*!< Coordenada de la cuarta dimensión espacial W */
} Point4D;

/**
 * @brief Renderiza un cubo 3D rotando sobre los ejes X, Y y Z en la pantalla OLED.
 */
void WIREGFX_Graphics_DrawCube(void);

/**
 * @brief Renderiza un teseracto (hipercubo 4D) con proyección en perspectiva estereográfica.
 */
void WIREGFX_Graphics_DrawTesseract(void);

/**
 * @brief Renderiza una pirámide de base cuadrada rotando en el espacio tridimensional.
 */
void WIREGFX_Graphics_DrawPyramid(void);

/**
 * @brief Renderiza una esfera alámbrica tridimensional.
 */
void WIREGFX_Graphics_DrawSphere(void);

/**
 * @brief Gestiona el ciclo secuencial de animación de figuras tridimensionales.
 * @details Conmuta automáticamente la figura visualizada cada 10 segundos:
 *          **Cubo 3D** &rarr; **Teseracto 4D** &rarr; **Pirámide 3D**.
 */
void WIREGFX_DisplayTask(void);

/**
 * @brief Reinicia el ciclo de animación para comenzar inmediatamente desde el Cubo 3D.
 */
void WIREGFX_ResetCycle(void);

#endif /* INC_WIREGFX_H_ */
