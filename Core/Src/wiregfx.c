/*
 * wiregfx.c
 *
 *  Created on: 28 abr 2026
 *      Author: gonza
 */

#include "wiregfx.h"

// --- Ángulos de rotación internos ---
static uint8_t angle_x = 0;
static uint8_t angle_y = 0;
static uint8_t angle_z = 0;
static uint8_t angle_xz = 0;
static uint8_t angle_yz = 0;
static uint8_t angle_xw = 0;

// Look-Up Table (LUT) de Seno. 256 valores precalculados.
// Está escalada de -127 a 127. (1.0 = 127). Esto nos permite dividir rápido con ">> 7".
const int8_t sin_LUT[256] = {
    0, 3, 6, 9, 12, 15, 18, 21, 24, 28, 31, 34, 37, 40, 43, 46,
    48, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88,
    90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 109, 111, 112, 114, 115, 117,
    118, 119, 120, 121, 122, 123, 124, 124, 125, 126, 126, 126, 127, 127, 127, 127,
    127, 127, 127, 127, 126, 126, 126, 125, 124, 124, 123, 122, 121, 120, 119, 118,
    117, 115, 114, 112, 111, 109, 108, 106, 104, 102, 100, 98, 96, 94, 92, 90,
    88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 48,
    46, 43, 40, 37, 34, 31, 28, 24, 21, 18, 15, 12, 9, 6, 3, 0,
    -3, -6, -9, -12, -15, -18, -21, -24, -28, -31, -34, -37, -40, -43, -46, -48,
    -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83, -85, -88, -90,
    -92, -94, -96, -98, -100, -102, -104, -106, -108, -109, -111, -112, -114, -115, -117, -118,
    -119, -120, -121, -122, -123, -124, -124, -125, -126, -126, -126, -127, -127, -127, -127, -127,
    -127, -127, -127, -126, -126, -126, -125, -124, -124, -123, -122, -121, -120, -119, -118, -117,
    -115, -114, -112, -111, -109, -108, -106, -104, -102, -100, -98, -96, -94, -92, -90, -88,
    -85, -83, -81, -78, -76, -73, -71, -68, -65, -63, -60, -57, -54, -51, -48, -46,
    -43, -40, -37, -34, -31, -28, -24, -21, -18, -15, -12, -9, -6, -3, 0
};

static const Point3D cube_vertices[8] = {
    {-16, -16, -16}, { 16, -16, -16}, { 16,  16, -16}, {-16,  16, -16},
    {-16, -16,  16}, { 16, -16,  16}, { 16,  16,  16}, {-16,  16,  16}
};

static const int cube_edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}
};

static const Point4D tesseract_vertices[16] = {
    {-16, -16, -16, -16}, { 16, -16, -16, -16}, { 16,  16, -16, -16}, {-16,  16, -16, -16},
    {-16, -16,  16, -16}, { 16, -16,  16, -16}, { 16,  16,  16, -16}, {-16,  16,  16, -16},
    {-16, -16, -16,  16}, { 16, -16, -16,  16}, { 16,  16, -16,  16}, {-16,  16, -16,  16},
    {-16, -16,  16,  16}, { 16, -16,  16,  16}, { 16,  16,  16,  16}, {-16,  16,  16,  16}
};

static const int tesseract_edges[32][2] = {
    {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7},
    {8,9}, {9,10}, {10,11}, {11,8}, {12,13}, {13,14}, {14,15}, {15,12}, {8,12}, {9,13}, {10,14}, {11,15},
    {0,8}, {1,9}, {2,10}, {3,11}, {4,12}, {5,13}, {6,14}, {7,15}
};

// --- Implementación de Funciones ---

void WIREGFX_Graphics_DrawCube(void) {
    Point2D projected[8];
    int32_t sin_x = sin_LUT[angle_x];
    int32_t cos_x = sin_LUT[(uint8_t)(angle_x + 64)];
    int32_t sin_y = sin_LUT[angle_y];
    int32_t cos_y = sin_LUT[(uint8_t)(angle_y + 64)];
    int32_t sin_z = sin_LUT[angle_z];
    int32_t cos_z = sin_LUT[(uint8_t)(angle_z + 64)];

    ssd1306_Fill(Black);

    for (int i = 0; i < 8; i++) {
        int32_t x = cube_vertices[i].x;
        int32_t y = cube_vertices[i].y;
        int32_t z = cube_vertices[i].z;

        // Rotaciones (X, Y, Z) aplicando corrimiento >> 7 para punto fijo
        int32_t xy = (y * cos_x - z * sin_x) >> 7;
        int32_t xz = (y * sin_x + z * cos_x) >> 7;
        y = xy; z = xz;

        int32_t yx = (x * cos_y + z * sin_y) >> 7;
        int32_t yz = (-x * sin_y + z * cos_y) >> 7;
        x = yx; z = yz;

        int32_t zx = (x * cos_z - y * sin_z) >> 7;
        int32_t zy = (x * sin_z + y * cos_z) >> 7;
        x = zx; y = zy;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    for (int i = 0; i < 12; i++) {
        ssd1306_Line(projected[cube_edges[i][0]].x, projected[cube_edges[i][0]].y,
                     projected[cube_edges[i][1]].x, projected[cube_edges[i][1]].y, White);
    }

    angle_x += 2; angle_y += 1; angle_z += 3;
}

void WIREGFX_Graphics_DrawTesseract(void) {
    Point2D projected[16];
    int32_t sin_xz = sin_LUT[angle_xz];
    int32_t cos_xz = sin_LUT[(uint8_t)(angle_xz + 64)];
    int32_t sin_yz = sin_LUT[angle_yz];
    int32_t cos_yz = sin_LUT[(uint8_t)(angle_yz + 64)];
    int32_t sin_xw = sin_LUT[angle_xw];
    int32_t cos_xw = sin_LUT[(uint8_t)(angle_xw + 64)];

    ssd1306_Fill(Black);

    for (int i = 0; i < 16; i++) {
        int32_t x = tesseract_vertices[i].x;
        int32_t y = tesseract_vertices[i].y;
        int32_t z = tesseract_vertices[i].z;
        int32_t w = tesseract_vertices[i].w;

        int32_t xw_x = (x * cos_xw - w * sin_xw) >> 7;
        int32_t xw_w = (x * sin_xw + w * cos_xw) >> 7;
        x = xw_x; w = xw_w;

        int32_t xz_x = (x * cos_xz - z * sin_xz) >> 7;
        int32_t xz_z = (x * sin_xz + z * cos_xz) >> 7;
        x = xz_x; z = xz_z;

        int32_t yz_y = (y * cos_yz - z * sin_yz) >> 7;
        int32_t yz_z = (y * sin_yz + z * cos_yz) >> 7;
        y = yz_y; z = yz_z;

        int32_t w_factor = 48 - w;
        x = (x * w_factor) >> 6;
        y = (y * w_factor) >> 6;

        projected[i].x = x + CENTER_X;
        projected[i].y = y + CENTER_Y;
    }

    for (int i = 0; i < 32; i++) {
        ssd1306_Line(projected[tesseract_edges[i][0]].x, projected[tesseract_edges[i][0]].y,
                     projected[tesseract_edges[i][1]].x, projected[tesseract_edges[i][1]].y, White);
    }

    angle_xz += 1; angle_yz += 1; angle_xw += 2;
}
