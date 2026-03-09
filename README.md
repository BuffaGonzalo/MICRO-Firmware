# MCC-Firmware

> **Firmware embebido para el Multi-Channel Controller (MCC)**
> Desarrollado sobre un microcontrolador STM32F103C8T6 con STM32CubeIDE.

---

## 📖 Descripción del Proyecto

El **MCC (Multi-Channel Controller)** es un controlador embebido diseñado para adquirir señales analógicas y de movimiento, y transmitirlas en tiempo real a una PC a través de USB o WiFi. Es la parte de hardware/firmware de un sistema más amplio que incluye también una aplicación de escritorio.

El sistema puede utilizarse como controlador de mando a distancia, plataforma de adquisición de datos o interfaz de control para robots y dispositivos motorizados.

---

## 🔗 Proyecto Hermano

Este repositorio contiene únicamente el firmware del microcontrolador. El software de PC que se comunica con este dispositivo se encuentra en el repositorio hermano:

➡️ **[MCC25_software](https://github.com/BuffaGonzalo/MCC25_software)** — Aplicación de escritorio para Windows/Linux que recibe los datos del MCC, los visualiza y permite controlar el dispositivo en tiempo real.

---

## 🛠️ Hardware

| Componente | Descripción |
|---|---|
| **MCU** | STM32F103C8T6 (Blue Pill) @ 72 MHz |
| **Display** | SSD1306 OLED 128×64 (I2C2) |
| **IMU** | MPU6050 — Acelerómetro + Giroscopio (I2C2) |
| **WiFi** | ESP-01 (ESP8266) — UART1 |
| **Entradas analógicas** | 8 canales ADC (PA0–PA7) — joysticks, potenciómetros |
| **Salidas PWM** | 4 canales TIM3 (PB4, PB5, PB0, PB1) — servos / motores |
| **Comunicación PC** | USB CDC (puerto COM virtual) |
| **Botón** | SW0 (PB15) |
| **LED** | LED_BUILTIN (PC13) |

---

## ✨ Funcionalidades

- **Adquisición de datos**: lectura de 8 canales ADC con DMA y datos del IMU (MPU6050) vía I2C con DMA.
- **Control PWM**: 4 salidas PWM independientes para control de motores o servos.
- **Comunicación USB**: puerto COM virtual (USB CDC) para comunicación directa con la PC.
- **Comunicación WiFi**: enlace UDP/TCP con la PC a través del módulo ESP-01.
- **Modo WebServer**: el ESP-01 levanta un punto de acceso WiFi propio y un servidor HTTP para que el usuario configure las credenciales de red desde cualquier navegador.
- **Protocolo UNER**: protocolo de comunicación binario con header, ID de comando y checksum, utilizado tanto en USB como en WiFi.
- **Display OLED**: visualización del estado de los canales ADC y parámetros del sistema en tiempo real.
- **Heartbeat configurable**: LED indicador de estado con patrones programables.

---

## 📄 Licencia

Este proyecto se distribuye bajo los términos de la licencia incluida en cada archivo fuente. El código del HAL de STMicroelectronics se distribuye bajo su propia licencia (ver `Drivers/`).
