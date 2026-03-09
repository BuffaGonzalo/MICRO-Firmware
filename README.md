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

## 📁 Estructura del Repositorio

```
MCC-Firmware/
├── Core/
│   ├── Inc/          # Headers de la aplicación
│   │   ├── main.h
│   │   ├── esp01.h       # Driver ESP-01 (WiFi)
│   │   ├── mpu6050.h     # Driver MPU6050 (IMU)
│   │   ├── ssd1306.h     # Driver SSD1306 (OLED)
│   │   ├── unerPrtcl.h   # Protocolo UNER
│   │   └── ...
│   └── Src/          # Código fuente de la aplicación
│       ├── main.c
│       ├── esp01.c
│       ├── mpu6050.c
│       ├── ssd1306.c
│       ├── unerPrtcl.c
│       └── ...
├── Drivers/          # HAL y CMSIS de ST (generados por CubeMX)
├── Middlewares/      # Middleware USB (generado por CubeMX)
├── USB_DEVICE/       # Configuración USB CDC
└── MCC_STM32F103.ioc # Archivo de configuración STM32CubeMX
```

---

## 🚀 Compilación y Flasheo

### Requisitos

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) v1.15 o superior
- Programador ST-Link (o cualquier compatible con OpenOCD)

### Pasos

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/BuffaGonzalo/MCC-Firmware.git
   ```
2. Abrir STM32CubeIDE y seleccionar **File → Open Projects from File System**, apuntando a la carpeta clonada.
3. Compilar con **Project → Build Project** (o `Ctrl+B`).
4. Conectar el programador ST-Link y flashear con **Run → Run** (o `F11`).

---

## 📡 Protocolo de Comunicación

El MCC utiliza el **Protocolo UNER** — un protocolo binario ligero con la siguiente trama:

```
[ 'U' | 'N' | 'E' | 'R' | nBytes | Token | ID | Payload... | CHK ]
```

Los comandos principales son:

| ID | Nombre | Descripción |
|---|---|---|
| `GETADC` | Get ADC | Solicita los 8 valores analógicos actuales |
| `SETPWM` | Set PWM | Establece el duty cycle de los 4 canales PWM |
| `GETFW` | Get Firmware | Devuelve la versión del firmware |

---

## 📄 Licencia

Este proyecto se distribuye bajo los términos de la licencia incluida en cada archivo fuente. El código del HAL de STMicroelectronics se distribuye bajo su propia licencia (ver `Drivers/`).