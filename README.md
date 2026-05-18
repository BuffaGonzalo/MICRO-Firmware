# MICRO-Firmware

Firmware para un autito de tipo péndulo invertido basado en el microcontrolador STM32F103 (con 64KB de FLASH y 20KB de RAM). El sistema está optimizado para cálculo en punto fijo (FPA) debido a la ausencia de FPU (unidad de punto flotante), y se encarga del control en tiempo real del autito, lectura de sensores y la comunicación con la PC.

## Resumen del funcionamiento

- Lee sensores (IMU y codificadores) para estimar la posición y el ángulo del autito.
- Realiza control en lazo cerrado mediante algoritmos implementados en punto fijo.
- Controla los actuadores (motor, servos, etc.) para mantener el equilibrio y la trayectoria deseada.
- Se comunica con una interfaz gráfica de PC para monitoreo y telemetría en tiempo real.

## Componentes utilizados

- **Microcontrolador:** STM32F103C8T6 (Cortex-M3, 64KB FLASH, 20KB RAM)
- **Sensores:** IMU (Inertial Measurement Unit - acelerómetro y giroscopio), codificadores ópticos para ruedas
- **Actuadores:** Motor DC, servo o sistema de dirección, controlador de potencia
- **Comunicación:** UART serial para vinculación con PC

## Interfaz gráfica complementaria

Para el monitoreo y control desde PC, este firmware se complementa con el proyecto [MICRO-Software](https://github.com/BuffaGonzalo/MICRO-Software), que provee la interfaz gráfica de usuario para visualización y ajuste remoto de parámetros.

---

Desarrollado por BuffaGonzalo.
