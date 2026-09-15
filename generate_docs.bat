@echo off
chcp 65001 > nul
echo ======================================================================
echo   MICRO-Firmware: Generador de Documentacion Tecnica (Doxygen)
echo ======================================================================
echo.

where doxygen >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] No se encontro 'doxygen' en la variable de entorno PATH.
    echo Por favor instale Doxygen desde: https://www.doxygen.nl/download.html
    echo Y asegurese de agregar la ruta al PATH del sistema o abrirlo con Doxywizard.
    echo.
    pause
    exit /b 1
)

where dot >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ADVERTENCIA] No se encontro 'dot' (Graphviz) en el PATH.
    echo Los diagramas de llamada (Call Graphs) y jerarquias podrian no generarse.
    echo Se recomienda instalar Graphviz: https://graphviz.org/download/
    echo.
)

where java >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    if exist "C:\ST\STM32CubeIDE_2.1.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.jre.win64_3.4.200.202601091518\jre\bin\java.exe" (
        set "PATH=C:\ST\STM32CubeIDE_2.1.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.jre.win64_3.4.200.202601091518\jre\bin;%PATH%"
    ) else (
        echo [ADVERTENCIA] No se encontro 'java' para PlantUML en el PATH.
    )
)

echo [INFO] Ejecutando Doxygen con configuracion 'Doxyfile'...
echo.
doxygen Doxyfile

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ======================================================================
    echo   Documentacion generada exitosamente en: Doc\html\index.html
    echo ======================================================================
    echo.
    if exist Doc\html\index.html (
        echo [INFO] Abriendo documentacion en el navegador web predeterminado...
        start Doc\html\index.html
    )
) else (
    echo.
    echo [ERROR] Hubo un error durante la generacion de Doxygen.
)

echo.
pause
