# Script de generación de documentación Doxygen para MICRO-Firmware
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "  MICRO-Firmware: Generador de Documentación Técnica (Doxygen)" -ForegroundColor Cyan
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ""

$doxygenCmd = Get-Command doxygen -ErrorAction SilentlyContinue
if (-not $doxygenCmd) {
    Write-Host "[ERROR] 'doxygen' no está instalado o no se encuentra en el PATH." -ForegroundColor Red
    Write-Host "Descárguelo desde: https://www.doxygen.nl/download.html" -ForegroundColor Yellow
    Write-Host "O puede abrir Doxywizard y cargar el archivo 'MICRO.doxygen' / 'Doxyfile'." -ForegroundColor Yellow
    Read-Host "Presione Enter para salir"
    exit 1
}

$dotCmd = Get-Command dot -ErrorAction SilentlyContinue
if (-not $dotCmd) {
    Write-Host "[ADVERTENCIA] 'dot' (Graphviz) no está detectado en el PATH." -ForegroundColor Yellow
    Write-Host "Los gráficos de llamadas vectoriales (Call Graphs) requieren Graphviz." -ForegroundColor Yellow
    Write-Host "Descárguelo desde: https://graphviz.org/download/" -ForegroundColor Yellow
}

Write-Host "[INFO] Procesando documentación con Doxyfile..." -ForegroundColor Green
& doxygen Doxyfile

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "======================================================================" -ForegroundColor Cyan
    Write-Host "  Documentación generada con éxito en Doc\html\index.html" -ForegroundColor Green
    Write-Host "======================================================================" -ForegroundColor Cyan
    if (Test-Path "Doc\html\index.html") {
        Start-Process "Doc\html\index.html"
    }
} else {
    Write-Host "[ERROR] Ocurrió un error al compilar la documentación." -ForegroundColor Red
}
