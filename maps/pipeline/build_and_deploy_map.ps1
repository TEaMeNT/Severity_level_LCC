param(
    [string]$MapName = "map_ditch",
    [string]$FbxPath = "c:\Users\teament\Desktop\xlam\work\maps\assets\map_ditch.fbx",
    [string]$XodrPath = "c:\Users\teament\Desktop\xlam\work\maps\assets\map_ditch.xodr",
    [string]$UprojectPath = "C:\Users\teament\Documents\Unreal Projects\map_ditch\map_ditch.uproject",
    [string]$UE4EditorCmd = "C:\Program Files\Epic Games\UE_4.26\Engine\Binaries\Win64\UE4Editor-Cmd.exe",
    [string]$CarlaContent = "C:\CARLA\CarlaUE4\Content"
)

$ErrorActionPreference = "Stop"

Write-Host "========================================================================" -ForegroundColor Cyan
Write-Host "CARLA AUTOMATED MAP BUILD & DEPLOY PIPELINE: $MapName" -ForegroundColor Cyan
Write-Host "========================================================================" -ForegroundColor Cyan

# 1. Запуск headless Python-скрипта в UE4
Write-Host "`n[1/4] Настройка мешей, PBR-материалов и уровня в UE4..." -ForegroundColor Yellow
$scriptPath = "c:\Users\teament\Desktop\xlam\work\maps\pipeline\import_and_cook_map.py"
& $UE4EditorCmd $UprojectPath -run=pythonscript -script="$scriptPath"
if ($LASTEXITCODE -ne 0) {
    Write-Warning "Python script завершился с кодом $LASTEXITCODE. Продолжаем кукинг..."
}

# 2. Cook проекта WindowsNoEditor
Write-Host "`n[2/4] Запуск кукинга (Cook WindowsNoEditor)..." -ForegroundColor Yellow
& $UE4EditorCmd $UprojectPath -run=cook -targetplatform=WindowsNoEditor -cookonthefly=0 -unattended
if ($LASTEXITCODE -ne 0) {
    Write-Error "Ошибка при кукинге проекта UE4!"
}

# 3. Деплой ассетов в CARLA
Write-Host "`n[3/4] Развертывание скомпилированных файлов в CARLA..." -ForegroundColor Yellow
$cookedRoot = Join-Path (Split-Path $UprojectPath) "Saved\Cooked\WindowsNoEditor\map_ditch\Content"

# Остановка CARLA если запущена для предотвращения блокировки файлов
Stop-Process -Name "CarlaUE4-Win64-Shipping", "CarlaUE4" -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 1

# Копирование материалов и мешей
Copy-Item "$cookedRoot\MAT_*" -Destination $CarlaContent -Force
Copy-Item "$cookedRoot\map_ditch_*" -Destination $CarlaContent -Force

# Копирование текстур
$dstTex = "$CarlaContent\StarterContent\Textures"
New-Item -ItemType Directory -Path $dstTex -Force | Out-Null
Copy-Item "$cookedRoot\StarterContent\Textures\*" -Destination $dstTex -Force -Recurse

# Копирование уровня
$dstMap = "$CarlaContent\Carla\Maps\$MapName"
New-Item -ItemType Directory -Path $dstMap -Force | Out-Null
Copy-Item "$cookedRoot\Carla\Maps\$MapName\$MapName.*" -Destination $dstMap -Force

# Копирование OpenDRIVE
if (Test-Path $XodrPath) {
    $odrDirs = @(
        "$dstMap",
        "$dstMap\OpenDrive",
        "$CarlaContent\Carla\Maps\OpenDrive"
    )
    foreach ($d in $odrDirs) {
        New-Item -ItemType Directory -Path $d -Force | Out-Null
        Copy-Item $XodrPath -Destination "$d\$MapName.xodr" -Force
    }
}

Write-Host "`n[4/4] Сборка и деплой успешно завершены!" -ForegroundColor Green
Write-Host "Карта '$MapName' готова к вызову через client.load_world('$MapName')!" -ForegroundColor Green
