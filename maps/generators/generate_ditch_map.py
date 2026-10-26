#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
DITCH ROAD GENERATOR (CARLA / MathWorks RoadRunner / ASAM OpenDRIVE 1.4)
================================================================================
Role: Senior Technical Artist & CAD Engineer
Standard Compliance:
  - СП 34.13330.2021 ("Автомобильные дороги")
  - ГОСТ Р 52289-2019 ("Технические средства организации дорожного движения")
  - ISO 26262 Rollover HARA (Hazard Analysis and Risk Assessment)
  - ASAM OpenDRIVE 1.4 Specification

Outputs:
  - map_ditch.fbx: 3D geometry with submeshes, materials, bevel transitions, UVs
  - map_ditch.xodr: Synchronized logical road network map
================================================================================
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path

# OpenDRIVE 1.4 XML Content
XODR_CONTENT = '''<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="DitchRoad" version="1.00" date="2026-10-22T00:00:00" north="0.0" south="0.0" east="0.0" west="0.0">
        <geoReference><![CDATA[+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1.0 +x_0=0.0 +y_0=0.0 +datum=WGS84 +units=m +no_defs]]></geoReference>
    </header>
    
    <!-- Прямолинейный участок загородной магистрали L = 1500 м вдоль оси X -->
    <road id="1" name="DitchRoad" length="1500.0" junction="-1">
        <link/>
        
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="1500.0">
                <line/>
            </geometry>
        </planView>

        <!-- Ровный высотный профиль проезжей части Z = 0.0 м -->
        <elevationProfile>
            <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
        </elevationProfile>

        <!-- 
             Поперечный высотный профиль кювета (lateralProfile/shape):
             Точно повторяет поперечное сечение 3D-меша:
             - Поле/земля: t in [-30.85, -20.85], dz = 0.0
             - Обратный откос кювета (1:4): t in [-20.85, -16.05], спуск от 0.0 до -1.20 м
             - Дно водоотводного кювета: t in [-16.05, -14.55], dz = -1.20 м
             - Внутренний откос насыпи (1:4): t in [-14.55, -9.75], подъем от -1.20 м до 0.0 м
             - Обочина и проезжая часть: t in [-9.75, 0.0], dz = 0.0 м
             - Левая граница/разделитель: t in [0.0, 1.0], dz = 0.0 м
        -->
        <lateralProfile>
            <shape s="0.0" t="-30.85" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <shape s="0.0" t="-20.85" a="0.0" b="-0.25" c="0.0" d="0.0"/>
            <shape s="0.0" t="-16.05" a="-1.20" b="0.0" c="0.0" d="0.0"/>
            <shape s="0.0" t="-14.55" a="-1.20" b="0.25" c="0.0" d="0.0"/>
            <shape s="0.0" t="-9.75" a="0.0" b="0.0" c="0.0" d="0.0"/>
            <shape s="0.0" t="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
        </lateralProfile>

        <lanes>
            <laneSection s="0.0">
                
                <!-- Левая сторона: техническая полоса и жесткий барьер 0.80 м -->
                <left>
                    <lane id="1" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="1.0" b="0.0" c="0.0" d="0.0"/>
                        <!-- Физическая высота жесткого удерживающего ограждения 0.80 м -->
                        <height sOffset="0.0" inner="0.80" outer="0.80"/>
                    </lane>
                </left>

                <!-- Осевая линия: сплошная белая линия разметки 0.15 м -->
                <center>
                    <lane id="0" type="none" level="false">
                        <roadMark sOffset="0.0" type="solid" weight="standard" color="white" width="0.15"/>
                    </lane>
                </center>

                <!-- Правая сторона (попутное движение, обочина, откосы, дно кювета, поле) -->
                <right>
                    <!-- Первая (левая попутная) полоса: 3.75 м, ограничение 90 км/ч -->
                    <lane id="-1" type="driving" level="false">
                        <link/>
                        <width sOffset="0.0" a="3.75" b="0.0" c="0.0" d="0.0"/>
                        <speed sOffset="0.0" max="90" unit="km/h"/>
                        <!-- Прерывистая разметка между полосами 1 и 2 -->
                        <roadMark sOffset="0.0" type="broken" weight="standard" color="white" width="0.15"/>
                    </lane>

                    <!-- Вторая (правая попутная) полоса: 3.75 м, ограничение 90 км/ч -->
                    <lane id="-2" type="driving" level="false">
                        <link/>
                        <width sOffset="0.0" a="3.75" b="0.0" c="0.0" d="0.0"/>
                        <speed sOffset="0.0" max="90" unit="km/h"/>
                        <!-- Сплошная краевая линия разметки на внешней кромке проезжей части -->
                        <roadMark sOffset="0.0" type="solid" weight="standard" color="white" width="0.15"/>
                    </lane>

                    <!-- Укрепленная обочина (без металлических отбойников и блоков): 2.25 м -->
                    <lane id="-3" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="2.25" b="0.0" c="0.0" d="0.0"/>
                    </lane>

                    <!-- Внутренний откос насыпи кювета (уклон 1:4, глубина 1.20 м): 4.80 м -->
                    <lane id="-4" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="4.80" b="0.0" c="0.0" d="0.0"/>
                    </lane>

                    <!-- Плоское дно трапециевидного кювета на отметке Z = -1.20 м: 1.50 м -->
                    <lane id="-5" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="1.50" b="0.0" c="0.0" d="0.0"/>
                    </lane>

                    <!-- Внешний (обратный) откос кювета (уклон 1:4, подъем до Z = 0.0 м): 4.80 м -->
                    <lane id="-6" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="4.80" b="0.0" c="0.0" d="0.0"/>
                    </lane>

                    <!-- Придорожная полоса (равнинное поле Z = 0.0 м без препятствий): 10.0 м -->
                    <lane id="-7" type="shoulder" level="false">
                        <link/>
                        <width sOffset="0.0" a="10.0" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                </right>

            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>
'''

def find_blender():
    """Finds Blender executable on the system."""
    # 1. Check PATH
    blender_path = shutil.which("blender")
    if blender_path:
        return blender_path
        
    # 2. Check standard Windows installations
    candidate_paths = [
        r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 5.1\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 4.3\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 4.2\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 4.1\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 4.0\blender.exe",
        r"C:\Program Files\Blender Foundation\Blender 3.6\blender.exe",
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\Blender Foundation\blender.exe"),
    ]
    for p in candidate_paths:
        if os.path.isfile(p):
            return p
            
    # 3. Check registry if on Windows
    if sys.platform == "win32":
        try:
            import winreg
            for hkey in (winreg.HKEY_LOCAL_MACHINE, winreg.HKEY_CURRENT_USER):
                try:
                    with winreg.OpenKey(hkey, r"SOFTWARE\BlenderFoundation") as key:
                        val, _ = winreg.QueryValueEx(key, "InstallDir")
                        exe = os.path.join(val, "blender.exe")
                        if os.path.isfile(exe):
                            return exe
                except OSError:
                    pass
        except ImportError:
            pass

    return None

def generate_xodr(output_path):
    """Writes the OpenDRIVE XML file."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(XODR_CONTENT.strip())
    print(f"[XODR] Created logical map: {output_path} ({len(XODR_CONTENT)} bytes)")

def generate_fbx(output_path, blender_exe):
    """Executes Blender background script to generate the 3D FBX model."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    blender_script = os.path.join(script_dir, "build_ditch_fbx.py")
    
    cmd = [
        blender_exe,
        "--background",
        "--python", blender_script,
        "--",
        os.path.abspath(output_path)
    ]
    print(f"[FBX] Running Blender: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("[FBX ERROR] Blender stderr:\n", result.stderr)
        print("[FBX ERROR] Blender stdout:\n", result.stdout)
        raise RuntimeError(f"Blender failed with exit code {result.returncode}")
    print(f"[FBX] Successfully generated 3D model: {output_path}")

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Target file paths in current directory
    xodr_file = os.path.join(base_dir, "map_ditch.xodr")
    fbx_file = os.path.join(base_dir, "map_ditch.fbx")
    
    print("=" * 70)
    print("DITCH ROAD MAP GENERATOR: SP 34.13330.2021 / ASAM OpenDRIVE 1.4")
    print("=" * 70)
    
    # 1. Generate XODR
    generate_xodr(xodr_file)
    
    # 2. Find Blender
    blender_exe = find_blender()
    if not blender_exe:
        print("[ERROR] Blender executable not found! Please ensure Blender is installed.")
        sys.exit(1)
    print(f"[INFO] Using Blender executable: {blender_exe}")
    
    # 3. Generate FBX
    generate_fbx(fbx_file, blender_exe)
    
    # 4. In case caller expects files in a 'maps/' subfolder
    maps_subfolder = os.path.join(base_dir, "maps")
    if os.path.isdir(maps_subfolder) or not os.path.exists(maps_subfolder):
        try:
            os.makedirs(maps_subfolder, exist_ok=True)
            shutil.copy2(xodr_file, os.path.join(maps_subfolder, "map_ditch.xodr"))
            shutil.copy2(fbx_file, os.path.join(maps_subfolder, "map_ditch.fbx"))
            print(f"[INFO] Copied synchronized pair to subfolder: {maps_subfolder}")
        except Exception as e:
            pass

    print("=" * 70)
    print("GENERATION COMPLETED SUCCESSFULLY!")
    print(f"  - Logical Map: {xodr_file}")
    print(f"  - 3D Geometry: {fbx_file}")
    print("=" * 70)

if __name__ == "__main__":
    main()
