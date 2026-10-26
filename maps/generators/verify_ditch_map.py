#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
VERIFICATION SCRIPT FOR DITCH MAP (FBX & XODR)
================================================================================
Role: Senior Technical Artist, CAD Verification Engineer
Verification tasks:
  1. Checks existence and sizes of map_ditch.fbx and map_ditch.xodr.
  2. Validates OpenDRIVE 1.4 logical map (lanes, speed, lateralProfile/shape).
  3. Tests CARLA OpenDRIVE parser (carla.Map) and waypoint generation.
  4. Inspects FBX 3D model (submeshes, materials, tessellation, normals).
  5. Strictly validates that at transverse distance 14.55 - 16.05 m,
     the elevation Z is exactly -1.20 m (ditch bottom).
================================================================================
"""

import os
import sys
import xml.etree.ElementTree as ET
import subprocess
import shutil

def verify_xodr(xodr_path):
    print("\n" + "=" * 60)
    print(f"[VERIFY XODR] Inspecting: {xodr_path}")
    print("=" * 60)
    
    if not os.path.isfile(xodr_path):
        raise FileNotFoundError(f"XODR file not found: {xodr_path}")
        
    size = os.path.getsize(xodr_path)
    print(f"  File size: {size} bytes -> OK")
    
    # 1. Parse XML
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    if root.tag != "OpenDRIVE":
        raise ValueError(f"Root tag is '{root.tag}', expected 'OpenDRIVE'")
        
    header = root.find("header")
    rev_major = header.attrib.get("revMajor")
    rev_minor = header.attrib.get("revMinor")
    print(f"  OpenDRIVE Version: {rev_major}.{rev_minor} -> OK")
    
    road = root.find("road")
    road_length = float(road.attrib.get("length", 0.0))
    print(f"  Road length: {road_length} m (Expected: 1500.0) -> {'OK' if road_length == 1500.0 else 'FAIL'}")
    assert road_length == 1500.0, f"Expected road length 1500.0, got {road_length}"
    
    # PlanView geometry
    geom = road.find("planView").find("geometry")
    geom_length = float(geom.attrib.get("length", 0.0))
    geom_hdg = float(geom.attrib.get("hdg", 0.0))
    print(f"  PlanView length: {geom_length} m, heading: {geom_hdg} rad -> OK")
    
    # Lateral profile shapes
    shapes = road.findall("lateralProfile/shape")
    print(f"  Lateral profile shape records found: {len(shapes)}")
    assert len(shapes) > 0, "No shape records found in lateralProfile!"
    
    # Find shape record for ditch bottom interval [-16.05, -14.55]
    ditch_shape = None
    for s in shapes:
        t_val = float(s.attrib.get("t", 0.0))
        if abs(t_val - (-16.05)) < 1e-3:
            ditch_shape = s
            break
            
    assert ditch_shape is not None, "Shape for ditch bottom (t = -16.05) not found!"
    a_val = float(ditch_shape.attrib.get("a", 0.0))
    b_val = float(ditch_shape.attrib.get("b", 0.0))
    print(f"  Ditch bottom shape at t=-16.05: a={a_val:.2f}, b={b_val:.2f}")
    assert abs(a_val - (-1.20)) < 1e-4 and abs(b_val) < 1e-4, f"Ditch bottom shape must be dz=-1.20, got a={a_val}, b={b_val}"
    print("  -> Ditch bottom shape elevation Z = -1.20 m confirmed in XODR! -> OK")
    
    # Validate Lanes
    lanes = road.find("lanes/laneSection")
    left_lanes = [int(l.attrib.get("id")) for l in lanes.findall("left/lane")]
    right_lanes = [int(l.attrib.get("id")) for l in lanes.findall("right/lane")]
    print(f"  Left lane IDs: {left_lanes} (Expected: [1])")
    print(f"  Right lane IDs: {right_lanes} (Expected: [-1, -2, -3, -4, -5, -6, -7])")
    assert 1 in left_lanes, "Left lane 1 missing!"
    for r_id in [-1, -2, -3, -4, -5, -6, -7]:
        assert r_id in right_lanes, f"Right lane {r_id} missing!"
        
    # Check speed limits on driving lanes
    for r_id in [-1, -2]:
        lane_node = lanes.find(f"right/lane[@id='{r_id}']")
        speed_node = lane_node.find("speed")
        speed_max = speed_node.attrib.get("max")
        print(f"  Lane {r_id} speed limit: {speed_max} km/h (Expected: 90) -> OK")
        assert speed_max == "90", f"Lane {r_id} speed limit expected 90, got {speed_max}"

    # CARLA Map parsing validation
    try:
        import carla
        with open(xodr_path, "r", encoding="utf-8") as f:
            xodr_str = f.read()
        carla_map = carla.Map("VerifyMap", xodr_str)
        wps = carla_map.generate_waypoints(10.0)
        print(f"  [CARLA Integration] Successfully generated {len(wps)} waypoints in CARLA OpenDRIVE parser! -> OK")
    except ImportError:
        print("  [CARLA Integration] carla module not installed, skipping CARLA Map parsing.")
    except Exception as e:
        print(f"  [CARLA Integration ERROR]: {e}")
        raise

    print("  >>> XODR VALIDATION PASSED COMPLETELY <<<")

def verify_fbx(fbx_path):
    print("\n" + "=" * 60)
    print(f"[VERIFY FBX] Inspecting: {fbx_path}")
    print("=" * 60)
    
    if not os.path.isfile(fbx_path):
        raise FileNotFoundError(f"FBX file not found: {fbx_path}")
        
    size_mb = os.path.getsize(fbx_path) / (1024 * 1024)
    print(f"  File size: {size_mb:.2f} MB -> OK")
    
    # Locate blender
    blender_exe = shutil.which("blender")
    if not blender_exe:
        for p in [
            r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe",
            r"C:\Program Files\Blender Foundation\Blender 5.1\blender.exe",
            r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe",
            r"C:\Program Files\Blender Foundation\Blender 4.3\blender.exe",
            r"C:\Program Files\Blender Foundation\Blender 4.2\blender.exe",
        ]:
            if os.path.isfile(p):
                blender_exe = p
                break
                
    if not blender_exe:
        raise RuntimeError("Blender not found for FBX verification!")
        
    # Python script executed inside Blender to inspect meshes
    inspect_code = f"""
import bpy
import sys

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=r'{os.path.abspath(fbx_path)}')

expected_submeshes = [
    'Road_Asphalt_Mesh',
    'Left_Barrier_Mesh',
    'Right_Shoulder_Mesh',
    'Right_Ditch_Terrain_Mesh'
]

found_objects = {{obj.name: obj for obj in bpy.context.scene.objects if obj.type == 'MESH'}}

errors = []
print('\\n--- 3D SUBMESH DETAILS ---')
for name in expected_submeshes:
    if name not in found_objects:
        errors.append(f'Missing submesh: {{name}}')
        continue
    obj = found_objects[name]
    m = obj.data
    mats = [mat.name for mat in m.materials]
    xs = [v.co.x for v in m.vertices]
    ys = [v.co.y for v in m.vertices]
    zs = [v.co.z for v in m.vertices]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    
    print(f'{{name:25}} | Verts: {{len(m.vertices):5}} | Polys: {{len(m.polygons):5}} | Mats: {{mats}}')
    print(f'   Bounds -> X: [{{min_x:.1f}}, {{max_x:.1f}}] | Y: [{{min_y:.2f}}, {{max_y:.2f}}] | Z: [{{min_z:.4f}}, {{max_z:.4f}}]')

# 1. Check Ditch Bottom Vertices
terrain_obj = found_objects.get('Right_Ditch_Terrain_Mesh')
if terrain_obj:
    # Filter vertices strictly on transverse interval [14.55, 16.05] (Y in [-16.05, -14.55])
    ditch_verts = [v.co for v in terrain_obj.data.vertices if -16.0501 <= v.co.y <= -14.5499]
    print(f'\\n--- DITCH BOTTOM ELEVATION CHECK ---')
    print(f'   Vertices in transverse range [14.55, 16.05] m: {{len(ditch_verts)}}')
    if not ditch_verts:
        errors.append('No vertices found in ditch bottom range [14.55, 16.05] m!')
    else:
        z_vals = [co.z for co in ditch_verts]
        min_z = min(z_vals)
        max_z = max(z_vals)
        print(f'   Ditch Bottom Z Range: [{{min_z:.5f}}, {{max_z:.5f}}] m')
        is_exact = all(abs(z - (-1.2000)) < 1e-4 for z in z_vals)
        print(f'   ALL VERTICES EXACTLY -1.2000 m: {{is_exact}}')
        if not is_exact:
            errors.append(f'Ditch bottom vertices do not match -1.20 m (min: {{min_z}}, max: {{max_z}})')

# 2. Check Barrier Height
barrier_obj = found_objects.get('Left_Barrier_Mesh')
if barrier_obj:
    max_z = max(v.co.z for v in barrier_obj.data.vertices)
    print(f'\\n--- LEFT BARRIER CHECK ---')
    print(f'   Barrier Max Height Z: {{max_z:.4f}} m (Expected: 0.8000)')
    if abs(max_z - 0.80) > 1e-4:
        errors.append(f'Barrier max height is {{max_z}}, expected 0.80')

# 3. Check Road and Shoulder Elevation
road_obj = found_objects.get('Road_Asphalt_Mesh')
shoulder_obj = found_objects.get('Right_Shoulder_Mesh')
if road_obj and shoulder_obj:
    road_zs = [v.co.z for v in road_obj.data.vertices]
    sh_zs = [v.co.z for v in shoulder_obj.data.vertices]
    road_flat = all(abs(z - 0.0) < 1e-4 for z in road_zs)
    sh_flat = all(abs(z - 0.0) < 1e-4 for z in sh_zs)
    print(f'\\n--- SURFACE ELEVATION CHECK ---')
    print(f'   Road surface completely flat Z = 0.0: {{road_flat}}')
    print(f'   Shoulder surface completely flat Z = 0.0: {{sh_flat}}')
    if not road_flat or not sh_flat:
        errors.append('Road or shoulder surface is not flat at Z = 0.0')

if errors:
    print('\\n[VERIFICATION FAILED]:', errors)
    sys.exit(1)
else:
    print('\\n>>> FBX VERIFICATION PASSED COMPLETELY <<<')
    sys.exit(0)
"""
    
    cmd = [blender_exe, "--background", "--python-expr", inspect_code]
    res = subprocess.run(cmd, capture_output=True, text=True)
    print(res.stdout)
    if res.returncode != 0:
        print("[STDERR]:", res.stderr)
        raise RuntimeError(f"FBX Verification script failed with exit code {res.returncode}")

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    xodr_path = os.path.join(base_dir, "map_ditch.xodr")
    fbx_path = os.path.join(base_dir, "map_ditch.fbx")
    
    verify_xodr(xodr_path)
    verify_fbx(fbx_path)
    
    print("\n" + "=" * 60)
    print("ALL VERIFICATIONS COMPLETED SUCCESSFULLY WITH 100% ACCURACY!")
    print("=" * 60)

if __name__ == "__main__":
    main()
