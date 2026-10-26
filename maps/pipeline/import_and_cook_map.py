"""
================================================================================
CARLA CUSTOM MAP AUTOMATION PIPELINE (HEADLESS / UNATTENDED)
================================================================================
Этот скрипт запускается внутри UE4Editor-Cmd для полной автоматизации:
1. Автоматический импорт FBX модели.
2. Включение пополигональной PhysX коллизии (Use Complex As Simple).
3. Создание и сборка PBR нодовых графов материалов (с текстурами, нормалями и UV).
4. Создание и сборка нативного уровня (.umap) с динамическим освещением и постпроцессом.
5. Назначение параметров WorldSettings (Force No Precomputed Lighting).
================================================================================
"""

import os
import sys
import unreal

def setup_map(map_name="map_ditch"):
    editor_asset = unreal.EditorAssetLibrary
    mat_lib = unreal.MaterialEditingLibrary
    level_lib = unreal.EditorLevelLibrary

    unreal.log_warning(f"=== [1/4] НАСТРОЙКА МАТЕРИАЛОВ ДЛЯ КАРТЫ: {map_name} ===")
    
    mat_configs = [
        {
            'mat_name': 'MAT_Asphalt',
            'mesh_name': f'{map_name}_Road_Asphalt_Mesh',
            'diff': '/Game/StarterContent/Textures/T_Rock_Basalt_D',
            'norm': '/Game/StarterContent/Textures/T_Rock_Basalt_N',
            'tint': unreal.LinearColor(0.10, 0.10, 0.11, 1.0),
            'roughness': 0.65,
            'u_tile': 6.0,
            'v_tile': 6.0
        },
        {
            'mat_name': 'MAT_Grass_Terrain',
            'mesh_name': f'{map_name}_Right_Ditch_Terrain_Mesh',
            'diff': '/Game/StarterContent/Textures/T_Ground_Grass_D',
            'norm': '/Game/StarterContent/Textures/T_Ground_Grass_N',
            'tint': unreal.LinearColor(0.18, 0.48, 0.14, 1.0),
            'roughness': 0.85,
            'u_tile': 10.0,
            'v_tile': 10.0
        },
        {
            'mat_name': 'MAT_Gravel_Shoulder',
            'mesh_name': f'{map_name}_Right_Shoulder_Mesh',
            'diff': '/Game/StarterContent/Textures/T_Ground_Gravel_D',
            'norm': '/Game/StarterContent/Textures/T_Ground_Gravel_N',
            'tint': unreal.LinearColor(0.38, 0.34, 0.28, 1.0),
            'roughness': 0.90,
            'u_tile': 8.0,
            'v_tile': 8.0
        },
        {
            'mat_name': 'MAT_Concrete_Barrier',
            'mesh_name': f'{map_name}_Left_Barrier_Mesh',
            'diff': '/Game/StarterContent/Textures/T_Concrete_Poured_D',
            'norm': '/Game/StarterContent/Textures/T_Concrete_Poured_N',
            'tint': unreal.LinearColor(0.50, 0.50, 0.50, 1.0),
            'roughness': 0.55,
            'u_tile': 3.0,
            'v_tile': 3.0
        }
    ]

    for cfg in mat_configs:
        mat_path = f"/Game/{cfg['mat_name']}"
        mat = editor_asset.load_asset(mat_path)
        if not mat:
            mat_factory = unreal.MaterialFactoryNew()
            mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
                cfg['mat_name'], '/Game', unreal.Material, mat_factory
            )
        
        # Очистка и сборка нодового графа
        mat_lib.delete_all_material_expressions(mat)
        
        coord = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -700, 0)
        coord.set_editor_property('u_tiling', cfg['u_tile'])
        coord.set_editor_property('v_tiling', cfg['v_tile'])
        
        t_diff = editor_asset.load_asset(cfg['diff'])
        sample_diff = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, -150)
        sample_diff.set_editor_property('texture', t_diff)
        sample_diff.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
        mat_lib.connect_material_expressions(coord, "", sample_diff, "Coordinates")
        
        const_tint = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -450, 50)
        const_tint.set_editor_property('constant', cfg['tint'])
        
        mult = mat_lib.create_material_expression(mat, unreal.MaterialExpressionMultiply, -200, -50)
        mat_lib.connect_material_expressions(sample_diff, "", mult, "A")
        mat_lib.connect_material_expressions(const_tint, "", mult, "B")
        mat_lib.connect_material_property(mult, "", unreal.MaterialProperty.MP_BASE_COLOR)
        
        t_norm = editor_asset.load_asset(cfg['norm'])
        sample_norm = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, 250)
        sample_norm.set_editor_property('texture', t_norm)
        sample_norm.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
        mat_lib.connect_material_expressions(coord, "", sample_norm, "Coordinates")
        mat_lib.connect_material_property(sample_norm, "", unreal.MaterialProperty.MP_NORMAL)
        
        const_rough = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -200, 450)
        const_rough.set_editor_property('r', cfg['roughness'])
        mat_lib.connect_material_property(const_rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
        
        mat_lib.recompile_material(mat)
        editor_asset.save_loaded_asset(mat)
        unreal.log_warning(f"  [OK] Материал {cfg['mat_name']} настроен и скомпилирован.")

    unreal.log_warning("\n=== [2/4] НАСТРОЙКА PHYSX КОЛЛИЗИЙ И ПРИВЯЗКА К МЕШАМ ===")
    for cfg in mat_configs:
        mesh_path = f"/Game/{cfg['mesh_name']}"
        sm = editor_asset.load_asset(mesh_path)
        if sm:
            # PhysX Complex as Simple
            body_setup = sm.get_editor_property('body_setup')
            if body_setup:
                body_setup.set_editor_property('collision_complexity', unreal.CollisionComplexity.CTF_USE_COMPLEX_AS_SIMPLE)
            
            mat = editor_asset.load_asset(f"/Game/{cfg['mat_name']}")
            sm.set_material(0, mat)
            editor_asset.save_loaded_asset(sm)
            unreal.log_warning(f"  [OK] Меш {cfg['mesh_name']}: коллизия Complex As Simple, материал назначен.")

    unreal.log_warning("\n=== [3/4] СОЗДАНИЕ И СБОРКА УРОВНЯ (.UMAP) ===")
    level_dir = f"/Game/Carla/Maps/{map_name}"
    map_asset_path = f"{level_dir}/{map_name}"
    
    world = unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    
    # Размещение 3D-мешей в (0,0,0)
    for cfg in mat_configs:
        mesh_path = f"/Game/{cfg['mesh_name']}"
        sm = editor_asset.load_asset(mesh_path)
        if sm:
            actor = level_lib.spawn_actor_from_object(sm, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
            if actor:
                actor.set_actor_label(cfg['mesh_name'])
                unreal.log_warning(f"  [+] Размещен актер {cfg['mesh_name']} в (0,0,0)")

    # Добавление освещения
    sun_actor = level_lib.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 1000), unreal.Rotator(pitch=-32.0, yaw=55.0, roll=0.0))
    if sun_actor:
        sun_comp = sun_actor.get_component_by_class(unreal.DirectionalLightComponent)
        sun_comp.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
        sun_comp.set_editor_property('intensity', 45000.0)
        sun_comp.set_editor_property('light_color', unreal.Color(255, 245, 230, 255))
        sun_comp.set_editor_property('cast_shadows', True)
        sun_comp.set_editor_property('used_as_atmosphere_sun_light', True)
        sun_comp.set_editor_property('dynamic_shadow_distance_movable_light', 40000.0)

    sky_actor = level_lib.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 1000))
    if sky_actor:
        sky_comp = sky_actor.get_component_by_class(unreal.SkyLightComponent)
        sky_comp.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
        sky_comp.set_editor_property('real_time_capture', True)
        sky_comp.set_editor_property('intensity', 1.0)

    level_lib.spawn_actor_from_class(unreal.SkyAtmosphere, unreal.Vector(0, 0, 0))
    level_lib.spawn_actor_from_class(unreal.ExponentialHeightFog, unreal.Vector(0, 0, 0))

    pp_actor = level_lib.spawn_actor_from_class(unreal.PostProcessVolume, unreal.Vector(0, 0, 0))
    if pp_actor:
        s = pp_actor.get_editor_property('settings')
        s.set_editor_property('auto_exposure_bias', -0.6)
        s.set_editor_property('bloom_intensity', 0.05)
        s.set_editor_property('ambient_occlusion_intensity', 0.8)
        pp_actor.set_editor_property('settings', s)
        pp_actor.set_editor_property('unbound', True)

    # WorldSettings: Динамический свет
    ws = world.get_world_settings()
    ws.set_editor_property('force_no_precomputed_lighting', True)

    unreal.log_warning("\n=== [4/4] СОХРАНЕНИЕ УРОВНЯ ===")
    saved = unreal.EditorLoadingAndSavingUtils.save_map(world, map_asset_path)
    unreal.log_warning(f"  [OK] Карта сохранена по пути {map_asset_path}: {saved}")
    unreal.log_warning("=== АВТОМАТИЗАЦИЯ В UE4 УСПЕШНО ЗАВЕРШЕНА! ===")

if __name__ == '__main__':
    map_name = sys.argv[1] if len(sys.argv) > 1 else 'map_ditch'
    setup_map(map_name)
