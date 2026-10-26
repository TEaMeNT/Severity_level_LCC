import unreal

editor_asset = unreal.EditorAssetLibrary
mat_lib = unreal.MaterialEditingLibrary

# 1. PBR-correct material tints (Physically realistic albedo)
mat_configs = [
    {
        'mat_name': 'MAT_Asphalt',
        'diff': '/Game/StarterContent/Textures/T_Rock_Basalt_D',
        'norm': '/Game/StarterContent/Textures/T_Rock_Basalt_N',
        'tint': unreal.LinearColor(0.10, 0.10, 0.11, 1.0), # Realistic dark asphalt
        'roughness': 0.65,
        'u_tile': 6.0,
        'v_tile': 6.0
    },
    {
        'mat_name': 'MAT_Grass_Terrain',
        'diff': '/Game/StarterContent/Textures/T_Ground_Grass_D',
        'norm': '/Game/StarterContent/Textures/T_Ground_Grass_N',
        'tint': unreal.LinearColor(0.18, 0.48, 0.14, 1.0), # Realistic green meadow
        'roughness': 0.85,
        'u_tile': 10.0,
        'v_tile': 10.0
    },
    {
        'mat_name': 'MAT_Gravel_Shoulder',
        'diff': '/Game/StarterContent/Textures/T_Ground_Gravel_D',
        'norm': '/Game/StarterContent/Textures/T_Ground_Gravel_N',
        'tint': unreal.LinearColor(0.38, 0.34, 0.28, 1.0), # Earthy sandy gravel
        'roughness': 0.90,
        'u_tile': 8.0,
        'v_tile': 8.0
    },
    {
        'mat_name': 'MAT_Concrete_Barrier',
        'diff': '/Game/StarterContent/Textures/T_Concrete_Poured_D',
        'norm': '/Game/StarterContent/Textures/T_Concrete_Poured_N',
        'tint': unreal.LinearColor(0.50, 0.50, 0.50, 1.0), # Clean neutral concrete
        'roughness': 0.55,
        'u_tile': 3.0,
        'v_tile': 3.0
    }
]

for cfg in mat_configs:
    mat_path = f"/Game/{cfg['mat_name']}"
    mat = editor_asset.load_asset(mat_path)
    if not mat:
        continue
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

# 2. Update Map Lighting & Post-Processing
map_path = '/Game/Carla/Maps/map_ditch/map_ditch'
world = unreal.EditorLoadingAndSavingUtils.load_map(map_path)
if world:
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    for a in actors:
        cls = a.get_class().get_name()
        if 'DirectionalLight' in cls:
            comp = a.get_component_by_class(unreal.DirectionalLightComponent)
            if comp:
                comp.set_editor_property('intensity', 45000.0) # Reduced from 65k to prevent overexposure
                comp.set_editor_property('light_color', unreal.Color(255, 245, 230, 255))
                # Angled sun to highlight ditch relief
                a.set_actor_rotation(unreal.Rotator(pitch=-32.0, yaw=55.0, roll=0.0), False)
                unreal.log_warning('DirectionalLight updated: 45k Lux, angled shadow pitch -32, yaw 55')
        elif 'SkyLight' in cls:
            comp = a.get_component_by_class(unreal.SkyLightComponent)
            if comp:
                comp.set_editor_property('intensity', 1.0) # Balanced ambient fill
        elif 'PostProcessVolume' in cls:
            s = a.get_editor_property('settings')
            # Exposure & glare control
            s.set_editor_property('auto_exposure_bias', -0.6) # Darker exposure to bring out texture detail
            s.set_editor_property('bloom_intensity', 0.1)     # Eliminate blinding glare/bloom
            s.set_editor_property('ambient_occlusion_intensity', 0.8) # Deepen relief creases
            s.set_editor_property('ambient_occlusion_radius', 300.0)
            a.set_editor_property('settings', s)
            unreal.log_warning('PostProcessVolume updated: Bias -0.6, Bloom 0.1, AO 0.8')

    unreal.EditorLoadingAndSavingUtils.save_map(world, map_path)
    unreal.log_warning('Saved tuned map_ditch successfully!')
