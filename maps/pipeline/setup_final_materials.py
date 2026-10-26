import unreal

editor_asset = unreal.EditorAssetLibrary
mat_lib = unreal.MaterialEditingLibrary

mat_configs = [
    {
        'mat_name': 'MAT_Asphalt',
        'diff': '/Game/StarterContent/Textures/T_Rock_Basalt_D',
        'norm': '/Game/StarterContent/Textures/T_Rock_Basalt_N',
        'tint': unreal.LinearColor(0.25, 0.25, 0.28, 1.0),
        'roughness': 0.6,
        'u_tile': 5.0,
        'v_tile': 5.0
    },
    {
        'mat_name': 'MAT_Grass_Terrain',
        'diff': '/Game/StarterContent/Textures/T_Ground_Grass_D',
        'norm': '/Game/StarterContent/Textures/T_Ground_Grass_N',
        'tint': unreal.LinearColor(0.40, 1.10, 0.35, 1.0),
        'roughness': 0.85,
        'u_tile': 8.0,
        'v_tile': 8.0
    },
    {
        'mat_name': 'MAT_Gravel_Shoulder',
        'diff': '/Game/StarterContent/Textures/T_Ground_Gravel_D',
        'norm': '/Game/StarterContent/Textures/T_Ground_Gravel_N',
        'tint': unreal.LinearColor(1.10, 1.00, 0.85, 1.0),
        'roughness': 0.9,
        'u_tile': 6.0,
        'v_tile': 6.0
    },
    {
        'mat_name': 'MAT_Concrete_Barrier',
        'diff': '/Game/StarterContent/Textures/T_Concrete_Poured_D',
        'norm': '/Game/StarterContent/Textures/T_Concrete_Poured_N',
        'tint': unreal.LinearColor(1.00, 1.00, 1.00, 1.0),
        'roughness': 0.5,
        'u_tile': 2.0,
        'v_tile': 2.0
    }
]

for cfg in mat_configs:
    mat_path = f"/Game/{cfg['mat_name']}"
    mat = editor_asset.load_asset(mat_path)
    if not mat:
        unreal.log_error(f"Failed to load material {mat_path}")
        continue
    
    unreal.log_warning(f"Configuring node graph for {cfg['mat_name']}...")
    mat_lib.delete_all_material_expressions(mat)
    
    # TexCoord
    coord = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -700, 0)
    coord.set_editor_property('u_tiling', cfg['u_tile'])
    coord.set_editor_property('v_tiling', cfg['v_tile'])
    
    # Diffuse Sample
    t_diff = editor_asset.load_asset(cfg['diff'])
    sample_diff = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, -150)
    sample_diff.set_editor_property('texture', t_diff)
    sample_diff.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    mat_lib.connect_material_expressions(coord, "", sample_diff, "Coordinates")
    
    # Tint Constant3Vector
    const_tint = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -450, 50)
    const_tint.set_editor_property('constant', cfg['tint'])
    
    # Multiply: Diffuse * Tint -> Base Color
    mult = mat_lib.create_material_expression(mat, unreal.MaterialExpressionMultiply, -200, -50)
    mat_lib.connect_material_expressions(sample_diff, "", mult, "A")
    mat_lib.connect_material_expressions(const_tint, "", mult, "B")
    mat_lib.connect_material_property(mult, "", unreal.MaterialProperty.MP_BASE_COLOR)
    
    # Normal Sample
    t_norm = editor_asset.load_asset(cfg['norm'])
    sample_norm = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, 250)
    sample_norm.set_editor_property('texture', t_norm)
    sample_norm.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
    mat_lib.connect_material_expressions(coord, "", sample_norm, "Coordinates")
    mat_lib.connect_material_property(sample_norm, "", unreal.MaterialProperty.MP_NORMAL)
    
    # Roughness Constant
    const_rough = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -200, 450)
    const_rough.set_editor_property('r', cfg['roughness'])
    mat_lib.connect_material_property(const_rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    
    mat_lib.recompile_material(mat)
    editor_asset.save_loaded_asset(mat)
    unreal.log_warning(f"  [OK] Recompiled & saved {cfg['mat_name']}")

# Assign to Static Meshes
meshes = [
    ('/Game/map_ditch_Road_Asphalt_Mesh', '/Game/MAT_Asphalt'),
    ('/Game/map_ditch_Right_Shoulder_Mesh', '/Game/MAT_Gravel_Shoulder'),
    ('/Game/map_ditch_Right_Ditch_Terrain_Mesh', '/Game/MAT_Grass_Terrain'),
    ('/Game/map_ditch_Left_Barrier_Mesh', '/Game/MAT_Concrete_Barrier'),
]

for m_path, m_mat in meshes:
    sm = editor_asset.load_asset(m_path)
    mat = editor_asset.load_asset(m_mat)
    if sm and mat:
        sm.set_material(0, mat)
        editor_asset.save_loaded_asset(sm)
        unreal.log_warning(f"Assigned {m_mat} to {m_path}")

# Open and update Level
map_path = '/Game/Carla/Maps/map_ditch/map_ditch'
world = unreal.EditorLoadingAndSavingUtils.load_map(map_path)
if world:
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    mat_asphalt = editor_asset.load_asset('/Game/MAT_Asphalt')
    mat_grass = editor_asset.load_asset('/Game/MAT_Grass_Terrain')
    mat_gravel = editor_asset.load_asset('/Game/MAT_Gravel_Shoulder')
    mat_concrete = editor_asset.load_asset('/Game/MAT_Concrete_Barrier')
    
    for a in actors:
        name = a.get_name()
        comp = a.get_component_by_class(unreal.StaticMeshComponent)
        if comp:
            if 'Road' in name and mat_asphalt:
                comp.set_material(0, mat_asphalt)
            elif 'Shoulder' in name and mat_gravel:
                comp.set_material(0, mat_gravel)
            elif 'Ditch' in name and mat_grass:
                comp.set_material(0, mat_grass)
            elif 'Barrier' in name and mat_concrete:
                comp.set_material(0, mat_concrete)
                
    saved = unreal.EditorLoadingAndSavingUtils.save_map(world, map_path)
    unreal.log_warning(f"Updated and saved map {map_path}: {saved}")

unreal.log_warning("SUCCESSFULLY REBUILT ALL 4 MATERIAL GRAPHS WITH TEXTURES & TINTS!")
