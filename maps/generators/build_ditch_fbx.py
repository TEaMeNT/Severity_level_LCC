"""
Generates Blender 3D model for road and ditch scenario.
This script is executed inside Blender in background mode.
"""
import sys
import bpy
import bmesh
from mathutils import Vector

def create_material(name, color_rgba, roughness=0.7):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = color_rgba
        bsdf.inputs["Roughness"].default_value = roughness
    return mat

def build_grid_mesh(mesh_name, x_steps, y_z_profile, uv_scale_y=1.0):
    """
    Builds a ribbon mesh from a 2D (Y, Z) cross-section profile swept along X.
    x_steps: list of X coordinates (e.g. 0.0 to 1500.0 with step 5.0)
    y_z_profile: list of (Y, Z) tuples sorted from higher Y to lower Y (e.g. 0 to -30.85)
    """
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.verify()
    
    num_x = len(x_steps)
    num_p = len(y_z_profile)
    
    # Create vertices
    vert_grid = []
    for i, x in enumerate(x_steps):
        row = []
        for j, (y, z) in enumerate(y_z_profile):
            v = bm.verts.new((x, y, z))
            row.append(v)
        vert_grid.append(row)
        
    bm.verts.ensure_lookup_table()
    
    # Create quad faces with normal pointing UP (+Z)
    for i in range(num_x - 1):
        for j in range(num_p - 1):
            v0 = vert_grid[i][j]       # (x_i, y_j)
            v1 = vert_grid[i][j + 1]   # (x_i, y_{j+1})
            v2 = vert_grid[i + 1][j + 1] # (x_{i+1}, y_{j+1})
            v3 = vert_grid[i + 1][j]   # (x_{i+1}, y_j)
            
            # Since y_{j+1} < y_j, v0 -> v1 is along -Y, v1 -> v2 is along +X
            # Winding v0 -> v1 -> v2 -> v3 has normal (-Y x +X) = +Z
            face = bm.faces.new((v0, v1, v2, v3))
            
            # UV mapping
            for loop in face.loops:
                vx = loop.vert.co.x
                vy = loop.vert.co.y
                loop[uv_layer].uv = (vx / 10.0, vy / uv_scale_y)
                
    bm.normal_update()
    
    mesh = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(mesh)
    bm.free()
    return mesh

def build_barrier_mesh(mesh_name, x_steps, y_inner=0.0, y_outer=1.0, z_height=0.80):
    """
    Builds a solid volumetric box mesh for the median barrier.
    """
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.verify()
    
    num_x = len(x_steps)
    
    # For each X slice, 4 vertices:
    # 0: bottom inner (y_inner, 0.0)
    # 1: bottom outer (y_outer, 0.0)
    # 2: top outer (y_outer, z_height)
    # 3: top inner (y_inner, z_height)
    slices = []
    for x in x_steps:
        v0 = bm.verts.new((x, y_inner, 0.0))
        v1 = bm.verts.new((x, y_outer, 0.0))
        v2 = bm.verts.new((x, y_outer, z_height))
        v3 = bm.verts.new((x, y_inner, z_height))
        slices.append((v0, v1, v2, v3))
        
    bm.verts.ensure_lookup_table()
    
    for i in range(num_x - 1):
        s0 = slices[i]
        s1 = slices[i + 1]
        
        # Top face (Z = +0.80): pointing +Z
        # s0[3] -> s1[3] -> s1[2] -> s0[2]
        f_top = bm.faces.new((s0[3], s1[3], s1[2], s0[2]))
        
        # Outer face (Y = 1.0): pointing +Y
        # s0[1] -> s0[2] -> s1[2] -> s1[1]
        f_out = bm.faces.new((s0[1], s0[2], s1[2], s1[1]))
        
        # Inner face (Y = 0.0): pointing -Y
        # s0[0] -> s1[0] -> s1[3] -> s0[3]
        f_in = bm.faces.new((s0[0], s1[0], s1[3], s0[3]))
        
        # Bottom face (Z = 0.0): pointing -Z
        # s0[0] -> s0[1] -> s1[1] -> s1[0]
        f_bot = bm.faces.new((s0[0], s0[1], s1[1], s1[0]))
        
        for f in (f_top, f_out, f_in, f_bot):
            for loop in f.loops:
                loop[uv_layer].uv = (loop.vert.co.x / 10.0, loop.vert.co.z / 1.0)
                
    # End caps at x = 0 and x = 1500
    # Cap at X = 0 (pointing -X):
    # s0: (v0, v3, v2, v1)
    f_cap0 = bm.faces.new((slices[0][0], slices[0][3], slices[0][2], slices[0][1]))
    # Cap at X = 1500 (pointing +X):
    # s_last: (v0, v1, v2, v3)
    f_cap1 = bm.faces.new((slices[-1][0], slices[-1][1], slices[-1][2], slices[-1][3]))
    
    for f in (f_cap0, f_cap1):
        for loop in f.loops:
            loop[uv_layer].uv = (loop.vert.co.y, loop.vert.co.z)
            
    bm.normal_update()
    
    mesh = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(mesh)
    bm.free()
    return mesh

def main(output_fbx_path):
    print(f"Starting Blender generation -> {output_fbx_path}")
    
    # Reset scene
    bpy.ops.wm.read_factory_settings(use_empty=True)
    
    # 1500m length, step 5.0m -> 301 slices
    L = 1500.0
    step = 5.0
    num_steps = int(round(L / step)) + 1
    x_steps = [i * step for i in range(num_steps)]
    
    # Create Materials
    mat_asphalt = create_material("MAT_Asphalt", (0.10, 0.10, 0.11, 1.0), roughness=0.6)
    mat_barrier = create_material("MAT_Concrete_Barrier", (0.55, 0.55, 0.52, 1.0), roughness=0.7)
    mat_shoulder = create_material("MAT_Gravel_Shoulder", (0.35, 0.32, 0.28, 1.0), roughness=0.85)
    mat_grass = create_material("MAT_Grass_Terrain", (0.18, 0.35, 0.12, 1.0), roughness=0.9)
    
    # 1. Road_Asphalt_Mesh
    # Covers two lanes: lane -1 (0 to 3.75) and lane -2 (3.75 to 7.50)
    asphalt_profile = [
        (0.00, 0.0),    # Centerline
        (-3.75, 0.0),   # Lane divider
        (-7.50, 0.0),   # Outer edge of travel lanes
    ]
    mesh_asphalt = build_grid_mesh("Road_Asphalt_Mesh", x_steps, asphalt_profile, uv_scale_y=3.75)
    obj_asphalt = bpy.data.objects.new("Road_Asphalt_Mesh", mesh_asphalt)
    obj_asphalt.data.materials.append(mat_asphalt)
    bpy.context.collection.objects.link(obj_asphalt)
    
    # 2. Right_Shoulder_Mesh
    # Paved/reinforced shoulder from 7.50 to 9.75 m (width 2.25 m)
    shoulder_profile = [
        (-7.50, 0.0),   # Joint with asphalt
        (-8.625, 0.0),  # Mid shoulder
        (-9.75, 0.0),   # Outer shoulder edge
    ]
    mesh_shoulder = build_grid_mesh("Right_Shoulder_Mesh", x_steps, shoulder_profile, uv_scale_y=2.25)
    obj_shoulder = bpy.data.objects.new("Right_Shoulder_Mesh", mesh_shoulder)
    obj_shoulder.data.materials.append(mat_shoulder)
    bpy.context.collection.objects.link(obj_shoulder)
    
    # 3. Right_Ditch_Terrain_Mesh
    # Cross section with 0.25m bevels at joints:
    # - Crest at 9.75m: bevel from 9.75 to 10.00m (smooth transition into 1:4 slope)
    # - 1:4 inner slope from 10.00 to 14.30m (drop 1.075m over 4.30m)
    # - Toe bevel from 14.30 to 14.55m into horizontal bottom
    # - Ditch bottom flat from 14.55 to 16.05m at Z = -1.200m
    # - Toe bevel from 16.05 to 16.30m into 1:4 outer slope
    # - 1:4 outer slope from 16.30 to 20.60m (rise 1.075m over 4.30m)
    # - Crest bevel from 20.60 to 20.85m into horizontal terrain
    # - Roadside terrain from 20.85 to 30.85m at Z = 0.000m
    ditch_profile = [
        (-9.75,   0.0000),   # Crest joint with shoulder
        (-9.85,  -0.0150),   # Crest bevel mid
        (-10.00, -0.0625),   # 1:4 slope start
        (-12.15, -0.6000),   # Inner slope mid
        (-14.30, -1.1375),   # 1:4 slope end / toe bevel start
        (-14.45, -1.1850),   # Toe bevel mid
        (-14.55, -1.2000),   # Ditch bottom start
        (-15.30, -1.2000),   # Ditch bottom mid
        (-16.05, -1.2000),   # Ditch bottom end
        (-16.15, -1.1850),   # Toe bevel mid
        (-16.30, -1.1375),   # 1:4 outer slope start
        (-18.45, -0.6000),   # Outer slope mid
        (-20.60, -0.0625),   # 1:4 outer slope end / crest bevel start
        (-20.75, -0.0150),   # Crest bevel mid
        (-20.85,  0.0000),   # Roadside field start
        (-25.85,  0.0000),   # Roadside field mid
        (-30.85,  0.0000),   # Roadside field outer boundary
    ]
    mesh_ditch = build_grid_mesh("Right_Ditch_Terrain_Mesh", x_steps, ditch_profile, uv_scale_y=10.0)
    obj_ditch = bpy.data.objects.new("Right_Ditch_Terrain_Mesh", mesh_ditch)
    obj_ditch.data.materials.append(mat_grass)
    bpy.context.collection.objects.link(obj_ditch)
    
    # 4. Left_Barrier_Mesh
    # Volumetric concrete barrier: Y in [0.0, 1.0], Z in [0.0, 0.80]
    mesh_barrier = build_barrier_mesh("Left_Barrier_Mesh", x_steps, y_inner=0.0, y_outer=1.0, z_height=0.80)
    obj_barrier = bpy.data.objects.new("Left_Barrier_Mesh", mesh_barrier)
    obj_barrier.data.materials.append(mat_barrier)
    bpy.context.collection.objects.link(obj_barrier)
    
    # Set smooth shading for road/ditch, sharp for barrier
    for poly in mesh_asphalt.polygons:
        poly.use_smooth = True
    for poly in mesh_shoulder.polygons:
        poly.use_smooth = True
    for poly in mesh_ditch.polygons:
        poly.use_smooth = True
        
    bpy.context.view_layer.update()
    
    # Export FBX with axis_forward='X', axis_up='Z', global_scale=1.0
    print("Exporting FBX...")
    bpy.ops.export_scene.fbx(
        filepath=output_fbx_path,
        check_existing=False,
        use_selection=False,
        global_scale=1.0,
        apply_unit_scale=True,
        apply_scale_options='FBX_SCALE_ALL',
        axis_forward='X',
        axis_up='Z',
        object_types={'MESH'},
        use_mesh_modifiers=True,
        mesh_smooth_type='FACE',
        bake_anim=False
    )
    print("FBX Export completed successfully!")

if __name__ == "__main__":
    # Check if custom output path passed via args after '--'
    out_path = "map_ditch.fbx"
    if "--" in sys.argv:
        idx = sys.argv.index("--")
        if idx + 1 < len(sys.argv):
            out_path = sys.argv[idx + 1]
    main(out_path)
