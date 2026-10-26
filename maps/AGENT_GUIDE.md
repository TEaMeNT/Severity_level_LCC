# AI Agent Operational Runbook: Headless FBX to CARLA Pipeline
> **Практическое руководство для автономных AI-агентов (LLM Coding Assistants) и инженеров автоматизации.**
> Описывает пошаговый протокол полной сборки кастомных локаций без графического интерфейса (100% headless CLI).

---

## 1. Возможности автономного агента

### Может ли AI-агент выполнить весь пайплайн самостоятельно?
**ОТВЕТ: ДА, НА 100%.**
AI-агенту с доступом к командной строке (PowerShell / Bash) **не требуется открытие окон Unreal Engine Editor или Blender**. Движок Unreal Engine 4 предоставляет полноценную консольную утилиту `UE4Editor-Cmd.exe`, которая поддерживает:
1. Выполнение произвольного Python-кода внутри окружения редактора (`-run=pythonscript`).
2. Неинтерактивную компиляцию и кукинг ассетов (`-run=cook`).
3. Пакетное развертывание и валидацию через Python CARLA API.

---

## 2. Разделение обязанностей: Человек vs AI-Агент

### Что требуется от ЧЕЛОВЕКА (пользователя):
Пользователю достаточно обеспечить окружение на машине:
1. **Установленный Unreal Engine 4.26** (стандартный путь: `C:\Program Files\Epic Games\UE_4.26\`).
2. **Установленный бинарный релиз CARLA 0.9.x** (стандартный путь: `C:\CARLA\`).
3. **Наличие пустого/базового проекта UE4** (например, `C:\Users\teament\Documents\Unreal Projects\map_ditch\map_ditch.uproject`).
4. **Текстовое задание или исходная 3D-модель**:
   - Вариант А: Положить `.fbx` и `.xodr` в папку `assets/`.
   - Вариант Б: Просто написать агенту в чат параметры: *«Сгенерируй прямую трассу 2 км с кюветом 1:3 и разделительным барьером»* (агент сам сгенерирует математическую модель через Blender).

### Что выполняет AI-АГЕНТ (автономно):
1. Проверяет конфигурацию проекта `DefaultGame.ini` на отключение `bShareMaterialShaderCode`.
2. Генерирует Python-скрипт сборщика уровня.
3. Запускает `UE4Editor-Cmd.exe` в фоновом режиме для импорта мешей, построения PBR-материалов и сборки уровня `.umap`.
4. Запускает cook-командлет `UE4Editor-Cmd.exe -run=cook`.
5. Копирует скомпилированные файлы `.uasset`, `.uexp`, `.xodr` и текстуры в директорию `C:\CARLA\CarlaUE4\Content\`.
6. Запускает процесс симулятора `CarlaUE4-Win64-Shipping.exe`.
7. Выполняет тестовый Python-скрипт: спавнит автомобили, проверяет физическую высоту $Z$ на коллизии полотна, снимает RGB-скриншот и подтверждает отсутствие засветов и черного экрана.

---

## 3. Пошаговый протокол действий для AI-агента

### Этап 1. Проверка и патчинг конфигурации UE4
Перед сборкой агент обязан проверить файл `<ProjectDir>\Config\DefaultGame.ini`. Если секция отсутствует, добавить:
```ini
[/Script/UnrealEd.ProjectPackagingSettings]
bShareMaterialShaderCode=False
bSharedMaterialNativeLibraries=False
```
*(Это предотвратит откат материалов в WorldGridMaterial).*

---

### Этап 2. Сборка уровня через headless Python скрипт
Агент запускает скрипт сборщика внутри UE4 командой:
```powershell
& "C:\Program Files\Epic Games\UE_4.26\Engine\Binaries\Win64\UE4Editor-Cmd.exe" `
  "<PathToUproject>" `
  -run=pythonscript -script="<PathToMapsDir>\pipeline\import_and_cook_map.py"
```

#### Что делает `import_and_cook_map.py` (шаблон кода):
```python
import unreal

editor_asset = unreal.EditorAssetLibrary
mat_lib = unreal.MaterialEditingLibrary
level_lib = unreal.EditorLevelLibrary

# 1. Полноценный PBR граф материала
def create_pbr_mat(mat_name, diff_path, norm_path, tint_rgb, roughness, u_tile, v_tile):
    mat_path = f"/Game/{mat_name}"
    mat = editor_asset.load_asset(mat_path)
    mat_lib.delete_all_material_expressions(mat)

    # UV тайлинг
    coord = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -700, 0)
    coord.set_editor_property('u_tiling', u_tile)
    coord.set_editor_property('v_tiling', v_tile)

    # Diffuse * PBR Tint
    sample_diff = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, -150)
    sample_diff.set_editor_property('texture', editor_asset.load_asset(diff_path))
    sample_diff.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    mat_lib.connect_material_expressions(coord, "", sample_diff, "Coordinates")

    const_tint = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -450, 50)
    const_tint.set_editor_property('constant', tint_rgb)

    mult = mat_lib.create_material_expression(mat, unreal.MaterialExpressionMultiply, -200, -50)
    mat_lib.connect_material_expressions(sample_diff, "", mult, "A")
    mat_lib.connect_material_expressions(const_tint, "", mult, "B")
    mat_lib.connect_material_property(mult, "", unreal.MaterialProperty.MP_BASE_COLOR)

    # Normal Map
    sample_norm = mat_lib.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -450, 250)
    sample_norm.set_editor_property('texture', editor_asset.load_asset(norm_path))
    sample_norm.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
    mat_lib.connect_material_expressions(coord, "", sample_norm, "Coordinates")
    mat_lib.connect_material_property(sample_norm, "", unreal.MaterialProperty.MP_NORMAL)

    # Roughness
    const_rough = mat_lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -200, 450)
    const_rough.set_editor_property('r', roughness)
    mat_lib.connect_material_property(const_rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

    mat_lib.recompile_material(mat)
    editor_asset.save_loaded_asset(mat)

# 2. Физическая коллизия меша
def set_complex_collision(mesh_path, mat_name):
    sm = editor_asset.load_asset(mesh_path)
    if sm:
        bs = sm.get_editor_property('body_setup')
        if bs:
            bs.set_editor_property('collision_complexity', unreal.CollisionComplexity.CTF_USE_COMPLEX_AS_SIMPLE)
        sm.set_material(0, editor_asset.load_asset(f"/Game/{mat_name}"))
        editor_asset.save_loaded_asset(sm)

# 3. Сборка уровня .umap со светом
def build_level(map_name, mesh_names):
    world = unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    for m in mesh_names:
        sm = editor_asset.load_asset(f"/Game/{m}")
        level_lib.spawn_actor_from_object(sm, unreal.Vector(0,0,0), unreal.Rotator(0,0,0))
    
    # Солнце (боковое, для рельефа кювета)
    sun = level_lib.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0,0,1000), unreal.Rotator(-32.0, 55.0, 0.0))
    sun_c = sun.get_component_by_class(unreal.DirectionalLightComponent)
    sun_c.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
    sun_c.set_editor_property('intensity', 45000.0)
    sun_c.set_editor_property('cast_shadows', True)
    sun_c.set_editor_property('dynamic_shadow_distance_movable_light', 40000.0)

    # Небосвод
    sky = level_lib.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0,0,1000))
    sky_c = sky.get_component_by_class(unreal.SkyLightComponent)
    sky_c.set_editor_property('mobility', unreal.ComponentMobility.MOVABLE)
    sky_c.set_editor_property('real_time_capture', True)
    sky_c.set_editor_property('intensity', 1.0)

    level_lib.spawn_actor_from_class(unreal.SkyAtmosphere, unreal.Vector(0,0,0))
    level_lib.spawn_actor_from_class(unreal.ExponentialHeightFog, unreal.Vector(0,0,0))

    # Постпроцесс (антиблик)
    pp = level_lib.spawn_actor_from_class(unreal.PostProcessVolume, unreal.Vector(0,0,0))
    s = pp.get_editor_property('settings')
    s.set_editor_property('auto_exposure_bias', -0.6)
    s.set_editor_property('bloom_intensity', 0.05)
    s.set_editor_property('ambient_occlusion_intensity', 0.8)
    pp.set_editor_property('settings', s)
    pp.set_editor_property('unbound', True)

    # Динамический режим
    world.get_world_settings().set_editor_property('force_no_precomputed_lighting', True)

    unreal.EditorLoadingAndSavingUtils.save_map(world, f"/Game/Carla/Maps/{map_name}/{map_name}")
```

---

### Этап 3. Запуск кукинга (Cook Commandlet)
Агент запускает компиляцию:
```powershell
& "C:\Program Files\Epic Games\UE_4.26\Engine\Binaries\Win64\UE4Editor-Cmd.exe" `
  "<PathToUproject>" `
  -run=cook -targetplatform=WindowsNoEditor -cookonthefly=0 -unattended
```
*Условие успеха:* В выводе лога должно быть `Success - 0 error(s)`.

---

### Этап 4. Автоматический деплой файлов в CARLA
Агент перемещает скомпилированные артефакты из:
`<ProjectDir>\Saved\Cooked\WindowsNoEditor\<ProjectName>\Content\`
в
`C:\CARLA\CarlaUE4\Content\`:

```powershell
$src = "<ProjectDir>\Saved\Cooked\WindowsNoEditor\<ProjectName>\Content"
$dst = "C:\CARLA\CarlaUE4\Content"

# Остановка CARLA перед копированием во избежание блокировки файлов
Stop-Process -Name "CarlaUE4-Win64-Shipping", "CarlaUE4" -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 1

# 1. Материалы и меши
Copy-Item "$src\MAT_*" "$dst\" -Force
Copy-Item "$src\<map_name>_*" "$dst\" -Force

# 2. Текстуры
New-Item -ItemType Directory -Path "$dst\StarterContent\Textures" -Force | Out-Null
Copy-Item "$src\StarterContent\Textures\*" "$dst\StarterContent\Textures\" -Force -Recurse

# 3. Уровень .umap и .uexp
$dstMap = "$dst\Carla\Maps\<map_name>"
New-Item -ItemType Directory -Path $dstMap -Force | Out-Null
Copy-Item "$src\Carla\Maps\<map_name>\<map_name>.*" "$dstMap\" -Force

# 4. OpenDRIVE .xodr
$xodrSrc = "<PathToMapsDir>\assets\<map_name>.xodr"
Copy-Item $xodrSrc "$dstMap\<map_name>.xodr" -Force
Copy-Item $xodrSrc "$dstMap\OpenDrive\<map_name>.xodr" -Force
Copy-Item $xodrSrc "$dst\Carla\Maps\OpenDrive\<map_name>.xodr" -Force
```

---

### Этап 5. Запуск симулятора и верификация через Python
1. **Запуск сервера CARLA (фоновый процесс):**
   ```powershell
   cmd.exe /c "C:\CARLA\CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe CarlaUE4 -stdout -FullStdOutLogOutput -windowed -ResX=1280 -ResY=720"
   ```
2. **Проверка через Python-тест:**
   ```python
   import carla

   client = carla.Client('localhost', 2000)
   client.set_timeout(30.0)
   world = client.load_world('map_ditch')

   # Проверка PhysX высоты
   bp = world.get_blueprint_library().filter('vehicle.lincoln.mkz_2017')[0]
   wp = world.get_map().get_waypoint(carla.Location(x=50.0, y=5.625, z=0.0))
   tf = wp.transform
   tf.location.z += 1.0
   veh = world.spawn_actor(bp, tf)

   for _ in range(25):
       world.tick()

   z = veh.get_location().z
   assert z > -1.0, f"Ошибка коллизии: автомобиль провалился (Z = {z})"
   print(f"[УСПЕХ] Автомобиль стабильно стоит на коллизии: Z = {z:.3f} м")
   ```

---

## 4. Быстрый запуск в 1 команду
В папке `pipeline/` подготовлен единый универсальный PowerShell-скрипт `build_and_deploy_map.ps1`. Для любой новой карты агенту достаточно выполнить:
```powershell
powershell -ExecutionPolicy Bypass -File pipeline/build_and_deploy_map.ps1 -MapName "имя_новой_карты"
```
Скрипт автоматически выполнит шаги 2, 3 и 4 без необходимости ручного вмешательства.
