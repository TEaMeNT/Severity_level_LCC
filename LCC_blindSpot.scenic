# ==============================================================================
# SCENIC SCENARIO: Lane Change Collision (LCC) with Blind Spot Monitoring Failure
# ISO 26262-3 / SAE J2980 Hazard Analysis and Risk Assessment (HARA)
# ==============================================================================
#
# КАРТА И СИСТЕМЫ КООРДИНАТ:
# - Дорожная сеть: map_ditch.xodr (прямой загородный участок автомагистрали, 2 полосы).
# - Направление движения ТС: строго на Восток (+X в глобальной системе CARLA).
# - Полоса -1 (левая): центр Y = +1.875 м. Слева расположен бетонный отбойник (Y <= 0.0 м).
# - Полоса -2 (правая): центр Y = +5.625 м. Справа расположена обочина (Y = 7.50..9.75 м)
#   и дренажный кювет с уклоном 1:4 (Y = 9.75..14.55 м, Z = -1.20 м).
# - В Scenic при направлении на Восток (heading = -pi/2):
#   * локальная ось X направлена вправо/влево от кузова ТС (поперечная координата);
#   * локальная ось Y направлена вперед/назад от кузова ТС (продольная координата).
#
# ПАРАМЕТРЫ СЛЕПОЙ ЗОНЫ (Строго по исходному ТЗ проекта, неизменяемые константы):
#   EGO_LENGTH = 4.93 м, EGO_WIDTH = 1.86 м (седан Lincoln MKZ 2017)
#   TARGET_LENGTH = 4.18 м, TARGET_WIDTH = 1.83 м (купе Audi TT)
#   TOP_BLIND_SPOT = (EGO_LENGTH / 2) - 2.0 = +0.465 м (граница у центральной стойки)
#   BOTTOM_BLIND_SPOT = -(EGO_LENGTH / 2) - 3.0 = -5.465 м (3 м позади заднего бампера)
#   NEAR_BLIND_SPOT = 1.01 м (боковой зазор до борта)
#   FAR_BLIND_SPOT = 4.15 м (внешняя граница смежной полосы)
#   TARGET_CENTER_Y_FRONT = TOP_BLIND_SPOT - (TARGET_LENGTH / 2) = -1.625 м
#   TARGET_CENTER_Y_BACK = BOTTOM_BLIND_SPOT = -5.465 м
# ==============================================================================

param map = "C:/Users/teament/Desktop/xlam/work/maps/assets/map_ditch.xodr"
param carla_map = 'map_ditch'
param use2DMap = True
param render = False
param timestep = 0.01
param weather = 'ClearNoon'
model scenic.simulators.carla.model

# Геометрические габариты участников сценария
EGO_LENGTH = 4.93    # длина Lincoln MKZ 2017 в CARLA
EGO_WIDTH = 1.86     # ширина Lincoln MKZ 2017 в CARLA
TARGET_LENGTH = 4.18 # длина Audi TT в CARLA
TARGET_WIDTH = 1.83  # ширина Audi TT в CARLA

class EgoCar(Car):
    width: EGO_WIDTH  
    length: EGO_LENGTH 

class TargetCar(Car):
    width: TARGET_WIDTH 
    length: TARGET_LENGTH

# Границы зоны необнаружения датчиков слепых зон (Исходное неизменяемое ТЗ)
TOP_BLIND_SPOT = (EGO_LENGTH / 2) - 2.0 
BOTTOM_BLIND_SPOT = -(EGO_LENGTH / 2) - 3.0
NEAR_BLIND_SPOT = 1.01
FAR_BLIND_SPOT = 4.15

# Допустимый диапазон продольного положения центра целевого автомобиля
TARGET_CENTER_Y_FRONT = TOP_BLIND_SPOT - (TARGET_LENGTH / 2)
TARGET_CENTER_Y_BACK = BOTTOM_BLIND_SPOT

# Параметр подкласса дорожных условий:
# 'SUBCLASS_A_BARRIER' - левая полоса ограничена отбойником (маневр влево)
# 'SUBCLASS_C_DITCH'   - правая полоса ограничена кюветом (маневр вправо)
param subclass = 'SUBCLASS_C_DITCH'
SPEED_MPS = 100 / 3.6
start_x = 50

# БЛОК 1: Параметризация дорожных подклассов и стохастического расположения ТС
# В глобальных координатах Scenic:
# - start_y задает стартовую полосу автомобиля Ego:
#   * Для Подкласса А: Lane -2 (правая полоса, start_y = -5.625).
#     Целевое ТС располагается в ЛЕВОЙ слепой зоне: target_offset_x < 0.
#   * Для Подкласса С: Lane -1 (левая полоса, start_y = -1.875).
#     Целевое ТС располагается в ПРАВОЙ слепой зоне: target_offset_x > 0.
if globalParameters.subclass == 'SUBCLASS_A_BARRIER':
    start_y = -5.625
    target_offset_x = Range(-FAR_BLIND_SPOT, -(NEAR_BLIND_SPOT + TARGET_WIDTH / 2 + 0.1))
else:
    start_y = -1.875
    target_offset_x = Range(NEAR_BLIND_SPOT + TARGET_WIDTH / 2 + 0.1, FAR_BLIND_SPOT)

# Продольное смещение по исходному ТЗ: случайная точка в пределах слепой зоны
target_offset_y = Range(TARGET_CENTER_Y_BACK, TARGET_CENTER_Y_FRONT)

# Поведение TrafficManagerBehavior: передает управление симуляцией на сторону Python
behavior TrafficManagerBehavior():
    while True:
        wait

# Спавн основного ТС (EgoCar) на крейсерской позиции
ego = new EgoCar at (start_x, start_y, 0.2),
    facing roadDirection,
    with blueprint 'vehicle.lincoln.mkz_2017',
    with behavior TrafficManagerBehavior()

# Спавн целевого ТС (TargetCar) со смещением в слепую зону
target = new TargetCar offset by Vector(target_offset_x, target_offset_y),
    facing roadDirection,
    with blueprint 'vehicle.audi.tt',
    with behavior TrafficManagerBehavior()

# Ограничение длительности симуляции одного прогона (30 секунд = 3000 тиков)
terminate after 30 seconds

param v0_val = SPEED_MPS