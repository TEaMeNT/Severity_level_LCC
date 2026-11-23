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
# ПАРАМЕТРЫ СЛЕПОЙ ЗОНЫ И ГАБАРИТЫ (CARLA 0.9.16 bbox, геометрия слепой зоны неизменна):
#   EGO_LENGTH = 5.36 м, EGO_WIDTH = 2.03 м (vehicle.chevrolet.impala: 5.3575 x 2.0332 м)
#   TARGET_LENGTH = 4.19 м, TARGET_WIDTH = 1.82 м (vehicle.seat.leon: 4.1928 x 1.8162 м)
#   TOP_BLIND_SPOT = (EGO_LENGTH / 2) - 2.0 = +0.68 м (2.0 м от переднего бампера)
#   BOTTOM_BLIND_SPOT = -(EGO_LENGTH / 2) - 3.0 = -5.68 м (3.0 м позади заднего бампера)
#   NEAR_BLIND_SPOT = 1.01 м (боковой зазор до оси Ego)
#   FAR_BLIND_SPOT = 4.15 м (внешняя граница смежной полосы)
#   TARGET_CENTER_Y_FRONT = TOP_BLIND_SPOT - (TARGET_LENGTH / 2) = -1.415 м
#   TARGET_CENTER_Y_BACK = BOTTOM_BLIND_SPOT = -5.680 м
# ==============================================================================

param map = localPath("maps/assets/map_ditch.xodr")
param carla_map = 'map_ditch'
param use2DMap = True
param render = False
param timestep = 0.01
param weather = 'ClearNoon'
model scenic.simulators.carla.model

# Геометрические габариты участников сценария (точные значения CARLA 0.9.16)
EGO_LENGTH = 5.36    # длина vehicle.chevrolet.impala в CARLA (5.3575 м)
EGO_WIDTH = 2.03     # ширина vehicle.chevrolet.impala в CARLA (2.0332 м)
TARGET_LENGTH = 4.19 # длина vehicle.seat.leon в CARLA (4.1928 м)
TARGET_WIDTH = 1.82  # ширина vehicle.seat.leon в CARLA (1.8162 м)

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
# 'SUBCLASS_B_DITCH'   - правая полоса ограничена кюветом (маневр вправо)
param subclass = 'SUBCLASS_B_DITCH'
# Стохастическая снаряжённая масса Target (кг): B/C/D/SUV выборка (EEA / парк РФ)
param target_mass_kg = TruncatedNormal(1480.0, 220.0, 1100.0, 2050.0)
SPEED_MPS = 100 / 3.6
start_x = 50

# БЛОК 1: Параметризация дорожных подклассов и стохастического расположения ТС
# В глобальных координатах Scenic:
# - start_y задает стартовую полосу автомобиля Ego:
#   * Для Подкласса А: Lane -2 (правая полоса, start_y = -5.625).
#     Целевое ТС располагается в ЛЕВОЙ слепой зоне: target_offset_x < 0.
#   * Для Подкласса B: Lane -1 (левая полоса, start_y = -1.875).
#     Целевое ТС располагается в ПРАВОЙ слепой зоне: target_offset_x > 0.
if globalParameters.subclass == 'SUBCLASS_A_BARRIER':
    start_y = -5.625
    target_offset_x = Range(-FAR_BLIND_SPOT, -(NEAR_BLIND_SPOT + TARGET_WIDTH / 2 + 0.1))
elif globalParameters.subclass == 'SUBCLASS_B_DITCH':
    start_y = -1.875
    target_offset_x = Range(NEAR_BLIND_SPOT + TARGET_WIDTH / 2 + 0.1, FAR_BLIND_SPOT)
else:
    raise ValueError("Unsupported subclass parameter: " + repr(globalParameters.subclass))

# Продольное смещение по исходному ТЗ: случайная точка в пределах слепой зоны
target_offset_y = Range(TARGET_CENTER_Y_BACK, TARGET_CENTER_Y_FRONT)

# Поведение TrafficManagerBehavior: передает управление симуляцией на сторону Python
behavior TrafficManagerBehavior():
    while True:
        wait

# Спавн основного ТС (EgoCar) на крейсерской позиции
ego = new EgoCar at (start_x, start_y, 0.2),
    facing roadDirection,
    with blueprint 'vehicle.chevrolet.impala',
    with rolename 'ego',
    with behavior TrafficManagerBehavior()

# Спавн целевого ТС (TargetCar) со смещением в слепую зону
target = new TargetCar offset by Vector(target_offset_x, target_offset_y),
    facing roadDirection,
    with blueprint 'vehicle.seat.leon',
    with rolename 'target',
    with behavior TrafficManagerBehavior()

# Ограничение длительности симуляции одного прогона (45 секунд = 4500 тиков при dt=0.01)
terminate after 45 seconds

param v0_val = SPEED_MPS