# ==============================================================================
# HARA SIMULATION RUNNER: Lane Change Collision (LCC) Blind Spot Failure
# Стандарты: ISO 26262-3 (HARA) / SAE J2980 (Appendix B - Severity, Appendix E - Shock)
# Инженерный рефакторинг: двусторонний мониторинг столкновений, 6-режимная
# террамеханика смены покрытий, разделение Controllability (C) и Severity (S).
# ==============================================================================
#
# СИСТЕМА И АРХИТЕКТУРА ИССЛЕДОВАНИЯ:
# Мы исследуем сценарий функционального отказа контроля слепых зон (Blind Spot Detection / LCC).
# При отказе автомобиль Ego начинает перестроение на крейсерской скорости 100 км/ч (27.78 м/с)
# в соседнюю полосу, в которой находится целевое ТС (Target) в пределах слепой зоны.
#
# СИСТЕМА КООРДИНАТ КАРТЫ MAP_DITCH:
# - Продольная ось дороги: +X (направление на Восток, ограничение 90 км/ч, крейсерская 100 км/ч).
# - Поперечная ось дороги: +Y (вправо по ходу движения ТС).
# - Вертикальная ось: +Z (вверх).
# - Полоса -1 (левая): центр Y = +1.875 м. Слева (Y <= 0.0 м) жесткий бетонный отбойник New Jersey (h = 0.80 м).
# - Полоса -2 (правая): центр Y = +5.625 м. Справа (Y >= 7.50 м) обочина и водоотводный кювет.
#   * Полоса -3, Y in [7.50, 9.75) м: укрепленная грунтово-гравийная обочина (Shoulder, Z = 0.0 м).
#   * Полоса -4, Y in [9.75, 14.55) м: внутренний откос кювета с уклоном 1:4 (~14°, спуск до Z = -1.20 м).
#   * Полоса -5, Y in [14.55, 16.05) м: дно водоотводного кювета (Ditch Bed, Z = -1.20 м).
#   * Полоса -6, Y in [16.05, 20.85] м: обратный откос кювета с уклоном 1:4 (подъем до Z = 0.0 м).
#
# ИНЖЕНЕРНЫЕ БЛОКИ РЕФАКТОРИНГА:
# 1. Привязка карты map_ditch и универсальное разрешение путей (CLI --map-dir).
# 2. Когнитивная модель водителя (SAE J2980 Appendix E) и разделение метрик S и C (--driver-post-shock):
#    - Фаза шока T_shock ~ N(0.85, 0.12) с [0.65, 1.05] с: спазм 0.25 с, далее кастор (*= 0.95), педали 0.0.
#    - Пост-шок: 'passive' (Severity S, выбег), 'imperfect' (паника SAE J2980), 'active' (Controllability C).
# 3. Двусторонний мониторинг столкновений (Ego + Target) по ISO 26262-3:
#    - Датчики коллизий установлены на оба автомобиля.
#    - Регистрация и векторный расчет удара о бетонный отбойник как для Ego, так и для вытолкнутого Target.
# 4. Научно обоснованная 6-режимная модель трения шин по центру ТС (loc.y) для CARLA PhysX:
#    - Режим 1: Сухой асфальт — штатное качение (tire_friction = 3.50, lat_stiff = 20.0).
#    - Режим 2: Сухой асфальт — боковой юз после удара (tire_friction = 2.45, lat_stiff = 18.0).
#    - Режим 3: Укрепленная гравийная обочина [7.50, 9.75) м (tire_friction = 1.90, drag soil_mu = 0.06).
#    - Режим 4: Откос/дно кювета — прямое качение (tire_friction = 1.25, drag soil_mu = 0.22).
#    - Режим 5: Откос/дно кювета — боковое зарывание и спотыкание (NHTSA Soil-Trip):
#               опорные шины 3.20 / разгруженные 0.35, f_furrow = 0.35g, f_plow = 0.25g,
#               опрокидывающий момент (* 3500.0) при |V_lat| > 3.2 м/с, вертикальный подскок 0.25g на дне.
#    - Режим 6: Возврат на асфальт (Road Recovery) — восстановление штатного сцепления 3.50 при Y < 7.50 м.
# 5. Итоговая каскадная агрегация тяжести ISO 26262-3 (max по всем факторам) и логирование в CSV.
# ==============================================================================

import os
import sys
import time
import math
import random
import csv
import argparse
import traceback
from collections import deque

# Настройка кодировки стандартного вывода на Windows для предотвращения кракозябр
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

import carla
import scenic
from scenic.simulators.carla.simulator import CarlaSimulator

# ------------------------------------------------------------------------------
# Carla Python API Configuration
# ------------------------------------------------------------------------------
CARLA_API_PATH = r"C:\CARLA\PythonAPI\carla"
if CARLA_API_PATH not in sys.path:
    sys.path.append(CARLA_API_PATH)
try:
    from agents.navigation.controller import VehiclePIDController  # type: ignore
except ImportError:
    VehiclePIDController = None

# ------------------------------------------------------------------------------
# Пути к файлам проекта и физические константы
# ------------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENIC_PATH = os.path.join(BASE_DIR, "LCC_blindSpot.scenic")
RESULTS_DIR = os.path.join(BASE_DIR, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)
CSV_PATH = os.path.join(RESULTS_DIR, "lcc_hara_results.csv")
CARLA_MAP_NAME = "map_ditch"

# Список стандартных путей к ассетам карты map_ditch
DEFAULT_MAP_DIRS = [
    os.path.join(BASE_DIR, "maps", "assets"),
    os.path.join(BASE_DIR, "..", "maps", "assets"),
    r"C:\Users\teament\Desktop\xlam\work\maps\assets",
]


def resolve_map_assets(custom_dir: str = None) -> tuple[str, str]:
    """
    Разрешение пути к папке ассетов карты и файлу OpenDRIVE map_ditch.xodr.
    Поддерживает CLI-переопределение через --map-dir, относительные пути и fallback.
    """
    candidates = []
    if custom_dir:
        candidates.append(os.path.abspath(custom_dir))
    candidates.extend(DEFAULT_MAP_DIRS)
    candidates.append(BASE_DIR)

    for c_dir in candidates:
        if os.path.isdir(c_dir):
            xodr_ditch = os.path.join(c_dir, "map_ditch.xodr")
            if os.path.isfile(xodr_ditch):
                return c_dir, xodr_ditch
            xodr_generic = os.path.join(c_dir, "map.xodr")
            if os.path.isfile(xodr_generic):
                return c_dir, xodr_generic

    # Аварийный fallback на текущую рабочую директорию
    fallback_xodr = os.path.join(BASE_DIR, "map.xodr")
    return BASE_DIR, fallback_xodr


# Физические параметры участников (Строго по ТЗ проекта):
# Ego: Lincoln MKZ 2017 (масса 3490 кг как аналог тяжелого представительского седана Аурус Сенат)
EGO_MASS = 3490.0        # Масса ТС Ego (кг)
EGO_H_CG = 0.65          # Высота центра тяжести ТС Ego над опорной поверхностью (м)

# Target: Audi TT (легкое спорткупе массой 2000 кг)
TARGET_MASS = 2000.0     # Масса целевого ТС (кг)
TARGET_H_CG = 0.52       # Высота центра тяжести целевого ТС (м)

# Коэффициент восстановления при пластической деформации кузовных панелей (SAE J2980)
RESTITUTION_E = 0.15

# Скоростной режим трассы (загородная дорога с ограничением 90 км/ч, крейсерская 100 км/ч)
SPEED_LIMIT_MPS = 90.0 / 3.6    # 25.0 м/с
CRUISE_SPEED_MPS = 100.0 / 3.6  # 27.78 м/с

# Идентификаторы дорожных подклассов
SUBCLASS_A = "SUBCLASS_A_BARRIER"  # Маневр влево: риск удара о бетонный отбойник (вес 0.20)
SUBCLASS_C = "SUBCLASS_C_DITCH"    # Маневр вправо: риск схода и опрокидывания в кювете (вес 0.80)


def normalize_deg(angle: float) -> float:
    """Нормализация углов Эйлера к диапазону [-180, 180] градусов."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def safe_apply_physics_control(vehicle, phys):
    """
    КРИТИЧЕСКИ ВАЖНЫЙ ФИКС БАГА ФИЗИЧЕСКОГО ДВИЖКА CARLA PhysX:

    В симуляторе CARLA вызов метода vehicle.apply_physics_control(phys) приводит
    к пересозданию внутреннего объекта динамического твердого тела в PhysX.
    По умолчанию в PhysX при этом линейная скорость (velocity) и угловая скорость
    (angular_velocity) сбрасываются в вектор (0, 0, 0)!

    Данная функция кэширует мгновенные векторы поступательного и вращательного движения
    за доли миллисекунды до применения настроек физики и немедленно восстанавливает их
    через set_target_velocity() и set_target_angular_velocity(), строго сохраняя
    физический импульс и кинематику движения ТС в пространстве.
    """
    if vehicle is None:
        return
    vel = vehicle.get_velocity()
    ang_vel = vehicle.get_angular_velocity()
    vehicle.apply_physics_control(phys)
    vehicle.set_target_velocity(vel)
    vehicle.set_target_angular_velocity(ang_vel)


# ==============================================================================
# AUTOPILOT & SIMULATION MANAGER
# ==============================================================================
class AutopilotManager:
    """
    Менеджер симуляции HARA LCC Blind Spot:
    - Конфигурация Ego и Target, поддержание строя в слепой зоне.
    - Инициализация и обработка двух датчиков столкновений (Ego + Target).
    - Когнитивная модель водителя SAE J2980 с тремя режимами пост-шока ('passive', 'imperfect', 'active').
    - 6-режимная модель террамеханики смены дорожных покрытий по центру ТС.
    - Расчет метрик тяжести по стандартам ISO 26262-3 и SAE J2980.
    """

    def __init__(self, client, world, subclass: str = SUBCLASS_C, driver_post_shock: str = 'passive'):
        self.client = client
        self.world = world
        self.subclass = subclass
        self.driver_post_shock = driver_post_shock

        if self.client is not None:
            self.tm = self.client.get_trafficmanager(8000)
            self.tm.set_synchronous_mode(True)
        else:
            self.tm = None

        self.ego_vehicle = None
        self.target_vehicle = None
        self.configured = False
        self.is_running = False
        self.spectator = None
        self.tick_count = 0
        self.callback_id = None

        self.lane_change_triggered = False
        self.lost_control = False
        self.collision_tick = None

        self.target_v0_mps = CRUISE_SPEED_MPS
        self.ego_tm_speed = self.target_v0_mps + 0.4

        self.initial_lon_offset = 0.0
        self.target_err_integral = 0.0
        self.stability_counter = 0

        self.pid_controller = None
        self.start_lane_id = None

        # Датчики столкновений для обоих ТС (ISO 26262-3)
        self.ego_collision_sensor = None
        self.target_collision_sensor = None
        self.speed_history = {}

        # Кэши состояний террамеханики и истории скоростей
        self.vehicle_terramech_state = {}  # actor_id -> state str
        self.ditch_v_history = {}          # actor_id -> deque of carla.Vector3D (maxlen=20)

        # ----------------------------------------------------------------------
        # БЛОК 2: Когнитивная модель водителя (SAE J2980 Appendix E)
        # ----------------------------------------------------------------------
        # Время акустико-вестибулярного ступора: 0.65 - 1.05 с, медиана 0.85 с
        self.shock_duration_sec = min(1.05, max(0.65, random.gauss(0.85, 0.12)))
        self.shock_duration_ticks = int(self.shock_duration_sec / 0.01)
        self.last_steer = 0.0
        self.impact_steer = 0.0
        self.current_steer = 0.0

        # ----------------------------------------------------------------------
        # БЛОК 3: Первичное столкновение ТС с ТС
        # ----------------------------------------------------------------------
        self.side_collision_frame = None
        self.v_closing_lat = 0.0
        self.delta_v_ego = 0.0
        self.delta_v_target = 0.0
        self.effective_side_delta_v = 0.0
        self.sev_side_val = 0

        # ----------------------------------------------------------------------
        # БЛОК 4.1: Вторичное столкновение с отбойником (Подкласс А)
        # ----------------------------------------------------------------------
        self.barrier_collision_frame = None
        self.barrier_impact_speed = 0.0
        self.barrier_impact_type = ""
        self.sev_barrier_val = 0
        self.barrier_departed_actors = set()  # {'ego', 'target'}

        # ----------------------------------------------------------------------
        # БЛОК 4.2: Вторичное опрокидывание / закапывание в кювете (Подкласс С)
        # ----------------------------------------------------------------------
        self.departed_actors = set()     # {'ego', 'target'}
        self.departure_speeds = {}       # actor_id -> float (mps)
        self.max_tilt_deg = {}           # actor_id -> float (deg)
        self.max_lateral_slip = {}       # actor_id -> float (mps)
        self.wheel_furrowing_triggered = False
        self.max_ditch_3d_dv = {}        # actor_id -> float (mps)
        self.sev_ditch_val = 0

    # --------------------------------------------------------------------------
    # Обработчик столкновений (Collision Sensor Callback для обоих ТС)
    # --------------------------------------------------------------------------
    def on_collision(self, event, source_actor: str = 'ego'):
        """
        Универсальный обработчик столкновений для Ego и Target по ISO 26262-3:
        - Обнаруживает первичное боковое столкновение ТС-ТС.
        - Обнаруживает удар о бетонный отбойник для ЛЮБОГО из ТС (Ego или Target).
        """
        other = event.other_actor
        frame = event.frame

        hist = self.speed_history.get(frame - 1)
        if not hist:
            hist = self.speed_history.get(frame)
        if not hist and self.speed_history:
            closest_frame = max(self.speed_history.keys())
            hist = self.speed_history[closest_frame]
        if not hist:
            return

        ego_v = hist['ego_v']
        ego_right = hist['ego_right']
        ego_forward = hist['ego_forward']
        target_v = hist['target_v']
        target_forward = hist['target_forward']
        lane_forward = hist['lane_forward']
        lane_right = hist['lane_right']

        # ----------------------------------------------------------------------
        # 1. Первичный удар ТС с ТС (поперечный импульс по SAE J2980)
        # ----------------------------------------------------------------------
        if 'vehicle' in other.type_id.lower():
            if self.side_collision_frame is None:
                self.side_collision_frame = frame

                # Поперечная скорость сближения за 1 кадр до контакта
                v_rel_x = ego_v.x - target_v.x
                v_rel_y = ego_v.y - target_v.y
                self.v_closing_lat = abs((v_rel_x * ego_right.x) + (v_rel_y * ego_right.y))

                # Истинные переданные Delta-V по закону сохранения импульса
                m_ego = EGO_MASS
                m_target = TARGET_MASS
                e = RESTITUTION_E

                self.delta_v_ego = (m_target / (m_ego + m_target)) * (1.0 + e) * self.v_closing_lat
                self.delta_v_target = (m_ego / (m_ego + m_target)) * (1.0 + e) * self.v_closing_lat
                self.effective_side_delta_v = max(self.delta_v_ego, self.delta_v_target)

                # Калиброванная шкала SAE J2980 Table B1
                if self.effective_side_delta_v > 8.0:
                    self.sev_side_val = 3
                elif self.effective_side_delta_v > 3.5:
                    self.sev_side_val = 2
                elif self.effective_side_delta_v > 1.0:
                    self.sev_side_val = 1
                else:
                    self.sev_side_val = 0

                print(f"\n[{frame}] ->> СТОЛКНОВЕНИЕ ТС-ТС (SAE J2980): "
                      f"v_closing_lat = {self.v_closing_lat:.2f} м/с | "
                      f"Delta-V Ego = {self.delta_v_ego:.2f} м/с, "
                      f"Target = {self.delta_v_target:.2f} м/с | "
                      f"S_side = S{self.sev_side_val}")

                # При боковом ударе целевой автомобиль отключает автопилот (свободный выбег в PhysX)
                if self.target_vehicle is not None:
                    try:
                        self.target_vehicle.set_autopilot(False)
                        self.target_vehicle.apply_control(carla.VehicleControl(
                            throttle=0.0,
                            steer=0.0,
                            brake=0.0,
                            hand_brake=False,
                            manual_gear_shift=False
                        ))
                    except Exception:
                        pass

            # Активация фазы шока водителя Ego при первом боковом ударе
            if not self.lost_control:
                self.lost_control = True
                self.collision_tick = self.tick_count
                self.impact_steer = self.last_steer
                self.current_steer = self.impact_steer
                print(f"[{frame}] ->> ФАЗА ШОКА АКТИВИРОВАНА (EGO): T_shock = {self.shock_duration_sec:.2f}c "
                      f"({self.shock_duration_ticks} тиков). Фиксация руля: steer = {self.impact_steer:+.3f}. "
                      f"Режим водителя после шока: '{self.driver_post_shock}'.")

        # ----------------------------------------------------------------------
        # 2. Вторичный удар об отбойник (Подкласс А) для Ego или Target
        # ----------------------------------------------------------------------
        else:
            # Проверяем, что удар совершен именно об отбойник (зона бетонного ограждения Y <= 1.0 м)
            actor_veh = self.ego_vehicle if source_actor == 'ego' else self.target_vehicle
            if actor_veh is None:
                return
            try:
                loc_y = actor_veh.get_location().y
            except Exception:
                return
            if loc_y > 1.0:
                # В зоне обочины/кювета (Y >= 7.5 м) контакт с грунтом или дорожной поверхностью
                # не является ударом о бетонный отбойник New Jersey
                return

            # Извлекаем кинематику именно того ТС, которое совершило столкновение
            if source_actor == 'ego':
                v_actor = ego_v
                fw_actor = ego_forward
            else:
                v_actor = target_v
                fw_actor = target_forward

            # Нормальная скорость впечатывания в плоскость отбойника
            v_barrier = abs((v_actor.x * lane_right.x) + (v_actor.y * lane_right.y))

            # Косинус угла атаки относительно продольной оси полосы
            cos_alpha = (fw_actor.x * lane_forward.x) + (fw_actor.y * lane_forward.y)

            if abs(cos_alpha) < 0.707:
                # Угол атаки > 45 град (работают лонжероны и зоны деформации)
                cur_impact_type = 'Frontal/Rear'
                if v_barrier > 18.0:
                    cur_sev = 3
                elif v_barrier > 11.0:
                    cur_sev = 2
                elif v_barrier > 6.0:
                    cur_sev = 1
                else:
                    cur_sev = 0
            else:
                # Угол атаки <= 45 град (боковой скользящий удар без зон смятия)
                cur_impact_type = 'Side-swipe'
                if v_barrier > 12.0:
                    cur_sev = 3
                elif v_barrier > 6.0:
                    cur_sev = 2
                elif v_barrier > 2.0:
                    cur_sev = 1
                else:
                    cur_sev = 0

            self.barrier_departed_actors.add(source_actor)

            # Сохраняем первичный или более тяжелый удар в отбойник
            is_first = (self.barrier_collision_frame is None)
            is_worse = (cur_sev > self.sev_barrier_val) or (
                cur_sev == self.sev_barrier_val and v_barrier > self.barrier_impact_speed
            )

            if is_first or is_worse:
                self.barrier_collision_frame = frame
                self.barrier_impact_speed = v_barrier
                self.barrier_impact_type = cur_impact_type
                self.sev_barrier_val = cur_sev

            print(f"\n[{frame}] ->> УДАР ОБ ОТБОЙНИК ({source_actor.upper()}): "
                  f"Тип: {cur_impact_type} | "
                  f"Скорость удара = {v_barrier:.2f} м/с | "
                  f"S_barrier = S{cur_sev} (макс. S_barrier: S{self.sev_barrier_val})")

            # Активация шока для Ego при прямом ударе Ego об отбойник
            if not self.lost_control and source_actor == 'ego':
                self.lost_control = True
                self.collision_tick = self.tick_count
                self.impact_steer = self.last_steer
                self.current_steer = self.impact_steer

    # --------------------------------------------------------------------------
    # 6-РЕЖИМНАЯ МОДЕЛЬ СМЕНЫ ДОРОЖНЫХ ПОКРЫТИЙ И ТЕРРАМЕХАНИКИ
    # --------------------------------------------------------------------------
    def update_vehicle_surface_physics(self, vehicle, mass: float, h_cg: float, actor_name: str):
        """
        Единая 6-режимная модель физики сцепления шин и террамеханики по центру ТС (loc.y):
        - Режим 1: Сухой асфальт — штатное качение (Y < 7.50, |V_lat| <= 2.2 м/с).
        - Режим 2: Сухой асфальт — срыв в боковой юз (Y < 7.50, lost_control, |V_lat| > 2.2 м/с).
        - Режим 3: Укрепленная гравийная обочина [7.50, 9.75) м — сопротивление гравия (soil_mu=0.06).
        - Режим 4: Откос/дно кювета [9.75, 21.0] м — прямолинейное качение по грунту (soil_mu=0.22).
        - Режим 5: Откос/дно кювета [9.75, 21.0] м — боковое зарывание и спотыкание (NHTSA Soil-Trip).
        - Режим 6: Возврат на асфальт (Road Recovery) — возврат сцепления 3.50 при Y < 7.50 м.
        """
        loc = vehicle.get_location()

        # Замер метрик и физика действуют строго в пределах рельефа карты
        if loc.y > 21.0 or loc.z < -2.0:
            # ТС за пределами рельефа карты — фиксируем скорость и динамику PhysX,
            # предотвращая падение в бесконечную пустоту (UE4 KillZ crash)
            try:
                vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                vehicle.set_simulate_physics(False)
            except Exception:
                pass
            return

        tf = vehicle.get_transform()
        rot = tf.rotation
        v_vec = vehicle.get_velocity()
        fw_vec = tf.get_forward_vector()
        right_vec = tf.get_right_vector()

        # Кинематические проекции в локальных осях ТС
        v_lon = (v_vec.x * fw_vec.x) + (v_vec.y * fw_vec.y)
        v_lat = (v_vec.x * right_vec.x) + (v_vec.y * right_vec.y)
        v_total = math.hypot(v_vec.x, v_vec.y)
        sin_beta = abs(v_lat) / max(v_total, 0.5)

        # ----------------------------------------------------------------------
        # ЗОНА А: Проезжая часть (Сухой асфальт, Y < 7.50 м)
        # ----------------------------------------------------------------------
        if loc.y < 7.50:
            if self.lost_control and abs(v_lat) > 2.2:
                # Режим 2: Срыв в боковой юз на сухом асфальте
                target_state = 'asphalt_slide'
                if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                    phys = vehicle.get_physics_control()
                    for w in phys.wheels:
                        w.tire_friction = 2.45
                        w.lat_stiff_value = 18.0
                        w.lat_stiff_max_load = 1.5
                    safe_apply_physics_control(vehicle, phys)
                    self.vehicle_terramech_state[vehicle.id] = target_state
            else:
                # Режим 1 и Режим 6: Штатное качение или возврат на сухой асфальт
                target_state = 'asphalt_grip'
                if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                    phys = vehicle.get_physics_control()
                    for w in phys.wheels:
                        w.tire_friction = 3.50
                        w.lat_stiff_value = 20.0
                        w.lat_stiff_max_load = 2.0
                    safe_apply_physics_control(vehicle, phys)
                    self.vehicle_terramech_state[vehicle.id] = target_state
            # На асфальте внешние силы грунта отсутствуют (F_ext = 0)

        # ----------------------------------------------------------------------
        # ЗОНА Б: Укрепленная грунтово-гравийная обочина (7.50 <= Y < 9.75 м, Z = 0.0 м)
        # ----------------------------------------------------------------------
        elif loc.y < 9.75:
            # Режим 3: Плотный щебень/гравий
            target_state = 'shoulder'
            if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                phys = vehicle.get_physics_control()
                for w in phys.wheels:
                    w.tire_friction = 1.90
                    w.lat_stiff_value = 14.0
                    w.lat_stiff_max_load = 1.8
                safe_apply_physics_control(vehicle, phys)
                self.vehicle_terramech_state[vehicle.id] = target_state

            # Сопротивление качению щебня (soil_mu = 0.06)
            drag_scale = -0.06 * mass * 9.81 / max(v_total, 0.1)
            f_drag = carla.Vector3D(x=v_vec.x * drag_scale, y=v_vec.y * drag_scale, z=0.0)
            vehicle.add_force(f_drag)
            # Закапывание, фиксация кювета и опрокидывающий момент на обочине ЗАПРЕЩЕНЫ

        # ----------------------------------------------------------------------
        # ЗОНА В: Откос и дно кювета (9.75 <= Y <= 21.0 м)
        # ----------------------------------------------------------------------
        else:
            # 1. Фиксация схода в кювет строго при Y >= 9.75 м
            if vehicle.id not in self.departure_speeds:
                v_dep = math.hypot(v_vec.x, v_vec.y, v_vec.z)
                self.departure_speeds[vehicle.id] = v_dep
                self.departed_actors.add(actor_name)
                snap_frame = self.world.get_snapshot().frame if self.world else 0
                print(f"\n[{snap_frame}] ->> СХОД В КЮВЕТ ({actor_name.upper()}): "
                      f"Скорость схода = {v_dep*3.6:.1f} км/ч ({v_dep:.2f} м/с), Y = {loc.y:.2f} м")

            # 2. Обновление динамических метрик кювета
            tilt = max(abs(normalize_deg(rot.roll)), abs(normalize_deg(rot.pitch)))
            self.max_tilt_deg[vehicle.id] = max(self.max_tilt_deg.get(vehicle.id, 0.0), tilt)
            self.max_lateral_slip[vehicle.id] = max(self.max_lateral_slip.get(vehicle.id, 0.0), abs(v_lat))

            # Скользящее окно 3D-изменения скорости за 20 тиков (0.20 с)
            if vehicle.id not in self.ditch_v_history:
                self.ditch_v_history[vehicle.id] = deque(maxlen=20)
            dq = self.ditch_v_history[vehicle.id]
            dq.append(v_vec)
            if len(dq) == 20:
                v_old = dq[0]
                dv_3d = math.sqrt(
                    (v_vec.x - v_old.x)**2 +
                    (v_vec.y - v_old.y)**2 +
                    (v_vec.z - v_old.z)**2
                )
                self.max_ditch_3d_dv[vehicle.id] = max(self.max_ditch_3d_dv.get(vehicle.id, 0.0), dv_3d)

            # 3. Режим 4: Прямолинейное качение по траве/грунту откоса (|V_lat| <= 2.0 м/с)
            if abs(v_lat) <= 2.0:
                target_state = 'ditch_rolling'
                if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                    phys = vehicle.get_physics_control()
                    for w in phys.wheels:
                        w.tire_friction = 1.25
                        w.lat_stiff_value = 10.0
                        w.lat_stiff_max_load = 1.6
                    safe_apply_physics_control(vehicle, phys)
                    self.vehicle_terramech_state[vehicle.id] = target_state

                # Вязкое сопротивление грунта качению (soil_mu = 0.22)
                drag_scale = -0.22 * mass * 9.81 / max(v_total, 0.1)
                f_drag = carla.Vector3D(x=v_vec.x * drag_scale, y=v_vec.y * drag_scale, z=0.0)
                vehicle.add_force(f_drag)

            # 4. Режим 5: Боковое зарывание и спотыкание (Soil Furrowing & Tripping, |V_lat| > 2.0 м/с)
            else:
                self.wheel_furrowing_triggered = True

                if v_lat > 2.0:
                    # Занос правым бортом вперед: правые колеса (1, 3) зарываются, левые (0, 2) разгружаются
                    target_state = 'furrow_right'
                    if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                        phys = vehicle.get_physics_control()
                        phys.wheels[1].tire_friction = 3.20
                        phys.wheels[1].lat_stiff_value = 40.0
                        phys.wheels[1].lat_stiff_max_load = 2.0
                        phys.wheels[3].tire_friction = 3.20
                        phys.wheels[3].lat_stiff_value = 40.0
                        phys.wheels[3].lat_stiff_max_load = 2.0
                        phys.wheels[0].tire_friction = 0.35
                        phys.wheels[0].lat_stiff_value = 10.0
                        phys.wheels[0].lat_stiff_max_load = 1.5
                        phys.wheels[2].tire_friction = 0.35
                        phys.wheels[2].lat_stiff_value = 10.0
                        phys.wheels[2].lat_stiff_max_load = 1.5
                        safe_apply_physics_control(vehicle, phys)
                        self.vehicle_terramech_state[vehicle.id] = target_state
                else:
                    # Занос левым бортом вперед: левые колеса (0, 2) зарываются, правые (1, 3) разгружаются
                    target_state = 'furrow_left'
                    if self.vehicle_terramech_state.get(vehicle.id) != target_state:
                        phys = vehicle.get_physics_control()
                        phys.wheels[0].tire_friction = 3.20
                        phys.wheels[0].lat_stiff_value = 40.0
                        phys.wheels[0].lat_stiff_max_load = 2.0
                        phys.wheels[2].tire_friction = 3.20
                        phys.wheels[2].lat_stiff_value = 40.0
                        phys.wheels[2].lat_stiff_max_load = 2.0
                        phys.wheels[1].tire_friction = 0.35
                        phys.wheels[1].lat_stiff_value = 10.0
                        phys.wheels[1].lat_stiff_max_load = 1.5
                        phys.wheels[3].tire_friction = 0.35
                        phys.wheels[3].lat_stiff_value = 10.0
                        phys.wheels[3].lat_stiff_max_load = 1.5
                        safe_apply_physics_control(vehicle, phys)
                        self.vehicle_terramech_state[vehicle.id] = target_state

                # Коэффициент глубины пропахивания борозды
                k_zone = 1.4 if (loc.z <= -0.70 or loc.y >= 13.0) else 1.0
                k_furrow = min(1.0, (abs(v_lat) - 2.0) / 4.0) * k_zone

                # Добавочная боковая сила среза грунтового валика (снижена до 0.35g во избежание двойного учета)
                f_furrow_mag = k_furrow * 0.35 * mass * 9.81 * sin_beta
                sign_lat = 1.0 if v_lat > 0 else -1.0
                f_furrow_vec = carla.Vector3D(
                    x=-sign_lat * right_vec.x * f_furrow_mag,
                    y=-sign_lat * right_vec.y * f_furrow_mag,
                    z=0.0
                )
                vehicle.add_force(f_furrow_vec)

                # Продольная сила торможения пахотой (снижена до 0.25g)
                f_plow_mag = k_furrow * 0.25 * mass * 9.81
                f_plow_vec = carla.Vector3D(
                    x=-v_vec.x / max(v_total, 0.1) * f_plow_mag,
                    y=-v_vec.y / max(v_total, 0.1) * f_plow_mag,
                    z=0.0
                )
                vehicle.add_force(f_plow_vec)

                # Сопротивление подъему на обратный откос кювета (Y >= 15.5 м)
                if loc.y >= 15.5:
                    y_scale = 1.20 if loc.y >= 20.0 else 0.35
                    f_brake_x = -v_vec.x * mass * 0.5 if loc.y >= 20.0 else 0.0
                    vehicle.add_force(carla.Vector3D(x=f_brake_x, y=-y_scale * mass * 9.81, z=0.0))

                # Реалистичный опрокидывающий момент спотыкания (Soil Tripping Torque)
                if abs(v_lat) > 3.2:
                    norm_roll = normalize_deg(rot.roll)
                    if abs(norm_roll) < 85.0:
                        lever = h_cg + 0.15
                        torque_mag = f_furrow_mag * lever * 3500.0
                        torque_vec = carla.Vector3D(
                            x=-sign_lat * fw_vec.x * torque_mag,
                            y=-sign_lat * fw_vec.y * torque_mag,
                            z=0.0
                        )
                        vehicle.add_torque(torque_vec)

                    # Вертикальный подброс кузова при ударе колес о перелом дна кювета
                    if loc.z <= -0.5 or loc.y >= 12.5:
                        vehicle.add_force(carla.Vector3D(x=0.0, y=0.0, z=0.25 * mass * 9.81))

    # --------------------------------------------------------------------------
    # Основной цикл симуляции (on_tick)
    # --------------------------------------------------------------------------
    def on_tick(self, snapshot):
        if not self.is_running:
            return
        try:
            # ------------------------------------------------------------------
            # Инициализация и конфигурация актеров при старте
            # ------------------------------------------------------------------
            if not self.configured:
                vehicles = self.world.get_actors().filter('vehicle.*')
                for v in vehicles:
                    type_str = v.type_id.lower()
                    if 'lincoln' in type_str:
                        self.ego_vehicle = v
                    elif 'audi' in type_str:
                        self.target_vehicle = v

                if self.ego_vehicle is not None and self.target_vehicle is not None:
                    ego_loc = self.ego_vehicle.get_transform().location
                    target_loc = self.target_vehicle.get_transform().location

                    carla_map = self.world.get_map()
                    ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                    self.start_lane_id = ego_wp.lane_id

                    ego_fw = self.ego_vehicle.get_transform().get_forward_vector()
                    disp = target_loc - ego_loc
                    self.initial_lon_offset = (disp.x * ego_fw.x) + (disp.y * ego_fw.y)

                    target_wp = carla_map.get_waypoint(target_loc, project_to_road=True)
                    right_vec = target_wp.transform.get_right_vector()
                    disp_lat = target_loc - target_wp.transform.location
                    lateral_offset = (disp_lat.x * right_vec.x) + (disp_lat.y * right_vec.y)

                    # Настройка Traffic Manager для обоих ТС
                    self.ego_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.ego_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.ego_vehicle, 100)
                    self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    self.target_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.target_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.target_vehicle, 100)
                    self.tm.vehicle_lane_offset(self.target_vehicle, float(lateral_offset))

                    # Применение кастомных масс в PhysX
                    phys_ego = self.ego_vehicle.get_physics_control()
                    phys_ego.mass = EGO_MASS
                    safe_apply_physics_control(self.ego_vehicle, phys_ego)

                    phys_target = self.target_vehicle.get_physics_control()
                    phys_target.mass = TARGET_MASS
                    safe_apply_physics_control(self.target_vehicle, phys_target)

                    # Инициализация ПИД-контроллера для Ego
                    if VehiclePIDController is not None:
                        args_lateral = {'K_P': 1.5, 'K_I': 0.05, 'K_D': 0.1, 'dt': 0.01}
                        args_long = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': 0.01}
                        self.pid_controller = VehiclePIDController(
                            self.ego_vehicle,
                            args_lateral=args_lateral,
                            args_longitudinal=args_long
                        )

                    # Датчики коллизий на ОБОИХ автомобилях (ISO 26262-3)
                    bp_sensor = self.world.get_blueprint_library().find('sensor.other.collision')

                    self.ego_collision_sensor = self.world.spawn_actor(
                        bp_sensor,
                        carla.Transform(),
                        attach_to=self.ego_vehicle
                    )
                    self.ego_collision_sensor.listen(lambda event: self.on_collision(event, source_actor='ego'))

                    self.target_collision_sensor = self.world.spawn_actor(
                        bp_sensor,
                        carla.Transform(),
                        attach_to=self.target_vehicle
                    )
                    self.target_collision_sensor.listen(lambda event: self.on_collision(event, source_actor='target'))

                    self.configured = True
                    try:
                        self.spectator = self.world.get_spectator()
                    except Exception:
                        self.spectator = None
                    print(f"[{snapshot.frame}] Актеры сконфигурированы: "
                          f"Ego ({self.ego_vehicle.type_id}, {EGO_MASS} кг, Lane {self.start_lane_id}) | "
                          f"Target ({self.target_vehicle.type_id}, {TARGET_MASS} кг) | "
                          f"Подкласс: {self.subclass} | "
                          f"Пост-шок режим: {self.driver_post_shock}")

            # ------------------------------------------------------------------
            # Основной цикл после конфигурации
            # ------------------------------------------------------------------
            else:
                ego_snap = snapshot.find(self.ego_vehicle.id)
                target_snap = snapshot.find(self.target_vehicle.id)
                if not ego_snap or not target_snap:
                    return

                ego_v = ego_snap.get_velocity()
                target_v = target_snap.get_velocity()
                ego_mps = math.hypot(ego_v.x, ego_v.y)
                target_mps = math.hypot(target_v.x, target_v.y)

                ego_loc = ego_snap.get_transform().location
                target_loc = target_snap.get_transform().location
                ego_fw = ego_snap.get_transform().get_forward_vector()
                ego_right = ego_snap.get_transform().get_right_vector()

                target_fw = target_snap.get_transform().get_forward_vector()
                target_right = target_snap.get_transform().get_right_vector()

                carla_map = self.world.get_map()
                current_ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                lane_forward = current_ego_wp.transform.get_forward_vector()
                lane_right = current_ego_wp.transform.get_right_vector()

                # Сохраняем кинематическую историю за 1 кадр до удара для обоих ТС
                self.speed_history[snapshot.frame] = {
                    'ego_v': ego_v,
                    'ego_forward': ego_fw,
                    'ego_right': ego_right,
                    'target_v': target_v,
                    'target_forward': target_fw,
                    'target_right': target_right,
                    'lane_forward': lane_forward,
                    'lane_right': lane_right
                }

                # Ограничиваем размер истории для экономии памяти
                if len(self.speed_history) > 200:
                    old_keys = sorted(self.speed_history.keys())[:-100]
                    for k in old_keys:
                        del self.speed_history[k]

                # Камера зрителя следует за Ego (без блокирующих RPC)
                if self.spectator is not None:
                    try:
                        cam_loc = ego_loc - (ego_fw * 12.0) + carla.Location(z=4.5)
                        cam_rot = ego_snap.get_transform().rotation
                        cam_rot.pitch -= 14.0
                        self.spectator.set_transform(carla.Transform(cam_loc, cam_rot))
                    except Exception:
                        pass

                self.tick_count += 1

                # --------------------------------------------------------------
                # БЛОК 4: 6-режимная террамеханика для ОБОИХ ТС (Ego и Target)
                # --------------------------------------------------------------
                for veh, mass, h_cg, actor_name in [
                    (self.ego_vehicle, EGO_MASS, EGO_H_CG, 'ego'),
                    (self.target_vehicle, TARGET_MASS, TARGET_H_CG, 'target')
                ]:
                    if veh is not None:
                        self.update_vehicle_surface_physics(veh, mass, h_cg, actor_name)

                # Управление Target при вытеснении на обочину/в кювет (Y >= 7.50 м)
                if target_loc.y >= 7.50:
                    try:
                        self.target_vehicle.set_autopilot(False)
                        target_brake = 0.0 if self.driver_post_shock == 'passive' else (
                            0.20 if self.driver_post_shock == 'imperfect' else 1.0
                        )
                        self.target_vehicle.apply_control(carla.VehicleControl(
                            throttle=0.0,
                            steer=0.0,
                            brake=float(target_brake),
                            hand_brake=False,
                            manual_gear_shift=False
                        ))
                    except Exception:
                        pass

                # --------------------------------------------------------------
                # ФАЗА 1: Разгон и удержание дистанции в слепой зоне
                # --------------------------------------------------------------
                if not self.lane_change_triggered:
                    disp = target_loc - ego_loc
                    current_lon_offset = (disp.x * ego_fw.x) + (disp.y * ego_fw.y)
                    error_x = current_lon_offset - self.initial_lon_offset

                    if self.tick_count % 100 == 0:
                        print(f"[{self.tick_count/100:4.1f}c] Ego: {ego_mps*3.6:5.1f} км/ч | "
                              f"Target: {target_mps*3.6:5.1f} км/ч | Отклонение: {error_x:+.2f} м | "
                              f"Окно стабильности: {self.stability_counter}/100")

                    d_error = target_mps - ego_mps
                    self.target_err_integral += error_x * 0.01
                    self.target_err_integral = max(-10.0, min(self.target_err_integral, 10.0))

                    p_gain = 0.8
                    i_gain = 0.4
                    d_gain = 0.5

                    correction = -(error_x * p_gain + self.target_err_integral * i_gain + d_error * d_gain)
                    correction = max(-5.0, min(correction, 5.0))
                    target_desired_mps = max(5.0, min(ego_mps + correction + 0.4, 40.0))
                    self.tm.set_desired_speed(self.target_vehicle, float(target_desired_mps * 3.6))

                    if self.tick_count % 20 == 0 and ego_mps > self.target_v0_mps * 0.5:
                        speed_err = self.target_v0_mps - ego_mps
                        if abs(speed_err) > 0.1:
                            self.ego_tm_speed += speed_err * 0.5
                            self.ego_tm_speed = max(10.0, min(self.ego_tm_speed, 35.0))
                            self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    is_speed_ready = ego_mps >= (CRUISE_SPEED_MPS - 0.15)
                    is_speed_sync = abs(ego_mps - target_mps) <= 0.15
                    is_dist_sync = abs(error_x) <= 0.15

                    # Проверка условий стабильности (100 тиков / 1 сек)
                    if is_speed_ready and is_speed_sync and is_dist_sync:
                        self.stability_counter += 1
                    else:
                        self.stability_counter = max(0, self.stability_counter - 2)

                    # Запасной триггер после 8 секунд, если машины уже на крейсерской скорости
                    forced_trigger = (self.tick_count >= 800 and ego_mps >= (CRUISE_SPEED_MPS - 0.5) and abs(error_x) <= 0.35)

                    if self.stability_counter >= 100 or forced_trigger:
                        self.lane_change_triggered = True
                        self.ego_vehicle.set_autopilot(False)
                        self.tm.set_desired_speed(self.target_vehicle, float(target_mps * 3.6))
                        print(f"\n[{snapshot.frame}] >>> МАНЕВР ИНИЦИИРОВАН ({self.subclass}): "
                              f"Ego начинает перестроение в соседнюю полосу <<<")

                # --------------------------------------------------------------
                # ФАЗА 2: Перестроение при отказе LCC / Фаза шока и пост-шок водителя
                # --------------------------------------------------------------
                else:
                    if not self.lost_control:
                        # До первого удара: ведение автомобиля Ego ПИД-контроллером в целевую полосу
                        ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                        if ego_wp.lane_id == self.start_lane_id:
                            if self.subclass == SUBCLASS_A:
                                target_lane_wp = ego_wp.get_left_lane()
                            else:
                                target_lane_wp = ego_wp.get_right_lane()
                        else:
                            target_lane_wp = ego_wp

                        if target_lane_wp is not None:
                            target_wps = target_lane_wp.next(15.0)
                            target_wp = target_wps[0] if target_wps else target_lane_wp
                            if self.pid_controller is not None:
                                control = self.pid_controller.run_step(self.target_v0_mps * 3.6, target_wp)
                                self.last_steer = control.steer
                                self.ego_vehicle.apply_control(control)

                    else:
                        # ПОСЛЕ УДАРА: когнитивная модель водителя SAE J2980
                        ticks_since_impact = self.tick_count - self.collision_tick

                        # ------------------------------------------------------
                        # 2.1. Фаза акустико-вестибулярного шока (t < T_shock)
                        # ------------------------------------------------------
                        if ticks_since_impact < self.shock_duration_ticks:
                            if ticks_since_impact <= 25:
                                # Первые 25 тиков (0.25 с): тонический мышечный спазм рук на руле
                                self.current_steer = self.impact_steer
                            else:
                                # С 26-го тика до T_shock: пассивная релаксация руля кастором (*= 0.95)
                                self.current_steer *= 0.95

                            # Педали полностью сброшены (свободный выбег coasting, БЕЗ торможения)
                            self.ego_vehicle.apply_control(carla.VehicleControl(
                                throttle=0.0,
                                steer=float(self.current_steer),
                                brake=0.0,
                                hand_brake=False,
                                manual_gear_shift=False
                            ))

                        # ------------------------------------------------------
                        # 2.2. После окончания фазы шока (t >= T_shock): ветвление по driver_post_shock
                        # ------------------------------------------------------
                        else:
                            # РЕЖИМ 1: 'passive' (По умолчанию — чистая оценка тяжести Severity S)
                            if self.driver_post_shock == 'passive':
                                # Водитель дезориентирован/отключен: педали отпущены, руль релаксирует кастором
                                self.current_steer *= 0.95
                                self.ego_vehicle.apply_control(carla.VehicleControl(
                                    throttle=0.0,
                                    steer=float(self.current_steer),
                                    brake=0.0,
                                    hand_brake=False,
                                    manual_gear_shift=False
                                ))

                            # РЕЖИМ 2: 'imperfect' (Несовершенная реакция в состоянии паники по SAE J2980)
                            elif self.driver_post_shock == 'imperfect':
                                target_road_y = 5.625 if self.subclass == SUBCLASS_C else 1.875
                                lateral_err = ego_loc.y - target_road_y

                                ego_rot = ego_snap.get_transform().rotation
                                yaw_deg = normalize_deg(ego_rot.yaw)
                                yaw_rad = math.radians(yaw_deg)
                                v_lat_lane = ego_v.y

                                raw_steer = -(0.30 * lateral_err + 0.10 * v_lat_lane + 0.35 * yaw_rad)
                                desired_steer = max(-0.35, min(0.35, raw_steer))

                                steer_delta = desired_steer - self.current_steer
                                steer_step = max(-0.04, min(0.04, steer_delta))
                                self.current_steer += steer_step

                                # Умеренное торможение без полной блокировки колес
                                brake_cmd = 0.35 if ego_loc.y < 7.50 else 0.20
                                self.ego_vehicle.apply_control(carla.VehicleControl(
                                    throttle=0.0,
                                    steer=float(self.current_steer),
                                    brake=float(brake_cmd),
                                    hand_brake=False,
                                    manual_gear_shift=False
                                ))

                            # РЕЖИМ 3: 'active' (Активное парирование заноса для исследований Controllability C)
                            else:
                                target_road_y = 5.625 if self.subclass == SUBCLASS_C else 1.875
                                lateral_err = ego_loc.y - target_road_y

                                ego_rot = ego_snap.get_transform().rotation
                                yaw_deg = normalize_deg(ego_rot.yaw)
                                yaw_rad = math.radians(yaw_deg)
                                v_lat_lane = ego_v.y

                                desired_steer = -(0.40 * lateral_err + 0.15 * v_lat_lane + 0.50 * yaw_rad)
                                desired_steer = max(-1.0, min(1.0, desired_steer))

                                steer_delta = desired_steer - self.current_steer
                                steer_step = max(-0.08, min(0.08, steer_delta))
                                self.current_steer += steer_step

                                is_danger_zone = (ego_loc.y >= 6.50) or (ego_loc.y <= 0.50) or (abs(lateral_err) > 1.2)
                                brake_cmd = 1.0 if is_danger_zone else 0.7
                                self.ego_vehicle.apply_control(carla.VehicleControl(
                                    throttle=0.0,
                                    steer=float(self.current_steer),
                                    brake=float(brake_cmd),
                                    hand_brake=False,
                                    manual_gear_shift=False
                                ))

                    # Периодический мониторинг маневра в консоль
                    if self.tick_count % 50 == 0:
                        cur_steer = self.current_steer if self.lost_control else self.last_steer
                        print(f"[{self.tick_count/100:4.1f}c] МАНЕВР: Ego Y={ego_loc.y:5.2f} (v={ego_mps*3.6:5.1f} км/ч) "
                              f"| Target Y={target_loc.y:5.2f} (v={target_mps*3.6:5.1f} км/ч) | "
                              f"dY={target_loc.y - ego_loc.y:+5.2f} | Руль Ego: {cur_steer:+.3f} | Удар: {self.lost_control}")

        except Exception:
            traceback.print_exc()

    # --------------------------------------------------------------------------
    # Расчет исхода и тяжести в кювете (SAE J2980 Appendix B)
    # --------------------------------------------------------------------------
    def compute_ditch_metrics(self):
        """
        Возвращает оценку тяжести вылета в кювет по SAE J2980 Appendix B:
        (sev_ditch_val, ditch_actor_str, departure_speed, max_lateral_slip,
         wheel_furrowing_triggered, max_tilt, ditch_3d_dv)
        """
        if not self.departed_actors:
            return 0, 'None', '', '', False, '', ''

        actor_str = 'both' if len(self.departed_actors) > 1 else list(self.departed_actors)[0]
        dep_speed = max(self.departure_speeds.values()) if self.departure_speeds else 0.0
        max_tilt = max(self.max_tilt_deg.values()) if self.max_tilt_deg else 0.0
        max_dv = max(self.max_ditch_3d_dv.values()) if self.max_ditch_3d_dv else 0.0
        max_lat = max(self.max_lateral_slip.values()) if self.max_lateral_slip else 0.0

        # Классификация тяжести в кювете по ISO 26262-3 / SAE J2980:
        # S3: Опрокидывание (>= 60 град) при скорости > 15 м/с ИЛИ удар dv_3d > 12 м/с
        if (max_tilt >= 60.0 and dep_speed > 15.0) or max_dv > 12.0:
            sev = 3
        # S2: Опрокидывание на малой скорости (<= 15 м/с) ИЛИ удар dv_3d > 6 м/с
        elif (max_tilt >= 60.0 and dep_speed <= 15.0) or max_dv > 6.0:
            sev = 2
        # S1: Сильный крен (25 - 60 град) без переворота ИЛИ удар dv_3d > 2 м/с
        elif (max_tilt >= 25.0) or max_dv > 2.0:
            sev = 1
        # S0: Плавный съезд без переворота и жестких ударов
        else:
            sev = 0

        return sev, actor_str, dep_speed, max_lat, self.wheel_furrowing_triggered, max_tilt, max_dv

    def get_barrier_collision_actor(self) -> str:
        """Возвращает строку участника, ударившего отбойник ('ego', 'target', 'both', или '')."""
        if 'ego' in self.barrier_departed_actors and 'target' in self.barrier_departed_actors:
            return 'both'
        elif 'ego' in self.barrier_departed_actors:
            return 'ego'
        elif 'target' in self.barrier_departed_actors:
            return 'target'
        else:
            return ''

    def start(self):
        self.is_running = True
        self.callback_id = self.world.on_tick(self.on_tick)

    def stop(self):
        self.is_running = False
        self.spectator = None
        if self.world is not None and self.callback_id is not None:
            try:
                self.world.remove_on_tick(self.callback_id)
            except Exception:
                pass
            self.callback_id = None
        if self.ego_collision_sensor is not None:
            try:
                if self.ego_collision_sensor.is_alive:
                    self.ego_collision_sensor.destroy()
            except Exception:
                pass
            self.ego_collision_sensor = None
        if self.target_collision_sensor is not None:
            try:
                if self.target_collision_sensor.is_alive:
                    self.target_collision_sensor.destroy()
            except Exception:
                pass
            self.target_collision_sensor = None
        if self.tm is not None:
            try:
                self.tm.set_synchronous_mode(False)
            except Exception:
                pass


# ==============================================================================
# MAIN SIMULATION CAMPAIGN ORCHESTRATOR
# ==============================================================================
def main():
    """
    Главный оркестратор кампании моделирования HARA:
    - Подключается к CARLA и инициализирует мир с картой 'map_ditch'
    - Разрешает пути к ассетам OpenDRIVE (map_ditch.xodr)
    - Компилирует сценарии Scenic для Подклассов А (отбойник) и С (кювет)
    - Формирует план прогонов со стохастической когнитивной моделью водителя
    - Проводит симуляции с сохранением всех 20 метрик в CSV (UTF-8-SIG)
    """
    parser = argparse.ArgumentParser(description="HARA LCC Blind Spot Simulation Campaign")
    parser.add_argument('--subclass', type=str, default='ALL', choices=['A', 'C', 'ALL'],
                        help="Road subclass: 'A' (Barrier), 'C' (Ditch), or 'ALL' (Campaign)")
    parser.add_argument('--driver-post-shock', type=str, default='passive',
                        choices=['passive', 'imperfect', 'active'],
                        help="Driver behavior mode after shock phase: 'passive' (default, coasting/engine braking for pure Severity S), 'imperfect' (limited panic recovery), or 'active' (full counter-steering & braking for Controllability C)")
    parser.add_argument('--mode', type=str, default='equal', choices=['equal', 'sequential', 'weighted'],
                        help="Campaign mode: 'equal' (50/50: N/2 of A, N/2 of C), 'sequential' (0.2 A / 0.8 C), or 'weighted'")
    parser.add_argument('--runs', type=int, default=20, help="Total number of simulation runs (default: 20)")
    parser.add_argument('--map-dir', type=str, default=None,
                        help="Path to directory containing map assets (default: auto-detected)")
    parser.add_argument('--clear', action='store_true', help="Clear/overwrite previous CSV results before running")
    parser.add_argument('--render', action='store_true', help="Enable Carla rendering window")
    args = parser.parse_args()

    print("=" * 80)
    print(" HARA SIMULATION CAMPAIGN: LANE CHANGE COLLISION (LCC) BLIND SPOT FAILURE")
    print(" ISO 26262-3 / SAE J2980 Appendix B & E (Dual-Sensor & 6-Mode Terramechanics)")
    print(f" Post-Shock Driver Mode: '{args.driver_post_shock}'")
    print("=" * 80)

    # 1. Подключение к симулятору CARLA
    print("\n[1/4] Подключение к CARLA (localhost:2000)...")
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        print(f"  [OK] Успешное подключение к CARLA. Текущая карта: {world.get_map().name}")
    except Exception as e:
        print(f"  [ОШИБКА] Не удалось подключиться к CARLA: {e}")
        sys.exit(1)

    # Разрешение путей к карте OpenDRIVE
    maps_assets_dir, map_xodr_path = resolve_map_assets(args.map_dir)
    print(f"  [КАРТА] Директория ассетов: {maps_assets_dir}")
    print(f"  [КАРТА] Файл OpenDRIVE: {map_xodr_path}")

    # 2. Инициализация CarlaSimulator Scenic
    print(f"\n[2/4] Инициализация CarlaSimulator с картой '{CARLA_MAP_NAME}'...")
    try:
        simulator = CarlaSimulator(
            carla_map=CARLA_MAP_NAME,
            map_path=map_xodr_path,
            timestep=0.01,
            render=args.render
        )
        print("  [OK] CarlaSimulator успешно инициализирован.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой инициализации CarlaSimulator: {e}")
        sys.exit(1)

    # 3. Компиляция сценариев Scenic для обоих подклассов
    print(f"\n[3/4] Компиляция Scenic сценария ({SCENIC_PATH})...")
    scenarios = {}
    try:
        scenarios[SUBCLASS_A] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_A, 'map': map_xodr_path}
        )
        scenarios[SUBCLASS_C] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_C, 'map': map_xodr_path}
        )
        print("  [OK] Сценарии для Подкласса А (Отбойник) и Подкласса С (Кювет) скомпилированы.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой компиляции Scenic: {e}")
        sys.exit(1)

    # 4. Формирование плана кампании
    campaign_plan = []
    if args.subclass == 'A':
        campaign_plan = [SUBCLASS_A] * args.runs
    elif args.subclass == 'C':
        campaign_plan = [SUBCLASS_C] * args.runs
    else:
        if args.mode == 'equal':
            # Равное распределение: половина для Подкласса А, половина для Подкласса С
            runs_a = args.runs // 2
            runs_c = args.runs - runs_a
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_C] * runs_c
        elif args.mode == 'sequential':
            # Экспозиция по дорожному профилю: 20% отбойник (А) и 80% кювет (С)
            runs_a = max(1, int(round(args.runs * 0.20)))
            runs_c = max(1, args.runs - runs_a)
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_C] * runs_c
        else:
            campaign_plan = random.choices([SUBCLASS_A, SUBCLASS_C], weights=[0.20, 0.80], k=args.runs)

    print(f"\n[4/4] Запуск кампании: всего {len(campaign_plan)} прогонов "
          f"({campaign_plan.count(SUBCLASS_A)} Подкласс А, {campaign_plan.count(SUBCLASS_C)} Подкласс С)")

    # Подготовка CSV-файла с обновленной структурой (20 колонок)
    csv_fieldnames = [
        'run_id', 'subclass', 'driver_post_shock_mode', 'shock_duration_sec', 'side_collision',
        'delta_v_ego_mps', 'delta_v_target_mps', 'effective_side_dv_mps',
        'barrier_collision', 'barrier_collision_actor', 'barrier_impact_type', 'barrier_impact_mps',
        'ditch_departure_actor', 'departure_speed_mps', 'max_lateral_slip_mps',
        'wheel_furrowing_triggered', 'max_roll_pitch_deg', 'ditch_3d_dv_mps',
        'severity', 'scenario_outcome'
    ]

    if args.clear and os.path.isfile(CSV_PATH):
        try:
            os.remove(CSV_PATH)
        except Exception:
            pass

    file_exists = os.path.isfile(CSV_PATH)

    # Проверка совместимости формата колонок в существующем CSV-файле
    if file_exists and not args.clear:
        try:
            with open(CSV_PATH, 'r', encoding='utf-8-sig') as f_check:
                header_line = f_check.readline()
                if 'barrier_collision_actor' not in header_line:
                    # Устаревший формат колонок: создаем резервную копию и перезаписываем заголовок
                    backup_csv = os.path.join(RESULTS_DIR, "lcc_hara_results_legacy_backup.csv")
                    os.replace(CSV_PATH, backup_csv)
                    print(f"  [ИНФО] Старый CSV сохранен в резервную копию: {backup_csv}")
                    file_exists = False
        except Exception:
            pass

    with open(CSV_PATH, mode='w' if args.clear or not file_exists else 'a', newline='', encoding='utf-8-sig') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=csv_fieldnames)
        if not file_exists or args.clear:
            writer.writeheader()

        # ----------------------------------------------------------------------
        # Цикл прогонов HARA кампании
        # ----------------------------------------------------------------------
        for run_idx, subclass in enumerate(campaign_plan, start=1):
            subclass_label = "Подкласс А (Отбойник)" if subclass == SUBCLASS_A else "Подкласс С (Кювет)"
            print("\n" + "=" * 80)
            print(f" ПРОГОН [{run_idx}/{len(campaign_plan)}] -> {subclass_label}")
            print(f" Режим водителя после шока: {args.driver_post_shock}")
            print("=" * 80)

            # Генерация сцены
            scenario = scenarios[subclass]
            scene, _ = scenario.generate()

            # Создание и запуск менеджера автопилота с выбранным режимом поведения водителя
            manager = AutopilotManager(
                client,
                world,
                subclass=subclass,
                driver_post_shock=args.driver_post_shock
            )
            manager.start()

            try:
                # Запуск симуляции Scenic (таймаут 30 секунд / 3000 тиков)
                simulator.simulate(scene, maxSteps=3000)
            except Exception as e:
                print(f"[ПРЕДУПРЕЖДЕНИЕ] Симуляция завершена или прервана: {e}")
            finally:
                # Остановка сенсоров и колбэков тика
                manager.stop()
                time.sleep(0.3)

                # --------------------------------------------------------------
                # БЛОК 5: Каскадная агрегация метрик тяжести по ISO 26262-3
                # --------------------------------------------------------------
                has_side_hit = manager.side_collision_frame is not None
                has_barrier_hit = manager.barrier_collision_frame is not None
                barrier_actor = manager.get_barrier_collision_actor()

                # Метрики кювета
                (sev_ditch_val, ditch_actor, dep_speed, max_lat,
                 furrow_trig, max_tilt, ditch_dv) = manager.compute_ditch_metrics()

                # Определение максимальной тяжести по всем факторам
                final_sev_val = max(manager.sev_side_val, manager.sev_barrier_val, sev_ditch_val)
                severity = f"S{final_sev_val}"

                # Классификация исхода сценария на русском языке
                outcome = "Без столкновений"
                if subclass == SUBCLASS_A:
                    b_type = manager.barrier_impact_type
                    actor_desc = f" ({barrier_actor})" if barrier_actor else ""
                    if has_side_hit and has_barrier_hit:
                        outcome = f"Боковой удар с целевым ТС + {b_type} удар в отбойник{actor_desc}"
                    elif has_barrier_hit:
                        outcome = f"{b_type} удар в отбойник{actor_desc} (без удара целевого ТС)"
                    elif has_side_hit:
                        outcome = "Боковой удар с целевым ТС (Без удара в отбойник)"
                else:  # SUBCLASS_C
                    prefix = "Боковой удар с целевым ТС + " if has_side_hit else ""
                    if ditch_actor != 'None':
                        if max_tilt >= 60.0:
                            outcome = f"{prefix}Вылет в кювет ({ditch_actor}) с опрокидыванием ({severity})"
                        elif furrow_trig:
                            outcome = f"{prefix}Вылет в кювет ({ditch_actor}) с зарыванием в грунт ({severity})"
                        else:
                            outcome = f"{prefix}Съезд в кювет ({ditch_actor}) без опрокидывания ({severity})"
                    elif has_side_hit:
                        outcome = "Боковой удар с целевым ТС (Без съезда в кювет)"

                # Формирование строки для CSV
                row_data = {
                    'run_id': run_idx,
                    'subclass': subclass,
                    'driver_post_shock_mode': manager.driver_post_shock,
                    'shock_duration_sec': round(manager.shock_duration_sec, 3),
                    'side_collision': has_side_hit,
                    'delta_v_ego_mps': round(manager.delta_v_ego, 2) if has_side_hit else '',
                    'delta_v_target_mps': round(manager.delta_v_target, 2) if has_side_hit else '',
                    'effective_side_dv_mps': round(manager.effective_side_delta_v, 2) if has_side_hit else '',
                    'barrier_collision': has_barrier_hit,
                    'barrier_collision_actor': barrier_actor,
                    'barrier_impact_type': manager.barrier_impact_type if has_barrier_hit else '',
                    'barrier_impact_mps': round(manager.barrier_impact_speed, 2) if has_barrier_hit else '',
                    'ditch_departure_actor': ditch_actor,
                    'departure_speed_mps': round(dep_speed, 2) if ditch_actor != 'None' else '',
                    'max_lateral_slip_mps': round(max_lat, 2) if ditch_actor != 'None' else '',
                    'wheel_furrowing_triggered': furrow_trig,
                    'max_roll_pitch_deg': round(max_tilt, 1) if ditch_actor != 'None' else '',
                    'ditch_3d_dv_mps': round(ditch_dv, 2) if ditch_actor != 'None' else '',
                    'severity': severity,
                    'scenario_outcome': outcome
                }

                writer.writerow(row_data)
                csv_file.flush()

                # Сводный отчет в консоль
                print("\n" + "-" * 80)
                print(f"ИТОГ ПРОГОНА [{run_idx}]:")
                print(f"  * Подкласс: {subclass}")
                print(f"  * Режим водителя после шока: {manager.driver_post_shock}")
                print(f"  * Время шока водителя: {manager.shock_duration_sec:.3f} с ({manager.shock_duration_ticks} тиков)")
                print(f"  * Удар ТС-ТС: {has_side_hit} " +
                      (f"(Delta-V Ego: {manager.delta_v_ego:.2f} м/с, Target: {manager.delta_v_target:.2f} м/с, S_side: S{manager.sev_side_val})" if has_side_hit else ""))
                if subclass == SUBCLASS_A:
                    print(f"  * Удар в отбойник: {has_barrier_hit} " +
                          (f"({manager.barrier_impact_type}, {manager.barrier_impact_speed:.2f} м/с, актер: {barrier_actor}, S_barrier: S{manager.sev_barrier_val})" if has_barrier_hit else ""))
                else:
                    print(f"  * Сход в кювет: {ditch_actor} " +
                          (f"(Скорость: {dep_speed:.2f} м/с, Крен: {max_tilt:.1f}°, 3D Delta-V: {ditch_dv:.2f} м/с, S_ditch: S{sev_ditch_val})" if ditch_actor != 'None' else ""))
                print(f"  * Итоговая тяжесть ISO 26262-3: >>> {severity} <<<")
                print(f"  * Исход: {outcome}")
                print("-" * 80)

                time.sleep(0.5)

    print("\n" + "=" * 80)
    print(f"КАМПАНИЯ ЗАВЕРШЕНА! Результаты сохранены в: {CSV_PATH}")
    print("=" * 80)


if __name__ == '__main__':
    main()