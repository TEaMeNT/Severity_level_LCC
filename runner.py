# ==============================================================================
# HARA SIMULATION RUNNER: Lane Change Collision (LCC) Blind Spot Failure
# Стандарты: ISO 26262-3:2018 (HARA) / SAE J2980 (Appendix B - Severity, Appendix E - Shock)
# Архитектура: Исполнитель одиночных физических симуляций CARLA + Scenic
# Методология: Двусторонний мониторинг (Ego + Target), Метод граничных условий на бровке кювета
# ==============================================================================
#
# КАРТА И СИСТЕМА КООРДИНАТ MAP_DITCH:
# - Продольная ось дороги: +X (направление на Восток, крейсерская скорость 100 км/ч = 27.78 м/с).
# - Поперечная ось дороги: +Y (вправо по ходу движения ТС).
# - Вертикальная ось: +Z (вверх, уровень проезжей части и обочины Z = 0.0 м).
# - Левая полоса (Lane -1): центр Y = +1.875 м. Слева (Y <= 0.0 м) жесткий бетонный отбойник New Jersey (h = 0.80 м).
# - Правая полоса (Lane -2): центр Y = +5.625 м. Справа (Y >= 7.50 м) обочина и водоотводный кювет.
#   * Обочина (Lane -3, Y in [7.50, 9.75) м): укрепленная грунтово-гравийная полоса (Z = 0.0 м).
#   * Внутренний откос (Lane -4, Y in [9.75, 14.55) м): откос кювета с уклоном 1:4 (~14°, спуск до Z = -1.20 м).
#   * Дно кювета (Lane -5, Y in [14.55, 16.05) м): плоское дно на отметке Z = -1.20 м.
#   * Внешний откос (Lane -6, Y in [16.05, 20.85] м): обратный подъем 1:4 до Z = 0.0 м.
#   * Придорожное поле (Lane -7, Y > 20.85 м): ровная поверхность (Z = 0.0 м).
#
# ПРИНЦИП ЕДИНОЙ ОТВЕТСТВЕННОСТИ:
# Скрипт runner.py отвечает ИСКЛЮЧИТЕЛЬНО за выполнение заездов в CARLA, съем
# кинематических параметров каждого прогона и сохранение строки в CSV.
# Логика статистической агрегации кампании (>10%) вынесена в analyze_results.py.
# ==============================================================================

import os
import sys
import time
import math
import random
import csv
import argparse
import traceback

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

DEFAULT_MAPS_DIR = os.path.join(BASE_DIR, "maps", "assets")
FALLBACK_MAP_DIRS = [
    DEFAULT_MAPS_DIR,
    os.path.join(BASE_DIR, "..", "maps", "assets"),
    r"C:\Users\teament\Desktop\xlam\Severity_level_LCC\maps\assets",
    r"C:\Users\teament\Desktop\xlam\work\maps\assets",
    BASE_DIR,
]


def resolve_map_assets(custom_dir: str = None) -> tuple[str, str]:
    """
    Разрешение пути к папке ассетов карты и файлу OpenDRIVE map_ditch.xodr.
    Поддерживает CLI-переопределение через --map-dir, относительные пути и fallback.
    """
    candidates = []
    if custom_dir:
        candidates.append(os.path.abspath(custom_dir))
    candidates.extend(FALLBACK_MAP_DIRS)

    for c_dir in candidates:
        if os.path.isdir(c_dir):
            xodr_ditch = os.path.join(c_dir, "map_ditch.xodr")
            if os.path.isfile(xodr_ditch):
                return c_dir, xodr_ditch
            xodr_generic = os.path.join(c_dir, "map.xodr")
            if os.path.isfile(xodr_generic):
                return c_dir, xodr_generic

    # Аварийный fallback на текущую рабочую директорию
    fallback_xodr = os.path.join(BASE_DIR, "map_ditch.xodr")
    if not os.path.isfile(fallback_xodr):
        fallback_xodr = os.path.join(BASE_DIR, "map.xodr")
    return BASE_DIR, fallback_xodr


# Физические параметры участников сценария (Строго по ТЗ проекта):
# Ego: Lincoln MKZ 2017 (масса 3490 кг как аналог тяжелого представительского седана Аурус Сенат)
EGO_MASS = 3490.0        # Масса ТС Ego (кг)
EGO_H_CG = 0.65          # Высота центра тяжести ТС Ego (м)

# Target: Audi TT (легкое спорткупе массой 2000 кг)
TARGET_MASS = 2000.0     # Масса целевого ТС (кг)
TARGET_H_CG = 0.52       # Высота центра тяжести целевого ТС (м)

# Коэффициент восстановления при пластической деформации кузовных панелей (SAE J2980)
RESTITUTION_E = 0.15

# Скоростной режим трассы (загородная дорога: ограничение 90 км/ч, крейсерская 100 км/ч)
SPEED_LIMIT_MPS = 90.0 / 3.6    # 25.0 м/с
CRUISE_SPEED_MPS = 100.0 / 3.6  # 27.78 м/с

# Идентификаторы дорожных подклассов (с весами в ODD)
SUBCLASS_A = "SUBCLASS_A_BARRIER"  # Маневр влево: риск удара о бетонный отбойник (W_A = 0.20)
SUBCLASS_C = "SUBCLASS_C_DITCH"    # Маневр вправо: риск схода и опрокидывания в кювете (W_C = 0.80)

# Точная схема 25 столбцов CSV-файла результатов HARA
CSV_FIELDNAMES = [
    'run_id',
    'subclass',
    'driver_post_shock_mode',
    'shock_duration_sec',
    'side_collision',
    'delta_v_ego_mps',
    'delta_v_target_mps',
    'effective_side_dv_mps',
    'sev_side',
    'barrier_collision',
    'barrier_collision_actor',
    'barrier_impact_type',
    'barrier_impact_mps',
    'sev_barrier',
    'ditch_departure_actor',
    'departure_speed_mps',
    'road_perp_speed_mps',
    'encroachment_angle_deg',
    'vehicle_lat_skid_mps',
    'sideslip_angle_deg',
    'departure_yaw_rate_deg_s',
    'ditch_mechanism',
    'sev_ditch',
    'severity',
    'scenario_outcome'
]


def normalize_deg(angle: float) -> float:
    """Нормализация углов Эйлера к диапазону [-180, 180] градусов."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


# ==============================================================================
# AUTOPILOT & SIMULATION MANAGER
# ==============================================================================
class AutopilotManager:
    """
    Менеджер симуляции HARA LCC Blind Spot Failure:
    - Фаза 0: Инициализация актеров, назначение масс PhysX, прикрепление датчиков коллизий к Ego и Target.
    - Фаза 1: Синхронный разгон до 100 км/ч и ПИД-удержание целевого ТС в слепой зоне.
    - Фаза 2: Инициация маневра перестроения, когнитивная модель водителя SAE J2980.
    - Двусторонний мониторинг ударов об отбойник (Ego + Target) по ISO 26262-3.
    - Метод граничных условий на бровке кювета (Y = 9.75 м) без искусственной террамеханики.
    """

    def __init__(self, client, world, subclass: str = SUBCLASS_C, driver_post_shock: str = 'passive', dt: float = 0.01):
        self.client = client
        self.world = world
        self.subclass = subclass
        self.driver_post_shock = driver_post_shock
        self.dt = float(dt)

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
        self.stability_time_sec = 0.0

        self.pid_controller = None
        self.start_lane_id = None

        # Датчики столкновений для обоих ТС (ISO 26262-3)
        self.ego_collision_sensor = None
        self.target_collision_sensor = None
        self.speed_history = {}

        # ----------------------------------------------------------------------
        # Когнитивная модель водителя (SAE J2980 Appendix E)
        # ----------------------------------------------------------------------
        # Время акустико-вестибулярного ступора: 0.65 - 1.05 с, медиана 0.85 с
        self.shock_duration_sec = min(1.05, max(0.65, random.gauss(0.85, 0.12)))
        self.shock_duration_ticks = max(1, int(round(self.shock_duration_sec / self.dt)))
        self.last_steer = 0.0
        self.impact_steer = 0.0
        self.current_steer = 0.0

        # ----------------------------------------------------------------------
        # Первичное столкновение ТС с ТС (SAE J2980 Table B1)
        # ----------------------------------------------------------------------
        self.side_collision_frame = None
        self.v_closing_lat = 0.0
        self.delta_v_ego = 0.0
        self.delta_v_target = 0.0
        self.effective_side_delta_v = 0.0
        self.sev_side_val = 0

        # ----------------------------------------------------------------------
        # Вторичный удар об отбойник (Подкласс А)
        # ----------------------------------------------------------------------
        self.barrier_collision_frame = None
        self.barrier_impact_speed = 0.0
        self.barrier_impact_type = ""
        self.sev_barrier_val = 0
        self.barrier_hit_actors = set()  # {'ego', 'target'}

        # ----------------------------------------------------------------------
        # Граничные условия вылета в кювет на бровке Y = 9.75 м (Подкласс С)
        # ----------------------------------------------------------------------
        self.ditch_metrics_by_actor = {}  # actor_name ('ego'/'target') -> dict of metrics
        self.sev_ditch_val = 0

    # --------------------------------------------------------------------------
    # Обработчик столкновений (Collision Sensor Callback для обоих ТС)
    # --------------------------------------------------------------------------
    def on_collision(self, event, source_actor: str = 'ego'):
        """
        Универсальный обработчик столкновений для Ego и Target по ISO 26262-3:
        - Обнаруживает первичное боковое столкновение ТС-ТС.
        - Обнаруживает вторичный удар о бетонный отбойник для Ego или Target.
        """
        other = event.other_actor
        frame = event.frame

        # Извлечение кинематики строго за 1 кадр до контакта (PhysX clean vectors)
        hist = self.speed_history.get(frame - 1)
        # Если frame - 1 почему-то недоступен, берем наиболее свежий доступный кадр до frame
        if not hist:
            hist = self.speed_history.get(frame - 2)
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
        # 1. Первичный боковой удар ТС с ТС (SAE J2980 Table B1)
        # ----------------------------------------------------------------------
        if 'vehicle' in other.type_id.lower():
            if self.side_collision_frame is None:
                self.side_collision_frame = frame

                # Поперечная скорость сближения строго за 1 кадр до контакта (frame - 1)
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

                # Калиброванная шкала SAE J2980 Table B1 (боковой удар)
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
            actor_veh = self.ego_vehicle if source_actor == 'ego' else self.target_vehicle
            if actor_veh is None:
                return
            try:
                loc_y = actor_veh.get_location().y
            except Exception:
                return

            # Отбойник расположен на Y = 0.0 м. Учитывая диагональное положение кузова, центр ТС при ударе может быть до Y = 3.2 м.
            # Отсекаем ложные срабатывания о правый кювет (Y >= 7.5 м).
            if loc_y > 3.2:
                return

            # Кинематика участника за 1 кадр до удара (frame - 1)
            if source_actor == 'ego':
                v_actor = ego_v
                fw_actor = ego_forward
            else:
                v_actor = target_v
                fw_actor = target_forward

            # Нормальная скорость впечатывания перпендикулярно плоскости отбойника
            v_barrier = abs((v_actor.x * lane_right.x) + (v_actor.y * lane_right.y))

            # Косинус угла атаки относительно продольной оси дороги
            cos_alpha = (fw_actor.x * lane_forward.x) + (fw_actor.y * lane_forward.y)

            if abs(cos_alpha) < 0.707:
                # Угол атаки > 45 град (Frontal/Rear: работают зоны деформации кузова)
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
                # Угол атаки <= 45 град (Side-swipe: боковой скользящий удар без зон смятия)
                cur_impact_type = 'Side-swipe'
                if v_barrier > 12.0:
                    cur_sev = 3
                elif v_barrier > 6.0:
                    cur_sev = 2
                elif v_barrier > 2.0:
                    cur_sev = 1
                else:
                    cur_sev = 0

            self.barrier_hit_actors.add(source_actor)

            # Сохранение первичного или наихудшего удара
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
                  f"S_barrier = S{cur_sev} (макс. по прогону: S{self.sev_barrier_val})")

            # Активация фазы шока для Ego при первичном ударе Ego об отбойник
            if not self.lost_control and source_actor == 'ego':
                self.lost_control = True
                self.collision_tick = self.tick_count
                self.impact_steer = self.last_steer
                self.current_steer = self.impact_steer

    # --------------------------------------------------------------------------
    # Метод граничных условий на бровке кювета Y = 9.75 м (Подкласс С)
    # --------------------------------------------------------------------------
    def check_ditch_boundary_conditions(self, actor_name: str, vehicle, lane_forward, lane_right, snapshot_frame: int):
        """
        Проверка пересечения бровки откоса кювета (Y >= 9.75 м) и фиксация граничных условий:
        - V_dep: полная горизонтальная скорость вылета (м/с).
        - V_perp_road: нормальная скорость ухода в кювет (м/с).
        - theta_enc: угол траектории вылета (градусы).
        - V_lat_car: поперечная скорость бокового скольжения колес (м/с).
        - beta: угол бокового заноса кузова (Sideslip Angle, градусы).
        - omega_z: угловая скорость рыскания кузова (Yaw Rate, град/с).
        - Классификация тяжести S0-S3 по научно обоснованной матрице (Cooperrider / NHTSA / AASHTO).
        """
        if vehicle is None or actor_name in self.ditch_metrics_by_actor:
            return

        loc = vehicle.get_location()

        # Фиксация происходит ровно в кадр пересечения бровки откоса кювета (Y >= 9.75 м)
        if loc.y >= 9.75:
            tf = vehicle.get_transform()
            vel = vehicle.get_velocity()
            ang_vel = vehicle.get_angular_velocity()  # В CARLA z возвращается в градусах/сек

            # 1. Полная горизонтальная скорость вылета (м/с)
            v_dep = math.hypot(vel.x, vel.y)

            # 2. Нормальная скорость ухода в кювет перпендикулярно оси дороги
            v_perp_road = abs(vel.x * lane_right.x + vel.y * lane_right.y)

            # 3. Угол траектории вылета относительно дороги (Encroachment Angle, град)
            sin_theta = min(1.0, v_perp_road / max(v_dep, 0.1))
            theta_enc = math.degrees(math.asin(sin_theta))

            # 4. Кинематические проекции в локальных осях кузова автомобиля
            fw_vec = tf.get_forward_vector()
            rt_vec = tf.get_right_vector()
            v_lon_car = abs(vel.x * fw_vec.x + vel.y * fw_vec.y)
            v_lat_car = abs(vel.x * rt_vec.x + vel.y * rt_vec.y)

            # 5. Угол бокового заноса кузова (Sideslip Angle beta, град)
            beta = math.degrees(math.atan2(v_lat_car, max(v_lon_car, 0.01)))

            # 6. Угловая скорость рыскания (Yaw Rate omega_z, град/с)
            omega_z = abs(ang_vel.z)

            # 7. Классификация механизма и тяжести по матрице граничных условий:
            # S3: Критические / фатальные травмы (MAIS 5-6 > 10%)
            # Cooperrider SAE 900366 / Digges NHTSA / SAE J2980 Table B1
            is_s3_skid = (v_dep > 18.0) and (v_lat_car > 5.5 or beta >= 25.0 or omega_z > 45.0)
            is_s3_nose = (v_perp_road > 12.0)

            # S2: Тяжелые травмы (MAIS 3-6 > 10%) - NCHRP Report 665 / Mak & Sicking
            is_s2_skid = (v_dep > 11.0) and (v_lat_car > 3.5 or beta >= 15.0 or omega_z > 25.0)
            is_s2_bottom = (v_perp_road > 6.0)

            # S1: Легкие травмы - AASHTO Roadside Design Guide / СП 34.13330.2021
            is_s1_tracking = (v_dep > 8.0 and v_perp_road > 2.0)
            is_s1_low_skid = (v_dep <= 11.0 and beta >= 15.0)

            if is_s3_skid or is_s3_nose:
                actor_sev = 3
                mechanism = 'High-Speed Soil-Trip Rollover' if is_s3_skid else 'Severe Ditch Nose-In Impact'
            elif is_s2_skid or is_s2_bottom:
                actor_sev = 2
                mechanism = 'Moderate Rollover Risk' if is_s2_skid else 'Hard Ditch Bottom Impact'
            elif is_s1_tracking or is_s1_low_skid:
                actor_sev = 1
                mechanism = 'Traversable Tracking Slope Departure'
            else:
                actor_sev = 0
                mechanism = 'Shallow Grazing Departure'

            self.ditch_metrics_by_actor[actor_name] = {
                'v_dep': v_dep,
                'v_perp_road': v_perp_road,
                'theta_enc': theta_enc,
                'v_lat_car': v_lat_car,
                'beta': beta,
                'omega_z': omega_z,
                'mechanism': mechanism,
                'actor_sev': actor_sev,
                'frame': snapshot_frame,
                'y_loc': loc.y
            }

            print(f"\n[{snapshot_frame}] ->> СХОД В КЮВЕТ ({actor_name.upper()} на бровке Y=9.75м): "
                  f"V_dep={v_dep*3.6:.1f} км/ч ({v_dep:.2f} м/с) | "
                  f"V_perp={v_perp_road:.2f} м/с (угол {theta_enc:.1f}°) | "
                  f"V_lat={v_lat_car:.2f} м/с (занос beta={beta:.1f}°, omega_z={omega_z:.1f}°/с) | "
                  f"Механизм: '{mechanism}' -> S{actor_sev}")

    # --------------------------------------------------------------------------
    # Основной цикл симуляции (on_tick)
    # --------------------------------------------------------------------------
    def on_tick(self, snapshot):
        if not self.is_running:
            return
        try:
            # ------------------------------------------------------------------
            # ФАЗА 0: Инициализация и конфигурация актеров при старте
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

                    # Настройка синхронного Traffic Manager для обоих ТС
                    self.ego_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.ego_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.ego_vehicle, 100)
                    self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    self.target_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.target_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.target_vehicle, 100)
                    self.tm.vehicle_lane_offset(self.target_vehicle, float(lateral_offset))

                    # Назначение физических масс в движке PhysX (чистая нативная физика подвески)
                    phys_ego = self.ego_vehicle.get_physics_control()
                    phys_ego.mass = EGO_MASS
                    self.ego_vehicle.apply_physics_control(phys_ego)

                    phys_target = self.target_vehicle.get_physics_control()
                    phys_target.mass = TARGET_MASS
                    self.target_vehicle.apply_physics_control(phys_target)

                    # Инициализация ПИД-контроллера траектории для Ego
                    if VehiclePIDController is not None:
                        args_lateral = {'K_P': 1.1, 'K_I': 0.03, 'K_D': 0.15, 'dt': self.dt}
                        args_long = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': self.dt}
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
            # ФАЗЫ 1 И 2 ПОСЛЕ КОНФИГУРАЦИИ
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

                # Кэширование пред-аварийной кинематики за 1 кадр до контакта для обоих ТС
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

                # Очистка старых кадров из словаря
                if len(self.speed_history) > 200:
                    old_keys = sorted(self.speed_history.keys())[:-100]
                    for k in old_keys:
                        del self.speed_history[k]

                # Камера зрителя следует за Ego
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
                # МОНИТОРИНГ БРОВКИ КЮВЕТА Y = 9.75 М ДЛЯ ОБОИХ ТС (Подкласс С)
                # --------------------------------------------------------------
                self.check_ditch_boundary_conditions('ego', self.ego_vehicle, lane_forward, lane_right, snapshot.frame)
                self.check_ditch_boundary_conditions('target', self.target_vehicle, lane_forward, lane_right, snapshot.frame)

                # Безопасное гашение скорости за пределами рельефа карты (Y > 21.0 м или Z < -2.0 м)
                for veh in [self.ego_vehicle, self.target_vehicle]:
                    if veh is not None:
                        try:
                            vloc = veh.get_location()
                            if vloc.y > 21.0 or vloc.z < -2.0:
                                veh.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                                veh.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                        except Exception:
                            pass

                # Перевод вытолкнутого Target на обочине (Y >= 7.50 м) в свободный выбег (SAE J2980)
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
                # ФАЗА 1: Синхронный разгон и эластичная сцепка в слепой зоне
                # --------------------------------------------------------------
                if not self.lane_change_triggered:
                    disp = target_loc - ego_loc
                    current_lon_offset = (disp.x * ego_fw.x) + (disp.y * ego_fw.y)
                    error_x = current_lon_offset - self.initial_lon_offset

                    # Вывод мониторинга разгона с интервалом 1.0 с
                    log_interval_p1 = max(1, int(round(1.0 / self.dt)))
                    if self.tick_count % log_interval_p1 == 0:
                        sim_time = self.tick_count * self.dt
                        print(f"[{sim_time:4.1f}c] Ego: {ego_mps*3.6:5.1f} км/ч | "
                              f"Target: {target_mps*3.6:5.1f} км/ч | Отклонение: {error_x:+.2f} м | "
                              f"Окно стабильности: {self.stability_time_sec:.2f}/1.00 c")

                    d_error = target_mps - ego_mps
                    self.target_err_integral += error_x * self.dt
                    self.target_err_integral = max(-10.0, min(self.target_err_integral, 10.0))

                    p_gain = 0.8
                    i_gain = 0.4
                    d_gain = 0.5

                    correction = -(error_x * p_gain + self.target_err_integral * i_gain + d_error * d_gain)
                    correction = max(-5.0, min(correction, 5.0))
                    target_desired_mps = max(5.0, min(ego_mps + correction + 0.4, 40.0))
                    self.tm.set_desired_speed(self.target_vehicle, float(target_desired_mps * 3.6))

                    # Периодическая подстройка круиз-контроля Ego раз в ~0.20 с
                    ego_corr_interval = max(1, int(round(0.20 / self.dt)))
                    if self.tick_count % ego_corr_interval == 0 and ego_mps > self.target_v0_mps * 0.5:
                        speed_err = self.target_v0_mps - ego_mps
                        if abs(speed_err) > 0.1:
                            self.ego_tm_speed += speed_err * 0.5
                            self.ego_tm_speed = max(10.0, min(self.ego_tm_speed, 35.0))
                            self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    is_speed_ready = ego_mps >= (CRUISE_SPEED_MPS - 0.15)
                    is_speed_sync = abs(ego_mps - target_mps) <= 0.15
                    is_dist_sync = abs(error_x) <= 0.15

                    # Проверка условий стабильности по непрерывному физическому времени (1.0 с)
                    if is_speed_ready and is_speed_sync and is_dist_sync:
                        self.stability_time_sec += self.dt
                        self.stability_counter += 1
                    else:
                        self.stability_time_sec = max(0.0, self.stability_time_sec - 2.0 * self.dt)
                        self.stability_counter = max(0, self.stability_counter - 2)

                    # Запасной триггер после 8.0 секунд стабильного движения
                    sim_time = self.tick_count * self.dt
                    forced_trigger = (sim_time >= 8.0 and ego_mps >= (CRUISE_SPEED_MPS - 0.5) and abs(error_x) <= 0.35)

                    if self.stability_time_sec >= 1.0 or forced_trigger:
                        self.lane_change_triggered = True
                        self.ego_vehicle.set_autopilot(False)
                        self.tm.set_desired_speed(self.target_vehicle, float(target_mps * 3.6))
                        print(f"\n[{snapshot.frame}] >>> МАНЕВР ИНИЦИИРОВАН ({self.subclass}): "
                              f"Ego начинает перестроение в слепую зону <<<")

                # --------------------------------------------------------------
                # ФАЗА 2: Опасное перестроение и когнитивная реакция водителя
                # --------------------------------------------------------------
                else:
                    if not self.lost_control:
                        # До первого удара: ведение автомобиля Ego ПИД-контроллером в соседнюю полосу
                        ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                        if ego_wp.lane_id == self.start_lane_id:
                            if self.subclass == SUBCLASS_A:
                                target_lane_wp = ego_wp.get_left_lane()
                            else:
                                target_lane_wp = ego_wp.get_right_lane()
                        else:
                            target_lane_wp = ego_wp

                        if target_lane_wp is not None:
                            target_wps = target_lane_wp.next(25.0)
                            target_wp = target_wps[0] if target_wps else target_lane_wp
                            if self.pid_controller is not None:
                                control = self.pid_controller.run_step(self.target_v0_mps * 3.6, target_wp)
                                self.last_steer = control.steer
                                self.ego_vehicle.apply_control(control)

                    else:
                        # ПОСЛЕ УДАРА: когнитивная модель водителя SAE J2980
                        ticks_since_impact = self.tick_count - self.collision_tick
                        time_since_impact = ticks_since_impact * self.dt

                        # 2.1. Фаза акустико-вестибулярного шока (t < T_shock)
                        if time_since_impact < self.shock_duration_sec:
                            if time_since_impact <= 0.25:
                                # Первые 0.25 с: тонический мышечный спазм рук на руле
                                self.current_steer = self.impact_steer
                            else:
                                # С 0.25 с до T_shock: пассивная релаксация руля кастором подвески (инвариантно к dt)
                                decay = math.pow(0.95, self.dt / 0.01)
                                self.current_steer *= decay

                            # Педали полностью сброшены (свободный выбег coasting, БЕЗ торможения)
                            self.ego_vehicle.apply_control(carla.VehicleControl(
                                throttle=0.0,
                                steer=float(self.current_steer),
                                brake=0.0,
                                hand_brake=False,
                                manual_gear_shift=False
                            ))

                        # 2.2. После окончания фазы шока (t >= T_shock): ветвление по driver_post_shock
                        else:
                            decay = math.pow(0.95, self.dt / 0.01)

                            # РЕЖИМ 1: 'passive' (По умолчанию — чистая оценка тяжести Severity S)
                            if self.driver_post_shock == 'passive':
                                # Водитель дезориентирован: педали сброшены, руль плавно релаксирует кастором
                                self.current_steer *= decay
                                self.ego_vehicle.apply_control(carla.VehicleControl(
                                    throttle=0.0,
                                    steer=float(self.current_steer),
                                    brake=0.0,
                                    hand_brake=False,
                                    manual_gear_shift=False
                                ))

                            # РЕЖИМ 2: 'imperfect' (Ограниченная реакция в состоянии паники по SAE J2980)
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
                                max_step = 4.0 * self.dt
                                steer_step = max(-max_step, min(max_step, steer_delta))
                                self.current_steer += steer_step

                                # Умеренное торможение без блокировки колес
                                brake_cmd = 0.35 if ego_loc.y < 7.50 else 0.20
                                self.ego_vehicle.apply_control(carla.VehicleControl(
                                    throttle=0.0,
                                    steer=float(self.current_steer),
                                    brake=float(brake_cmd),
                                    hand_brake=False,
                                    manual_gear_shift=False
                                ))

                            # РЕЖИМ 3: 'active' (Агрессивное парирование заноса для исследований Controllability C)
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
                                max_step = 8.0 * self.dt
                                steer_step = max(-max_step, min(max_step, steer_delta))
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

                    # Периодический мониторинг маневра в консоль с интервалом 0.5 с
                    log_interval_p2 = max(1, int(round(0.5 / self.dt)))
                    if self.tick_count % log_interval_p2 == 0:
                        cur_steer = self.current_steer if self.lost_control else self.last_steer
                        sim_time = self.tick_count * self.dt
                        print(f"[{sim_time:4.1f}c] МАНЕВР: Ego Y={ego_loc.y:5.2f} (v={ego_mps*3.6:5.1f} км/ч) "
                              f"| Target Y={target_loc.y:5.2f} (v={target_mps*3.6:5.1f} км/ч) | "
                              f"dY={target_loc.y - ego_loc.y:+5.2f} | Руль Ego: {cur_steer:+.3f} | Удар: {self.lost_control}")

        except Exception:
            traceback.print_exc()

    # --------------------------------------------------------------------------
    # Извлечение итоговых метрик по кювету (Подкласс С)
    # --------------------------------------------------------------------------
    def compute_ditch_metrics(self):
        """
        Возвращает метрики схода в кювет по зафиксированным граничным условиям на бровке Y=9.75 м.
        Если съехали оба ТС, выбирается ТС с наихудшим классом тяжести (или большей скоростью вылета).
        Возвращает:
        (sev_ditch_val, ditch_departure_actor, dep_speed, road_perp_speed,
         enc_angle, lat_skid, sideslip, yaw_rate, mechanism)
        """
        if not self.ditch_metrics_by_actor:
            return 0, 'None', '', '', '', '', '', '', ''

        # Определение строкового идентификатора участников вылета
        if 'ego' in self.ditch_metrics_by_actor and 'target' in self.ditch_metrics_by_actor:
            ditch_actor_str = 'both'
        elif 'ego' in self.ditch_metrics_by_actor:
            ditch_actor_str = 'ego'
        else:
            ditch_actor_str = 'target'

        # Выбор наихудшего исхода среди съехавших ТС
        worst_actor = None
        worst_sev = -1
        worst_dep_speed = -1.0

        for actor, data in self.ditch_metrics_by_actor.items():
            sev = data['actor_sev']
            v_dep = data['v_dep']
            if (sev > worst_sev) or (sev == worst_sev and v_dep > worst_dep_speed):
                worst_actor = actor
                worst_sev = sev
                worst_dep_speed = v_dep

        best_data = self.ditch_metrics_by_actor[worst_actor]

        return (
            best_data['actor_sev'],
            ditch_actor_str,
            round(best_data['v_dep'], 2),
            round(best_data['v_perp_road'], 2),
            round(best_data['theta_enc'], 1),
            round(best_data['v_lat_car'], 2),
            round(best_data['beta'], 1),
            round(best_data['omega_z'], 1),
            best_data['mechanism']
        )

    def get_barrier_collision_actor(self) -> str:
        """Возвращает строку участников удара в отбойник ('ego', 'target', 'both', или 'None')."""
        if 'ego' in self.barrier_hit_actors and 'target' in self.barrier_hit_actors:
            return 'both'
        elif 'ego' in self.barrier_hit_actors:
            return 'ego'
        elif 'target' in self.barrier_hit_actors:
            return 'target'
        else:
            return 'None'

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

        # Обязательное уничтожение обоих датчиков столкновений
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
    - Подключается к CARLA и инициализирует мир с картой 'map_ditch'.
    - Разрешает пути к ассетам OpenDRIVE (map_ditch.xodr).
    - Компилирует сценарии Scenic для Подклассов А (отбойник) и С (кювет).
    - Выполняет физические прогоны с сохранением кинематики в 25-колоночный CSV.
    - НЕ содержит логики статистической агрегации кампании (>10%).
    """
    parser = argparse.ArgumentParser(description="HARA LCC Blind Spot Simulation Campaign (Physical Runner)")
    parser.add_argument('--subclass', type=str, default='ALL', choices=['A', 'C', 'ALL'],
                        help="Road subclass: 'A' (Barrier), 'C' (Ditch), or 'ALL' (Campaign)")
    parser.add_argument('--driver-post-shock', type=str, default='passive',
                        choices=['passive', 'imperfect', 'active'],
                        help="Driver behavior mode after shock phase: 'passive' (default, coasting for pure Severity S), 'imperfect' (panic recovery), or 'active' (full counter-steering for Controllability C)")
    parser.add_argument('--mode', type=str, default='equal', choices=['equal', 'sequential', 'weighted'],
                        help="Campaign mode: 'equal' (50/50: N/2 of A, N/2 of C), 'sequential' (0.2 A / 0.8 C), or 'weighted'")
    parser.add_argument('--runs', type=int, default=20, help="Total number of simulation runs (default: 20)")
    parser.add_argument('--map-dir', type=str, default=DEFAULT_MAPS_DIR,
                        help="Path to directory containing map assets (default: maps/assets)")
    parser.add_argument('--clear', action='store_true', help="Clear/overwrite previous CSV results before running")
    parser.add_argument('--render', action='store_true', help="Enable Carla rendering window")
    parser.add_argument('--timestep', type=float, default=0.01,
                        help="Simulation timestep dt in seconds (default: 0.01, recommended: 0.005 - 0.025)")
    parser.add_argument('--seed', type=int, default=42,
                        help="Fixed random seed for deterministic Scenic scene generation and reproducibility (default: 42)")
    args = parser.parse_args()

    # Фиксация случайного зерна для воспроизводимости стохастических сцен
    if args.seed is not None:
        random.seed(args.seed)
        print(f"  [РЕПРОДУЦИРУЕМОСТЬ] Установлен фиксированный random seed: {args.seed}")

    # Проверка диапазона шага симуляции dt
    if args.timestep < 0.005 or args.timestep > 0.025:
        print(f"  [ПРЕДУПРЕЖДЕНИЕ] Шаг симуляции dt={args.timestep}c находится вне рекомендованного диапазона [0.005..0.025] с. Возможна нестабильность интегратора PhysX или избыточное время моделирования.")

    print("=" * 80)
    print(" HARA SIMULATION CAMPAIGN: LANE CHANGE COLLISION (LCC) BLIND SPOT FAILURE")
    print(" ISO 26262-3 / SAE J2980 (Dual Collision Sensors & Ditch Boundary Conditions)")
    print(f" Post-Shock Driver Mode: '{args.driver_post_shock}' (Default: passive)")
    print(f" Simulation Timestep dt: {args.timestep} s (Substepping: max_substep=0.01s, max_substeps=10)")
    print("=" * 80)

    # 1. Подключение к симулятору CARLA
    print("\n[1/4] Подключение к CARLA (localhost:2000)...")
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        print(f"  [OK] Успешное подключение к CARLA. Текущая карта: {world.get_map().name}")

        # Настройка PhysX substepping в CARLA для стабильной контактной динамики
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.timestep
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)
        print(f"  [OK] Режим PhysX substepping включен (dt={args.timestep}c, max_substep_dt=0.01c, max_substeps=10).")
    except Exception as e:
        print(f"  [ОШИБКА] Не удалось подключиться к CARLA / настроить PhysX: {e}")
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
            timestep=args.timestep,
            render=args.render
        )
        print("  [OK] CarlaSimulator успешно инициализирован.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой инициализации CarlaSimulator: {e}")
        sys.exit(1)

    # 3. Компиляция сценариев Scenic для обоих подклассов с актуальным путем к map и timestep
    print(f"\n[3/4] Компиляция Scenic сценария ({SCENIC_PATH})...")
    scenarios = {}
    try:
        scenarios[SUBCLASS_A] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_A, 'map': map_xodr_path, 'timestep': args.timestep}
        )
        scenarios[SUBCLASS_C] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_C, 'map': map_xodr_path, 'timestep': args.timestep}
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
            runs_a = args.runs // 2
            runs_c = args.runs - runs_a
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_C] * runs_c
        elif args.mode == 'sequential':
            runs_a = max(1, int(round(args.runs * 0.20)))
            runs_c = max(1, args.runs - runs_a)
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_C] * runs_c
        else:
            campaign_plan = random.choices([SUBCLASS_A, SUBCLASS_C], weights=[0.20, 0.80], k=args.runs)

    print(f"\n[4/4] Запуск кампании: всего {len(campaign_plan)} прогонов "
          f"({campaign_plan.count(SUBCLASS_A)} Подкласс А, {campaign_plan.count(SUBCLASS_C)} Подкласс С)")

    # Подготовка выходного CSV-файла со строгой 25-колоночной схемой
    if args.clear and os.path.isfile(CSV_PATH):
        try:
            os.remove(CSV_PATH)
        except Exception:
            pass

    file_exists = os.path.isfile(CSV_PATH)

    # Проверка совместимости колонок в существующем файле
    if file_exists and not args.clear:
        try:
            with open(CSV_PATH, 'r', encoding='utf-8-sig') as f_check:
                header_line = f_check.readline()
                if 'ditch_mechanism' not in header_line or 'sev_side' not in header_line:
                    # Устаревший формат колонок: создаем резервную копию
                    backup_csv = os.path.join(RESULTS_DIR, "lcc_hara_results_legacy_backup.csv")
                    os.replace(CSV_PATH, backup_csv)
                    print(f"  [ИНФО] Старый CSV сохранен в резервную копию: {backup_csv}")
                    file_exists = False
        except Exception:
            pass

    with open(CSV_PATH, mode='w' if args.clear or not file_exists else 'a', newline='', encoding='utf-8-sig') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
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

            # Генерация стохастической сцены Scenic
            scenario = scenarios[subclass]
            scene, _ = scenario.generate()

            # Создание и запуск менеджера симуляции
            manager = AutopilotManager(
                client,
                world,
                subclass=subclass,
                driver_post_shock=args.driver_post_shock,
                dt=args.timestep
            )
            manager.start()

            try:
                # Запуск симуляции Scenic (таймаут 30 секунд физического времени)
                max_steps = int(round(30.0 / args.timestep))
                simulator.simulate(scene, maxSteps=max_steps)
            except Exception as e:
                print(f"[ПРЕДУПРЕЖДЕНИЕ] Симуляция завершена или прервана: {e}")
            finally:
                # Остановка сенсоров и колбэков
                manager.stop()
                time.sleep(0.3)

                # --------------------------------------------------------------
                # Сбор и каскадная агрегация метрик тяжести по ISO 26262-3
                # --------------------------------------------------------------
                has_side_hit = manager.side_collision_frame is not None
                has_barrier_hit = manager.barrier_collision_frame is not None
                barrier_actor = manager.get_barrier_collision_actor()

                # Метрики кювета по граничным условиям
                (sev_ditch_val, ditch_actor, dep_speed, road_perp_speed,
                 enc_angle, lat_skid, sideslip, yaw_rate,
                 mechanism) = manager.compute_ditch_metrics()

                # Итоговая максимальная тяжесть по одиночному прогону
                final_sev_val = max(manager.sev_side_val, manager.sev_barrier_val, sev_ditch_val)
                severity = f"S{final_sev_val}"

                # Классификация исхода сценария на русском языке
                outcome = "Без столкновений"
                if subclass == SUBCLASS_A:
                    b_type = manager.barrier_impact_type
                    actor_desc = f" ({barrier_actor})" if barrier_actor != 'None' else ""
                    if has_side_hit and has_barrier_hit:
                        outcome = f"Боковой удар с целевым ТС + {b_type} удар в отбойник{actor_desc}"
                    elif has_barrier_hit:
                        outcome = f"{b_type} удар в отбойник{actor_desc} (без удара целевого ТС)"
                    elif has_side_hit:
                        outcome = "Боковой удар с целевым ТС (Без удара в отбойник)"
                else:  # SUBCLASS_C
                    prefix = "Боковой удар с целевым ТС + " if has_side_hit else ""
                    if ditch_actor != 'None':
                        outcome = f"{prefix}{mechanism} ({ditch_actor}) ({severity})"
                    elif has_side_hit:
                        outcome = "Боковой удар с целевым ТС (Без съезда в кювет)"

                # Формирование строки для CSV (строго 25 колонок)
                row_data = {
                    'run_id': run_idx,
                    'subclass': subclass,
                    'driver_post_shock_mode': manager.driver_post_shock,
                    'shock_duration_sec': round(manager.shock_duration_sec, 3),
                    'side_collision': has_side_hit,
                    'delta_v_ego_mps': round(manager.delta_v_ego, 2) if has_side_hit else '',
                    'delta_v_target_mps': round(manager.delta_v_target, 2) if has_side_hit else '',
                    'effective_side_dv_mps': round(manager.effective_side_delta_v, 2) if has_side_hit else '',
                    'sev_side': f"S{manager.sev_side_val}",
                    'barrier_collision': has_barrier_hit,
                    'barrier_collision_actor': barrier_actor,
                    'barrier_impact_type': manager.barrier_impact_type if has_barrier_hit else '',
                    'barrier_impact_mps': round(manager.barrier_impact_speed, 2) if has_barrier_hit else '',
                    'sev_barrier': f"S{manager.sev_barrier_val}",
                    'ditch_departure_actor': ditch_actor,
                    'departure_speed_mps': dep_speed if ditch_actor != 'None' else '',
                    'road_perp_speed_mps': road_perp_speed if ditch_actor != 'None' else '',
                    'encroachment_angle_deg': enc_angle if ditch_actor != 'None' else '',
                    'vehicle_lat_skid_mps': lat_skid if ditch_actor != 'None' else '',
                    'sideslip_angle_deg': sideslip if ditch_actor != 'None' else '',
                    'departure_yaw_rate_deg_s': yaw_rate if ditch_actor != 'None' else '',
                    'ditch_mechanism': mechanism if ditch_actor != 'None' else '',
                    'sev_ditch': f"S{sev_ditch_val}",
                    'severity': severity,
                    'scenario_outcome': outcome
                }

                writer.writerow(row_data)
                csv_file.flush()

                # Сводный вывод в консоль
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
                    print(f"  * Вылет в кювет: {ditch_actor} " +
                          (f"(Скорость: {dep_speed:.2f} м/с, Угол beta: {sideslip:.1f}°, Механизм: '{mechanism}', S_ditch: S{sev_ditch_val})" if ditch_actor != 'None' else ""))
                print(f"  * Итоговая тяжесть прогона: >>> {severity} <<<")
                print(f"  * Исход: {outcome}")
                print("-" * 80)

                time.sleep(0.3)

    print("\n" + "=" * 80)
    print(f"КАМПАНИЯ СИМУЛЯЦИЙ ЗАВЕРШЕНА!")
    print(f"Результаты записаны в: {CSV_PATH}")
    print(f"Для статистического анализа по правилу >10% ISO 26262-3 запустите:")
    print(f"  python analyze_results.py --input {CSV_PATH}")
    print("=" * 80)


if __name__ == '__main__':
    main()
