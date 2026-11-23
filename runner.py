# ==============================================================================
# HARA SIMULATION RUNNER: Lane Change Collision (LCC) Blind Spot Failure
# Стандарты: ISO 26262-3:2018 (Clause 6) / SAE J2980:2023 (5.2.3, Table B1)
# Архитектура: Исполнитель одиночных физических симуляций CARLA + Scenic
# Методология: Двусторонний мониторинг (Ego + Target), Метод граничных условий на бровке кювета
# ==============================================================================
#
# КАРТА И СИСТЕМА КООРДИНАТ MAP_DITCH:
# - Продольная ось дороги: +X (направление на Восток, крейсерская скорость 100 км/ч = 27.78 м/с).
# - Поперечная ось дороги: +Y (вправо по ходу движения ТС).
# - Вертикальная ось: +Z (вверх, уровень проезжей части и обочины Z = 0.0 м).
# - Левая полоса (Lane -1): центр Y = +1.875 м. Слева (Y <= 0.0 м) жёсткое вертикальное ограждение (h = 0.80 м).
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
import shutil

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
# Carla Python API Configuration (C:\CARLA)
# ------------------------------------------------------------------------------
CARLA_API_PATH = r"C:\CARLA\PythonAPI\carla"
if CARLA_API_PATH not in sys.path and os.path.isdir(CARLA_API_PATH):
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
FRAME_LOGS_DIR = os.path.join(RESULTS_DIR, "frame_logs")
MAX_FRAME_LOGS_BYTES = 1024 * 1024 * 1024  # Лимит 1 ГБ для покадровых логов
MIN_DISK_FREE_BYTES = int(1.5 * 1024 * 1024 * 1024)  # Минимально допустимый резерв свободного места (1.5 ГБ)

DEFAULT_MAPS_DIR = os.path.join(BASE_DIR, "maps", "assets")
FALLBACK_MAP_DIRS = [
    DEFAULT_MAPS_DIR,
    os.path.join(BASE_DIR, "maps"),
    os.path.join(BASE_DIR, "..", "maps", "assets"),
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


# Физические параметры участников сценария:
# Ego: vehicle.chevrolet.impala с заданной спец. массой защищаемого ОА (аналог Aurus Senat)
EGO_MASS = 3490.0            # Масса ТС Ego (кг)

# Target: vehicle.seat.leon (стохастическая масса из Scenic scene.params['target_mass_kg'])
TARGET_MASS_DEFAULT = 1480.0 # Резервная масса Target (кг), если параметр Scenic не передан

# Пропорциональный EBD max_brake_torque (Config F: ~65% передняя / ~35% задняя ось)
EGO_FRONT_BRAKE_TORQUE_NM = 3800.0
EGO_REAR_BRAKE_TORQUE_NM = 2050.0
TARGET_FRONT_BRAKE_NM_PER_KG = 1.00
TARGET_REAR_BRAKE_NM_PER_KG = 0.54

# Коэффициент восстановления при скользящем боковом контакте (Brach, Brach & Pongetti 2014, SAE 2014-01-0469)
RESTITUTION_E = 0.15

# Безразмерные множители сцепления UE4 UTireConfig::FrictionScale (Config F, tireFriction = mu_PhysMat * FrictionScale)
ASPHALT_TIRE_FRICTION_SCALE = 1.05   # Сухой чистый асфальт (~0.81-0.86 g, FMVSS 135 / NCAP)
GRAVEL_TIRE_FRICTION_SCALE = 0.57    # Укрепленная гравийная обочина (~0.46-0.48 g; отношение 0.57 / 1.05 = 0.543)

# Крейсерская скорость потока (100 км/ч)
CRUISE_SPEED_MPS = 100.0 / 3.6  # 27.78 м/с

# Идентификаторы дорожных подклассов (с весами в ODD)
SUBCLASS_A = "SUBCLASS_A_BARRIER"  # Маневр влево: риск удара о жёсткое ограждение (W_A = 0.20)
SUBCLASS_B = "SUBCLASS_B_DITCH"    # Маневр вправо: риск схода в кювет на бровке Y = 9.75 м (W_B = 0.80)

# Параметры модели задержки реакции (Green 2000, Olson & Sivak 1986, Taoka 1989, SAE J2980 5.2.3.2(d))
# и пост-ударного восстановления ('active', DOT HS 809 875 / 72 FR 17284)
REACTION_TIME_MEDIAN_SEC = 1.05
REACTION_TIME_LOG_SIGMA = 0.22
REACTION_TIME_MIN_SEC = 0.65
REACTION_TIME_MAX_SEC = 1.60
HOLD_TIME_MIN_SEC = 0.20
HOLD_TIME_MAX_SEC = 0.35
ACTIVE_MAX_STEER_RATE_PER_SEC = 2.5
BRAKE_RISE_TIME_SEC = 0.25

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

FRAME_LOG_FIELDNAMES = [
    # Общие параметры кадра
    'frame',
    'tick_count',
    'sim_time_sec',
    'phase',
    'lane_change_triggered',
    'lost_control',
    'time_since_impact_sec',
    'side_collision_occurred',
    'barrier_collision_occurred',
    # Параметры Ego
    'ego_x',
    'ego_y',
    'ego_z',
    'ego_roll_deg',
    'ego_pitch_deg',
    'ego_yaw_deg',
    'ego_vx',
    'ego_vy',
    'ego_vz',
    'ego_speed_mps',
    'ego_speed_kmh',
    'ego_v_lon_car_mps',
    'ego_v_lat_car_mps',
    'ego_v_perp_road_mps',
    'ego_sideslip_beta_deg',
    'ego_road_angle_theta_deg',
    'ego_omega_x_deg_s',
    'ego_omega_y_deg_s',
    'ego_omega_z_deg_s',
    'ego_ax',
    'ego_ay',
    'ego_az',
    'ego_steer',
    'ego_throttle',
    'ego_brake',
    'ego_in_ditch',
    # Параметры Target
    'target_x',
    'target_y',
    'target_z',
    'target_roll_deg',
    'target_pitch_deg',
    'target_yaw_deg',
    'target_vx',
    'target_vy',
    'target_vz',
    'target_speed_mps',
    'target_speed_kmh',
    'target_v_lon_car_mps',
    'target_v_lat_car_mps',
    'target_v_perp_road_mps',
    'target_sideslip_beta_deg',
    'target_road_angle_theta_deg',
    'target_omega_x_deg_s',
    'target_omega_y_deg_s',
    'target_omega_z_deg_s',
    'target_ax',
    'target_ay',
    'target_az',
    'target_steer',
    'target_throttle',
    'target_brake',
    'target_in_ditch'
]


def normalize_deg(angle: float) -> float:
    """Нормализация углов Эйлера к диапазону [-180, 180] градусов."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def sample_reaction_and_hold_times(rng=random) -> tuple[float, float]:
    """
    Сэмплирование времени реакции водителя (T_react) и времени мышечного удержания (T_hold).
    T_react сэмплируется из усечённого логнормального распределения (Green 2000, Taoka 1989)
    для трезвого внимательного водителя (SAE J2980:2023 Clause 5.2.3.2(d)).
    T_hold — инженерная эвристика кратковременного удержания руля/газа до сброса педали.
    """
    raw_react = math.exp(rng.gauss(math.log(REACTION_TIME_MEDIAN_SEC), REACTION_TIME_LOG_SIGMA))
    t_react = min(REACTION_TIME_MAX_SEC, max(REACTION_TIME_MIN_SEC, raw_react))
    t_hold = rng.uniform(HOLD_TIME_MIN_SEC, min(HOLD_TIME_MAX_SEC, t_react))
    return t_react, t_hold


def classify_ditch_boundary(
    v_dep: float,
    v_perp_road: float,
    v_lat_car: float,
    beta: float,
    omega_z: float,
) -> tuple[int, str]:
    """
    Чистый классификатор тяжести и механизма схода в кювет строго на бровке Y = 9.75 м (Вариант B).
    Возвращает (actor_sev [0..3], mechanism_name).
    """
    is_non_tracking = (beta >= 15.0)

    # S3: Критические / фатальные травмы (MAIS 5-6 > 10%, широкоугольный срыв beta >= 25° / быстрый спин или жёсткий упор носом)
    is_s3_skid = (v_dep > 18.0) and ((beta >= 25.0) or (beta >= 20.0 and omega_z > 45.0))
    is_s3_nose = (v_dep > 8.0) and (v_perp_road > 12.0)

    # S2: Тяжёлые травмы (MAIS 3-6 > 10%, инициация soil-trip Cooperrider V_lat > 5.5 м/с при beta >= 15°,
    # выраженный non-tracking срыв NCHRP 665 beta >= 20° или удар о дно кювета V_perp > 8.0 м/с)
    is_s2_trip = (v_dep > 14.0) and is_non_tracking and (v_lat_car > 5.5)
    is_s2_skid = (v_dep > 11.0) and (beta >= 20.0)
    is_s2_bottom = (v_dep > 8.0) and (v_perp_road > 8.0)

    # S1: Лёгкие и средние травмы (Alberta RDG Table HA.4 SI=1.8 / FHWA-HRT-21-045: проходимый съезд или занос beta >= 15°)
    is_s1_tracking = (v_dep > 8.0 and v_perp_road > 2.0)
    is_s1_skid = is_non_tracking

    if is_s3_skid or is_s3_nose:
        mechanism = 'High-Speed Soil-Trip Rollover' if is_s3_skid else 'Severe Ditch Nose-In Impact'
        return 3, mechanism
    if is_s2_trip or is_s2_skid or is_s2_bottom:
        mechanism = 'Moderate Rollover Risk' if (is_s2_trip or is_s2_skid) else 'Hard Ditch Bottom Impact'
        return 2, mechanism
    if is_s1_tracking or is_s1_skid:
        return 1, 'Traversable Tracking Slope Departure'
    return 0, 'Shallow Grazing Departure'


def compute_post_impact_control(
    actor_name: str,
    subclass: str,
    driver_post_shock: str,
    time_since_impact: float,
    dt: float,
    loc_y: float,
    yaw_deg: float,
    vy: float,
    impact_steer: float,
    impact_throttle: float,
    current_steer: float,
    current_brake: float,
    reaction_sec: float,
    hold_sec: float,
    in_ditch_or_barrier: bool,
    beta_deg: float = 0.0,
) -> tuple[float, float, float]:
    """
    Чистая расчётная функция управления ТС после первичного контакта:
    - t <= T_hold: удержание доударного входа (с принудительным сбросом газа у отбойника / на бровке).
    - T_hold < t < T_react: сброс газа, выбег с эвристическим возвратом руля кастором.
    - t >= T_react: при 'passive' — продолжение выбега (оценка чистой Severity S);
      при 'active' — раздельное удержание своей полосы (Ego vs Target), ограниченный темп руля
      (<= 2.5 ед/с по DOT HS 809 875), линейная рампа тормозного давления (t_rise = 0.25 с)
      и бортовая модуляция Kamm + Sideslip (|beta_deg|) без мировых координатных оракулов.
    """
    decay = math.pow(0.95, dt / 0.01)

    if time_since_impact < reaction_sec:
        if time_since_impact <= hold_sec:
            thr = 0.0 if in_ditch_or_barrier else impact_throttle
            return float(thr), float(impact_steer), 0.0
        return 0.0, float(current_steer * decay), 0.0

    if driver_post_shock == 'passive':
        return 0.0, float(current_steer * decay), 0.0

    if subclass == SUBCLASS_A:
        target_road_y = 5.625 if actor_name == 'ego' else 1.875
    else:
        target_road_y = 1.875 if actor_name == 'ego' else 5.625

    lateral_err = loc_y - target_road_y
    yaw_rad = math.radians(normalize_deg(yaw_deg))
    desired_steer = max(-1.0, min(1.0, -(0.40 * lateral_err + 0.15 * vy + 0.50 * yaw_rad)))

    max_step = ACTIVE_MAX_STEER_RATE_PER_SEC * dt
    steer_delta = desired_steer - current_steer
    new_steer = max(-1.0, min(1.0, current_steer + max(-max_step, min(max_step, steer_delta))))

    is_danger_zone = (abs(lateral_err) > 1.2) or (abs(beta_deg) > 4.0)
    desired_brake = 1.0 if is_danger_zone else 0.7
    brake_step = dt / BRAKE_RISE_TIME_SEC
    new_brake = min(desired_brake, current_brake + brake_step)
    if new_brake > 0.0:
        kamm_steer = max(0.40, 1.0 - 0.55 * min(1.0, abs(new_steer) / 0.6))
        beta_scale = max(0.45, 1.0 - max(0.0, abs(beta_deg) - 5.0) / 18.0)
        new_brake = min(new_brake, kamm_steer * beta_scale)

    return 0.0, float(new_steer), float(new_brake)


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

    def __init__(
        self,
        client,
        world,
        subclass: str = SUBCLASS_B,
        driver_post_shock: str = 'passive',
        dt: float = 0.01,
        log_frames: bool = False,
        target_mass: float = TARGET_MASS_DEFAULT,
    ):
        self.client = client
        self.world = world
        self.subclass = subclass
        self.driver_post_shock = driver_post_shock
        self.dt = float(dt)
        self.log_frames = log_frames
        self.target_mass = float(target_mass)
        self.frame_buffer = []

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
        self.stability_time_sec = 0.0

        self.pid_controller = None
        self.start_lane_id = None

        # Датчики столкновений для обоих ТС (ISO 26262-3)
        self.ego_collision_sensor = None
        self.target_collision_sensor = None
        self.friction_trigger = None
        self.speed_history = {}

        # ----------------------------------------------------------------------
        # Модель реакции водителей Ego и Target (Green 2000, Taoka 1989, SAE J2980:2023 5.2.3.2(d))
        # ----------------------------------------------------------------------
        self.ego_shock_duration_sec, self.ego_spasm_duration_sec = sample_reaction_and_hold_times()
        self.target_shock_duration_sec, self.target_spasm_duration_sec = sample_reaction_and_hold_times()

        self.ego_shock_duration_ticks = max(1, int(round(self.ego_shock_duration_sec / self.dt)))

        # Обратная совместимость с CSV-выгрузкой (shock_duration_sec)
        self.shock_duration_sec = self.ego_shock_duration_sec
        self.shock_duration_ticks = self.ego_shock_duration_ticks

        # Кэш параметров управления Ego до и после удара
        self.last_steer = 0.0
        self.last_throttle = 0.55
        self.impact_steer = 0.0
        self.impact_throttle = 0.55
        self.current_steer = 0.0
        self.ego_brake_cmd = 0.0

        # Кэш параметров управления Target до и после удара
        self.last_target_steer = 0.0
        self.last_target_throttle = 0.55
        self.target_impact_steer = 0.0
        self.target_impact_throttle = 0.55
        self.target_current_steer = 0.0
        self.target_brake_cmd = 0.0

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
        # Вторичный удар об ограждение (Подкласс А)
        # ----------------------------------------------------------------------
        self.barrier_collision_frame = None
        self.barrier_impact_speed = 0.0
        self.barrier_impact_type = ""
        self.sev_barrier_val = 0
        self.barrier_hit_actors = set()  # {'ego', 'target'}

        # ----------------------------------------------------------------------
        # Граничные условия вылета в кювет на бровке Y = 9.75 м (Подкласс B)
        # ----------------------------------------------------------------------
        self.ditch_metrics_by_actor = {}  # actor_name ('ego'/'target') -> dict of metrics

    def _activate_shock_phase(self, frame: int):
        """Симметричный перевод обоих ТС в пост-ударное состояние при первом контакте."""
        if self.lost_control:
            return
        self.lost_control = True
        self.collision_tick = self.tick_count

        self.impact_steer = self.last_steer
        self.impact_throttle = self.last_throttle
        self.current_steer = self.impact_steer
        self.ego_brake_cmd = 0.0

        self.target_impact_steer = self.last_target_steer
        self.target_impact_throttle = self.last_target_throttle
        self.target_current_steer = self.target_impact_steer
        self.target_brake_cmd = 0.0

        for veh, thr, steer in (
            (self.ego_vehicle, self.impact_throttle, self.impact_steer),
            (self.target_vehicle, self.target_impact_throttle, self.target_impact_steer),
        ):
            if veh is not None:
                try:
                    veh.set_autopilot(False)
                    veh.apply_control(carla.VehicleControl(
                        throttle=float(thr),
                        steer=float(steer),
                        brake=0.0,
                        hand_brake=False,
                        manual_gear_shift=False
                    ))
                except Exception:
                    pass

        print(f"[{frame}] ->> ПОСТ-УДАРНАЯ ФАЗА АКТИВИРОВАНА (EGO & TARGET): "
              f"Ego [T_react={self.ego_shock_duration_sec:.2f}c, T_hold={self.ego_spasm_duration_sec:.2f}c, "
              f"steer={self.impact_steer:+.3f}, thr={self.impact_throttle:.2f}] | "
              f"Target [T_react={self.target_shock_duration_sec:.2f}c, T_hold={self.target_spasm_duration_sec:.2f}c, "
              f"steer={self.target_impact_steer:+.3f}, thr={self.target_impact_throttle:.2f}]. "
              f"Режим после реакции: '{self.driver_post_shock}'.")

    # --------------------------------------------------------------------------
    # Обработчик столкновений (Collision Sensor Callback для обоих ТС)
    # --------------------------------------------------------------------------
    def on_collision(self, event, source_actor: str = 'ego'):
        """
        Универсальный обработчик столкновений для Ego и Target по ISO 26262-3:
        - Обнаруживает первичное боковое столкновение ТС-ТС.
        - Обнаруживает вторичный удар о жёсткое ограждение для Ego или Target.
        """
        other = event.other_actor
        frame = event.frame

        # Извлечение кинематики строго за 1 кадр до контакта (PhysX clean vectors)
        hist = self.speed_history.get(frame - 1)
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

                # Истинные переданные Delta-V по закону сохранения импульса для фактической массы Target
                m_ego = EGO_MASS
                m_target = self.target_mass
                e = RESTITUTION_E

                self.delta_v_ego = (m_target / (m_ego + m_target)) * (1.0 + e) * self.v_closing_lat
                self.delta_v_target = (m_ego / (m_ego + m_target)) * (1.0 + e) * self.v_closing_lat
                self.effective_side_delta_v = max(self.delta_v_ego, self.delta_v_target)

                # Калиброванная шкала SAE J2980:2023 Table B1 (боковой удар)
                if self.effective_side_delta_v > 8.0:
                    self.sev_side_val = 3
                elif self.effective_side_delta_v > 3.5:
                    self.sev_side_val = 2
                elif self.effective_side_delta_v > 1.0:
                    self.sev_side_val = 1
                else:
                    self.sev_side_val = 0

                print(f"\n[{frame}] ->> СТОЛКНОВЕНИЕ ТС-ТС (SAE J2980, m_tgt={m_target:.1f} кг): "
                      f"v_closing_lat = {self.v_closing_lat:.2f} м/с | "
                      f"Delta-V Ego = {self.delta_v_ego:.2f} м/с, "
                      f"Target = {self.delta_v_target:.2f} м/с | "
                      f"S_side = S{self.sev_side_val}")

            self._activate_shock_phase(frame)

        # ----------------------------------------------------------------------
        # 2. Вторичный удар об ограждение (Подкласс А) для Ego или Target
        # ----------------------------------------------------------------------
        else:
            if self.subclass != SUBCLASS_A:
                return
            actor_veh = self.ego_vehicle if source_actor == 'ego' else self.target_vehicle
            if actor_veh is None:
                return
            try:
                loc_y = actor_veh.get_location().y
            except Exception:
                return

            # Ограждение (0.80 м) расположено на Y = 0.0 м. С учётом габаритов Impala (L=5.36 м,
            # полудиагональ ~2.86 м) отсекаем срабатывания при Y > 3.5 м (правая полоса / обочина).
            if loc_y > 3.5:
                return

            # Кинематика участника за 1 кадр до удара (frame - 1)
            if source_actor == 'ego':
                v_actor = ego_v
                fw_actor = ego_forward
            else:
                v_actor = target_v
                fw_actor = target_forward

            # Нормальная скорость впечатывания строго ВНУТРЬ левого ограждения (отскок от барьера >= 0 обнуляется)
            v_inward_lr = (v_actor.x * lane_right.x) + (v_actor.y * lane_right.y)
            v_barrier = max(0.0, -v_inward_lr)

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
                # Угол атаки <= 45 град (Side-swipe: боковой скользящий удар)
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

            print(f"\n[{frame}] ->> УДАР ОБ ОГРАЖДЕНИЕ ({source_actor.upper()}): "
                  f"Тип: {cur_impact_type} | "
                  f"Скорость удара = {v_barrier:.2f} м/с | "
                  f"S_barrier = S{cur_sev} (макс. по прогону: S{self.sev_barrier_val})")

            self._activate_shock_phase(frame)

    # --------------------------------------------------------------------------
    # Покадровое логирование телеметрии обоих автомобилей
    # --------------------------------------------------------------------------
    def _record_telemetry_frame(self, snapshot, ego_snap, target_snap, lane_forward, lane_right):
        """Фиксация полной покадровой телеметрии Ego и Target в буфер памяти."""
        try:
            sim_time = self.tick_count * self.dt
            phase = '2_maneuver' if self.lane_change_triggered else '1_sync'
            time_since_impact = (
                (self.tick_count - self.collision_tick) * self.dt
                if (self.lost_control and self.collision_tick is not None)
                else 0.0
            )

            # 1. Кинематика Ego
            e_tf = ego_snap.get_transform()
            e_loc = e_tf.location
            e_rot = e_tf.rotation
            e_vel = ego_snap.get_velocity()
            e_spd = math.hypot(e_vel.x, e_vel.y)
            e_fw = e_tf.get_forward_vector()
            e_rt = e_tf.get_right_vector()
            e_v_lon = e_vel.x * e_fw.x + e_vel.y * e_fw.y
            e_v_lat = e_vel.x * e_rt.x + e_vel.y * e_rt.y
            e_v_perp = e_vel.x * lane_right.x + e_vel.y * lane_right.y
            e_beta = math.degrees(math.atan2(abs(e_v_lat), max(abs(e_v_lon), 0.01)))
            e_v_lon_road = e_vel.x * lane_forward.x + e_vel.y * lane_forward.y
            e_theta = math.degrees(math.atan2(abs(e_v_perp), max(abs(e_v_lon_road), 0.01)))
            e_ang_vel = ego_snap.get_angular_velocity()
            e_acc = ego_snap.get_acceleration()

            try:
                e_ctrl = self.ego_vehicle.get_control()
                e_steer, e_throttle, e_brake = e_ctrl.steer, e_ctrl.throttle, e_ctrl.brake
            except Exception:
                e_steer, e_throttle, e_brake = 0.0, 0.0, 0.0

            # 2. Кинематика Target
            t_tf = target_snap.get_transform()
            t_loc = t_tf.location
            t_rot = t_tf.rotation
            t_vel = target_snap.get_velocity()
            t_spd = math.hypot(t_vel.x, t_vel.y)
            t_fw = t_tf.get_forward_vector()
            t_rt = t_tf.get_right_vector()
            t_v_lon = t_vel.x * t_fw.x + t_vel.y * t_fw.y
            t_v_lat = t_vel.x * t_rt.x + t_vel.y * t_rt.y
            t_v_perp = t_vel.x * lane_right.x + t_vel.y * lane_right.y
            t_beta = math.degrees(math.atan2(abs(t_v_lat), max(abs(t_v_lon), 0.01)))
            t_v_lon_road = t_vel.x * lane_forward.x + t_vel.y * lane_forward.y
            t_theta = math.degrees(math.atan2(abs(t_v_perp), max(abs(t_v_lon_road), 0.01)))
            t_ang_vel = target_snap.get_angular_velocity()
            t_acc = target_snap.get_acceleration()

            try:
                t_ctrl = self.target_vehicle.get_control()
                t_steer, t_throttle, t_brake = t_ctrl.steer, t_ctrl.throttle, t_ctrl.brake
            except Exception:
                t_steer, t_throttle, t_brake = 0.0, 0.0, 0.0

            frame_data = {
                'frame': snapshot.frame,
                'tick_count': self.tick_count,
                'sim_time_sec': round(sim_time, 4),
                'phase': phase,
                'lane_change_triggered': self.lane_change_triggered,
                'lost_control': self.lost_control,
                'time_since_impact_sec': round(time_since_impact, 4),
                'side_collision_occurred': self.side_collision_frame is not None,
                'barrier_collision_occurred': self.barrier_collision_frame is not None,

                'ego_x': round(e_loc.x, 3),
                'ego_y': round(e_loc.y, 3),
                'ego_z': round(e_loc.z, 3),
                'ego_roll_deg': round(normalize_deg(e_rot.roll), 2),
                'ego_pitch_deg': round(normalize_deg(e_rot.pitch), 2),
                'ego_yaw_deg': round(normalize_deg(e_rot.yaw), 2),
                'ego_vx': round(e_vel.x, 3),
                'ego_vy': round(e_vel.y, 3),
                'ego_vz': round(e_vel.z, 3),
                'ego_speed_mps': round(e_spd, 3),
                'ego_speed_kmh': round(e_spd * 3.6, 2),
                'ego_v_lon_car_mps': round(e_v_lon, 3),
                'ego_v_lat_car_mps': round(e_v_lat, 3),
                'ego_v_perp_road_mps': round(e_v_perp, 3),
                'ego_sideslip_beta_deg': round(e_beta, 2),
                'ego_road_angle_theta_deg': round(e_theta, 2),
                'ego_omega_x_deg_s': round(e_ang_vel.x, 2),
                'ego_omega_y_deg_s': round(e_ang_vel.y, 2),
                'ego_omega_z_deg_s': round(e_ang_vel.z, 2),
                'ego_ax': round(e_acc.x, 3),
                'ego_ay': round(e_acc.y, 3),
                'ego_az': round(e_acc.z, 3),
                'ego_steer': round(e_steer, 4),
                'ego_throttle': round(e_throttle, 4),
                'ego_brake': round(e_brake, 4),
                'ego_in_ditch': bool(e_loc.y >= 9.75),

                'target_x': round(t_loc.x, 3),
                'target_y': round(t_loc.y, 3),
                'target_z': round(t_loc.z, 3),
                'target_roll_deg': round(normalize_deg(t_rot.roll), 2),
                'target_pitch_deg': round(normalize_deg(t_rot.pitch), 2),
                'target_yaw_deg': round(normalize_deg(t_rot.yaw), 2),
                'target_vx': round(t_vel.x, 3),
                'target_vy': round(t_vel.y, 3),
                'target_vz': round(t_vel.z, 3),
                'target_speed_mps': round(t_spd, 3),
                'target_speed_kmh': round(t_spd * 3.6, 2),
                'target_v_lon_car_mps': round(t_v_lon, 3),
                'target_v_lat_car_mps': round(t_v_lat, 3),
                'target_v_perp_road_mps': round(t_v_perp, 3),
                'target_sideslip_beta_deg': round(t_beta, 2),
                'target_road_angle_theta_deg': round(t_theta, 2),
                'target_omega_x_deg_s': round(t_ang_vel.x, 2),
                'target_omega_y_deg_s': round(t_ang_vel.y, 2),
                'target_omega_z_deg_s': round(t_ang_vel.z, 2),
                'target_ax': round(t_acc.x, 3),
                'target_ay': round(t_acc.y, 3),
                'target_az': round(t_acc.z, 3),
                'target_steer': round(t_steer, 4),
                'target_throttle': round(t_throttle, 4),
                'target_brake': round(t_brake, 4),
                'target_in_ditch': bool(t_loc.y >= 9.75),
            }
            self.frame_buffer.append(frame_data)
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Метод граничных условий на бровке кювета Y = 9.75 м (Подкласс B)
    # --------------------------------------------------------------------------
    def check_ditch_boundary_conditions(self, actor_name: str, vehicle, lane_right, snapshot_frame: int):
        """
        Проверка пересечения бровки откоса кювета (Y >= 9.75 м) и фиксация граничных условий:
        - V_dep: полная горизонтальная скорость вылета (м/с).
        - V_perp_road: нормальная скорость ухода в кювет (м/с).
        - theta_enc: угол траектории вылета (градусы).
        - V_lat_car: поперечная скорость бокового скольжения колес (м/с).
        - beta: угол бокового заноса кузова (Sideslip Angle, градусы).
        - omega_z: угловая скорость рыскания кузова (Yaw Rate, град/с).
        - Классификация тяжести S0-S3 строго на бровке без внутрикюветной оценки крена/ASI.
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

            # 7. Классификация механизма и тяжести по чистой функции матрицы граничных условий
            actor_sev, mechanism = classify_ditch_boundary(
                v_dep=v_dep,
                v_perp_road=v_perp_road,
                v_lat_car=v_lat_car,
                beta=beta,
                omega_z=omega_z,
            )

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

    def _apply_post_impact_control(
        self,
        actor_name: str,
        vehicle,
        snap,
        vel,
        loc,
        impact_steer: float,
        impact_throttle: float,
        current_steer: float,
        current_brake: float,
        reaction_sec: float,
        hold_sec: float,
        time_since_impact: float,
    ) -> tuple[float, float]:
        """Применение пост-ударного управления к одному из ТС (Ego или Target)."""
        if vehicle is None:
            return current_steer, current_brake

        in_ditch_or_barrier = (
            (loc.y >= 9.75 or actor_name in self.ditch_metrics_by_actor)
            or (loc.y <= 1.05 or actor_name in self.barrier_hit_actors)
        )
        tf = snap.get_transform()
        yaw_deg = tf.rotation.yaw
        fw = tf.get_forward_vector()
        rt = tf.get_right_vector()
        v_lon_car = abs((vel.x * fw.x) + (vel.y * fw.y))
        v_lat_car = abs((vel.x * rt.x) + (vel.y * rt.y))
        speed_xy = math.hypot(vel.x, vel.y)
        beta_deg = math.degrees(math.atan2(v_lat_car, max(v_lon_car, 0.01))) if speed_xy >= 1.0 else 0.0
        thr, new_steer, new_brake = compute_post_impact_control(
            actor_name=actor_name,
            subclass=self.subclass,
            driver_post_shock=self.driver_post_shock,
            time_since_impact=time_since_impact,
            dt=self.dt,
            loc_y=loc.y,
            yaw_deg=yaw_deg,
            vy=vel.y,
            impact_steer=impact_steer,
            impact_throttle=impact_throttle,
            current_steer=current_steer,
            current_brake=current_brake,
            reaction_sec=reaction_sec,
            hold_sec=hold_sec,
            in_ditch_or_barrier=in_ditch_or_barrier,
            beta_deg=beta_deg,
        )
        vehicle.apply_control(carla.VehicleControl(
            throttle=float(thr),
            steer=float(new_steer),
            brake=float(new_brake),
            hand_brake=False,
            manual_gear_shift=False
        ))
        return new_steer, new_brake

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
                    role_name = v.attributes.get('role_name', '').lower()
                    type_str = v.type_id.lower()
                    if role_name == 'ego':
                        self.ego_vehicle = v
                    elif role_name == 'target':
                        self.target_vehicle = v
                    elif 'impala' in type_str and self.ego_vehicle is None:
                        self.ego_vehicle = v
                    elif 'leon' in type_str and self.target_vehicle is None:
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

                    # Назначение физических масс, Config F FrictionScale, EBD и sweep-коллайдеров колес
                    phys_ego = self.ego_vehicle.get_physics_control()
                    phys_ego.mass = EGO_MASS
                    phys_ego.use_sweep_wheel_collision = True
                    ego_wheels = phys_ego.wheels
                    for idx, w in enumerate(ego_wheels):
                        w.tire_friction = ASPHALT_TIRE_FRICTION_SCALE
                        w.max_brake_torque = EGO_FRONT_BRAKE_TORQUE_NM if idx < 2 else EGO_REAR_BRAKE_TORQUE_NM
                    phys_ego.wheels = ego_wheels
                    self.ego_vehicle.apply_physics_control(phys_ego)

                    phys_target = self.target_vehicle.get_physics_control()
                    phys_target.mass = self.target_mass
                    phys_target.use_sweep_wheel_collision = True
                    target_wheels = phys_target.wheels
                    for idx, w in enumerate(target_wheels):
                        w.tire_friction = ASPHALT_TIRE_FRICTION_SCALE
                        w.max_brake_torque = float(
                            (TARGET_FRONT_BRAKE_NM_PER_KG if idx < 2 else TARGET_REAR_BRAKE_NM_PER_KG) * self.target_mass
                        )
                    phys_target.wheels = target_wheels
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

                    # ----------------------------------------------------------
                    # Зона пониженного FrictionScale (обочина + кювет) через static.trigger.friction
                    # Левая граница сдвинута до Y = 8.50 м (компенсация ~1.0 м полуширины кузова
                    # для срабатывания при реальном выходе колес за Y >= 7.50 м без срабатывания на асфальте);
                    # правая граница расширена до Y = 32.00 м (без ложного EndOverlap в кювете до Y = 30.0 м).
                    # Центр Y = 20.25 м, полуширина 11.75 м (extent_y = 1175.0 см).
                    # ----------------------------------------------------------
                    try:
                        bp_friction = self.world.get_blueprint_library().find('static.trigger.friction')
                        bp_friction.set_attribute('friction', str(GRAVEL_TIRE_FRICTION_SCALE))
                        bp_friction.set_attribute('extent_x', '100000.0')
                        bp_friction.set_attribute('extent_y', '1175.0')
                        bp_friction.set_attribute('extent_z', '200.0')
                        friction_transform = carla.Transform(
                            carla.Location(x=float(ego_loc.x + 500.0), y=20.25, z=0.0)
                        )
                        self.friction_trigger = self.world.spawn_actor(bp_friction, friction_transform)
                    except Exception as e:
                        print(f"[ПРЕДУПРЕЖДЕНИЕ] Не удалось заспавнить static.trigger.friction: {e}")
                        self.friction_trigger = None

                    self.configured = True
                    try:
                        self.spectator = self.world.get_spectator()
                    except Exception:
                        self.spectator = None

                    print(f"[{snapshot.frame}] Актеры сконфигурированы: "
                          f"Ego ({self.ego_vehicle.type_id}, {EGO_MASS:.1f} кг, Lane {self.start_lane_id}) | "
                          f"Target ({self.target_vehicle.type_id}, {self.target_mass:.1f} кг) | "
                          f"Подкласс: {self.subclass} | "
                          f"Пост-ударный режим: {self.driver_post_shock}")

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
                # ПОКАДРОВОЕ ЛОГИРОВАНИЕ ТЕЛЕМЕТРИИ (EGO + TARGET)
                # --------------------------------------------------------------
                if self.log_frames:
                    self._record_telemetry_frame(snapshot, ego_snap, target_snap, lane_forward, lane_right)

                # --------------------------------------------------------------
                # МОНИТОРИНГ БРОВКИ КЮВЕТА Y = 9.75 М ДЛЯ ОБОИХ ТС (Подклассы A и B)
                # --------------------------------------------------------------
                self.check_ditch_boundary_conditions('ego', self.ego_vehicle, lane_right, snapshot.frame)
                self.check_ditch_boundary_conditions('target', self.target_vehicle, lane_right, snapshot.frame)

                # Безопасное гашение скорости за пределами рельефа карты (Y > 28.0 м или Z < -2.0 м, за кюветом Y <= 20.85 м)
                for veh in [self.ego_vehicle, self.target_vehicle]:
                    if veh is not None:
                        try:
                            vloc = veh.get_location()
                            if vloc.y > 28.0 or vloc.z < -2.0:
                                veh.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                                veh.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                        except Exception:
                            pass

                # Кэширование актуальных педали газа и угла руля обоих ТС до момента столкновения
                if not self.lost_control:
                    try:
                        e_ctrl = self.ego_vehicle.get_control()
                        self.last_steer = e_ctrl.steer
                        self.last_throttle = e_ctrl.throttle
                    except Exception:
                        pass
                    try:
                        t_ctrl = self.target_vehicle.get_control()
                        self.last_target_steer = t_ctrl.steer
                        self.last_target_throttle = t_ctrl.throttle
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
                            self.ego_tm_speed = max(10.0, min(self.ego_tm_speed, 28.8))
                            self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    is_speed_ready = ego_mps >= (CRUISE_SPEED_MPS - 0.15)
                    is_speed_sync = abs(ego_mps - target_mps) <= 0.15
                    is_dist_sync = abs(error_x) <= 0.15

                    # Проверка условий стабильности по непрерывному физическому времени (1.0 с)
                    if is_speed_ready and is_speed_sync and is_dist_sync:
                        self.stability_time_sec += self.dt
                    else:
                        self.stability_time_sec = max(0.0, self.stability_time_sec - 2.0 * self.dt)

                    # Запасной триггер после 8.0 секунд стабильного движения
                    sim_time = self.tick_count * self.dt
                    forced_trigger = (sim_time >= 8.0 and ego_mps >= (CRUISE_SPEED_MPS - 0.25) and abs(error_x) <= 0.18)

                    if self.stability_time_sec >= 1.0 or forced_trigger:
                        self.lane_change_triggered = True
                        self.ego_vehicle.set_autopilot(False)
                        self.tm.set_desired_speed(self.target_vehicle, float(target_desired_mps * 3.6))
                        print(f"\n[{snapshot.frame}] >>> МАНЕВР ИНИЦИИРОВАН ({self.subclass}): "
                              f"Ego начинает перестроение в слепую зону <<<")

                # --------------------------------------------------------------
                # ФАЗА 2: Опасное перестроение и пост-ударное управление
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
                                self.last_throttle = control.throttle
                                self.ego_vehicle.apply_control(control)

                    else:
                        # ПОСЛЕ УДАРА: единый пост-ударный контроллер для Ego и Target
                        ticks_since_impact = self.tick_count - self.collision_tick
                        time_since_impact = ticks_since_impact * self.dt

                        self.current_steer, self.ego_brake_cmd = self._apply_post_impact_control(
                            actor_name='ego',
                            vehicle=self.ego_vehicle,
                            snap=ego_snap,
                            vel=ego_v,
                            loc=ego_loc,
                            impact_steer=self.impact_steer,
                            impact_throttle=self.impact_throttle,
                            current_steer=self.current_steer,
                            current_brake=self.ego_brake_cmd,
                            reaction_sec=self.ego_shock_duration_sec,
                            hold_sec=self.ego_spasm_duration_sec,
                            time_since_impact=time_since_impact,
                        )

                        self.target_current_steer, self.target_brake_cmd = self._apply_post_impact_control(
                            actor_name='target',
                            vehicle=self.target_vehicle,
                            snap=target_snap,
                            vel=target_v,
                            loc=target_loc,
                            impact_steer=self.target_impact_steer,
                            impact_throttle=self.target_impact_throttle,
                            current_steer=self.target_current_steer,
                            current_brake=self.target_brake_cmd,
                            reaction_sec=self.target_shock_duration_sec,
                            hold_sec=self.target_spasm_duration_sec,
                            time_since_impact=time_since_impact,
                        )

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
    # Извлечение итоговых метрик по кювету (Подкласс B)
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
        worst_tuple = (-1, -1.0, -1.0)

        for actor, data in self.ditch_metrics_by_actor.items():
            sev = data['actor_sev']
            hazard_score = max(data['v_lat_car'] / 5.5, data['v_perp_road'] / 12.0)
            cand_tuple = (sev, hazard_score, data['v_dep'])
            if cand_tuple > worst_tuple:
                worst_tuple = cand_tuple
                worst_actor = actor

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

        # Обязательное уничтожение физического триггера трения обочины
        if self.friction_trigger is not None:
            try:
                if self.friction_trigger.is_alive:
                    self.friction_trigger.destroy()
            except Exception:
                pass
            self.friction_trigger = None

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
    - Компилирует сценарии Scenic для Подклассов А (ограждение) и B (кювет).
    - Выполняет физические прогоны с сохранением кинематики в 25-колоночный CSV.
    - НЕ содержит логики статистической агрегации кампании (>10%).
    """
    parser = argparse.ArgumentParser(description="HARA LCC Blind Spot Simulation Campaign (Physical Runner)")
    parser.add_argument('--subclass', type=str, default='ALL', choices=['A', 'B', 'ALL'],
                        help="Road subclass: 'A' (Barrier), 'B' (Ditch), or 'ALL' (Campaign)")
    parser.add_argument('--driver-post-shock', type=str, default='passive',
                        choices=['passive', 'active'],
                        help="Driver behavior mode after shock phase: 'passive' (default, coasting for pure Severity S), or 'active' (counter-steering & braking for Controllability C)")
    parser.add_argument('--mode', type=str, default='equal', choices=['equal', 'sequential', 'weighted'],
                        help="Campaign mode: 'equal' (50/50: N/2 of A, N/2 of B), 'sequential' (0.2 A / 0.8 B), or 'weighted'")
    parser.add_argument('--runs', type=int, default=20, help="Total number of simulation runs (default: 20)")
    parser.add_argument('--map-dir', type=str, default=DEFAULT_MAPS_DIR,
                        help="Path to directory containing map assets (default: maps/assets)")
    parser.add_argument('--output', type=str, default=None,
                        help="Custom output CSV file path (default: results/lcc_hara_results.csv)")
    parser.add_argument('--clear', action='store_true', help="Clear/overwrite previous CSV results before running")
    parser.add_argument('--log-frames', action='store_true',
                        help="Записывать подробную покадровую телеметрию обоих автомобилей для каждого прогона в отдельный файл.")
    parser.add_argument('--render', action='store_true', help="Enable Carla rendering window")
    parser.add_argument('--timestep', type=float, default=0.01,
                        help="Simulation timestep dt in seconds (default: 0.01, recommended: 0.005 - 0.025)")
    parser.add_argument('--seed', type=int, default=42,
                        help="Fixed random seed for deterministic Scenic scene generation and reproducibility (default: 42)")
    args = parser.parse_args()

    output_csv_path = os.path.abspath(args.output) if args.output else CSV_PATH
    output_csv_dir = os.path.dirname(output_csv_path)
    if output_csv_dir:
        os.makedirs(output_csv_dir, exist_ok=True)

    # Фиксация случайного зерна для воспроизводимости стохастических сцен
    if args.seed is not None:
        random.seed(args.seed)
        print(f"  [РЕПРОДУЦИРУЕМОСТЬ] Установлен фиксированный random seed: {args.seed}")

    # Проверка диапазона шага симуляции dt
    if args.timestep < 0.005 or args.timestep > 0.025:
        print(f"  [ПРЕДУПРЕЖДЕНИЕ] Шаг симуляции dt={args.timestep}c находится вне рекомендованного диапазона [0.005..0.025] с. Возможна нестабильность интегратора PhysX или избыточное время моделирования.")

    print("=" * 80)
    print(" HARA SIMULATION CAMPAIGN: LANE CHANGE COLLISION (LCC) BLIND SPOT FAILURE")
    print(" ISO 26262-3 / SAE J2980:2023 (Dual Collision Sensors & Ditch Boundary Conditions)")
    print(f" Post-Shock Driver Mode: '{args.driver_post_shock}' (Default: passive)")
    print(f" Output CSV: {output_csv_path}")
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
        target_carla_map = None if CARLA_MAP_NAME in world.get_map().name else CARLA_MAP_NAME
        simulator = CarlaSimulator(
            carla_map=target_carla_map,
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
        scenarios[SUBCLASS_B] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_B, 'map': map_xodr_path, 'timestep': args.timestep}
        )
        print("  [OK] Сценарии для Подкласса А (Ограждение) и Подкласса B (Кювет) скомпилированы.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой компиляции Scenic: {e}")
        sys.exit(1)

    # 4. Формирование плана кампании
    campaign_plan = []
    if args.subclass == 'A':
        campaign_plan = [SUBCLASS_A] * args.runs
    elif args.subclass == 'B':
        campaign_plan = [SUBCLASS_B] * args.runs
    else:
        if args.mode == 'equal':
            runs_a = args.runs // 2
            runs_b = args.runs - runs_a
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_B] * runs_b
        elif args.mode == 'sequential':
            runs_a = max(1, int(round(args.runs * 0.20)))
            runs_b = max(1, args.runs - runs_a)
            campaign_plan = [SUBCLASS_A] * runs_a + [SUBCLASS_B] * runs_b
        else:
            campaign_plan = random.choices([SUBCLASS_A, SUBCLASS_B], weights=[0.20, 0.80], k=args.runs)

    print(f"\n[4/4] Запуск кампании: всего {len(campaign_plan)} прогонов "
          f"({campaign_plan.count(SUBCLASS_A)} Подкласс А, {campaign_plan.count(SUBCLASS_B)} Подкласс B)")

    # Подготовка выходного CSV-файла со строгой 25-колоночной схемой
    if args.clear and os.path.isfile(output_csv_path):
        try:
            os.remove(output_csv_path)
        except Exception:
            pass

    file_exists = os.path.isfile(output_csv_path)

    # Проверка совместимости колонок в существующем файле
    if file_exists and not args.clear:
        try:
            with open(output_csv_path, 'r', encoding='utf-8-sig') as f_check:
                header_line = f_check.readline()
                if 'ditch_mechanism' not in header_line or 'sev_side' not in header_line:
                    backup_csv = output_csv_path + ".legacy_backup.csv"
                    os.replace(output_csv_path, backup_csv)
                    print(f"  [ИНФО] Старый CSV сохранен в резервную копию: {backup_csv}")
                    file_exists = False
        except Exception:
            pass

    # Подготовка папки для покадровых логов и проверка дискового пространства
    if args.log_frames:
        try:
            free_bytes = shutil.disk_usage(RESULTS_DIR).free
            if free_bytes < MIN_DISK_FREE_BYTES:
                print(f"  [ПРЕДУПРЕЖДЕНИЕ] Свободное место на диске ({free_bytes / (1024**3):.2f} ГБ) меньше лимита 1.5 ГБ! Покадровое логирование отключено.")
                args.log_frames = False
            else:
                os.makedirs(FRAME_LOGS_DIR, exist_ok=True)
                if args.clear:
                    for f in os.listdir(FRAME_LOGS_DIR):
                        f_path = os.path.join(FRAME_LOGS_DIR, f)
                        if os.path.isfile(f_path):
                            try:
                                os.remove(f_path)
                            except Exception:
                                pass
                    print(f"  [ИНФО] Папка покадровых логов очищена: {FRAME_LOGS_DIR}")
        except Exception as e:
            print(f"  [ПРЕДУПРЕЖДЕНИЕ] Ошибка подготовки папки логов: {e}")
            args.log_frames = False

    with open(output_csv_path, mode='w' if args.clear or not file_exists else 'a', newline='', encoding='utf-8-sig') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        if not file_exists or args.clear:
            writer.writeheader()

        # ----------------------------------------------------------------------
        # Цикл прогонов HARA кампании
        # ----------------------------------------------------------------------
        for run_idx, subclass in enumerate(campaign_plan, start=1):
            # Генерация стохастической сцены Scenic
            scenario = scenarios[subclass]
            scene, _ = scenario.generate()
            target_mass = float(scene.params.get('target_mass_kg', TARGET_MASS_DEFAULT))

            subclass_label = "Подкласс А (Ограждение)" if subclass == SUBCLASS_A else "Подкласс B (Кювет)"
            print("\n" + "=" * 80)
            print(f" ПРОГОН [{run_idx}/{len(campaign_plan)}] -> {subclass_label}")
            print(f" Режим водителя после реакции: {args.driver_post_shock} | Масса Target (Scenic): {target_mass:.1f} кг")
            print("=" * 80)

            # Создание и запуск менеджера симуляции
            manager = AutopilotManager(
                client,
                world,
                subclass=subclass,
                driver_post_shock=args.driver_post_shock,
                dt=args.timestep,
                log_frames=args.log_frames,
                target_mass=target_mass,
            )
            manager.start()

            spawn_error_msg = None
            try:
                # Запуск симуляции Scenic (таймаут 45 секунд физического времени)
                max_steps = int(round(45.0 / args.timestep))
                simulator.simulate(scene, maxSteps=max_steps)
            except Exception as e:
                err_str = str(e).strip()
                exc_name = type(e).__name__
                if "Unable to spawn" in err_str or "SimulationCreationError" in exc_name or not manager.configured:
                    spawn_error_msg = err_str if "SimulationCreationError" in err_str else f"{exc_name}: {err_str}"
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
                if spawn_error_msg is not None:
                    outcome = f"Ошибка спавна: {spawn_error_msg}"
                else:
                    outcome = "Без столкновений"
                    if subclass == SUBCLASS_A:
                        b_type = manager.barrier_impact_type
                        actor_desc = f" ({barrier_actor})" if barrier_actor != 'None' else ""
                        if has_side_hit and has_barrier_hit:
                            outcome = f"Боковой удар с целевым ТС + {b_type} удар в ограждение{actor_desc}"
                        elif has_barrier_hit:
                            outcome = f"{b_type} удар в ограждение{actor_desc} (без удара целевого ТС)"
                        elif has_side_hit:
                            outcome = "Боковой удар с целевым ТС (Без удара в ограждение)"
                        if ditch_actor != 'None':
                            outcome = f"{outcome} + {mechanism} ({ditch_actor})"
                    else:  # SUBCLASS_B
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

                # --------------------------------------------------------------
                # Сохранение покадрового лога телеметрии (если включено)
                # --------------------------------------------------------------
                if args.log_frames and manager.frame_buffer:
                    log_filename = os.path.join(FRAME_LOGS_DIR, f"run_{run_idx:04d}_{subclass}.csv")
                    try:
                        with open(log_filename, mode='w', newline='', encoding='utf-8') as f_log:
                            log_writer = csv.DictWriter(f_log, fieldnames=FRAME_LOG_FIELDNAMES)
                            log_writer.writeheader()
                            log_writer.writerows(manager.frame_buffer)
                    except Exception as err:
                        print(f"  [ПРЕДУПРЕЖДЕНИЕ] Ошибка записи покадрового лога {log_filename}: {err}")

                    # Динамический контроль дискового пространства (Лимит 1 ГБ)
                    try:
                        current_logs_size = sum(
                            os.path.getsize(os.path.join(FRAME_LOGS_DIR, f))
                            for f in os.listdir(FRAME_LOGS_DIR)
                            if os.path.isfile(os.path.join(FRAME_LOGS_DIR, f))
                        )
                        current_free = shutil.disk_usage(RESULTS_DIR).free

                        if current_logs_size > MAX_FRAME_LOGS_BYTES or current_free < MIN_DISK_FREE_BYTES:
                            print(f"\n  [ПРЕДУПРЕЖДЕНИЕ] Превышен лимит размера покадровых логов "
                                  f"({current_logs_size / (1024**2):.1f} МБ > 1024 МБ) или порог диска "
                                  f"({current_free / (1024**3):.2f} ГБ < 1.5 ГБ)!")
                            print("  [ПРЕДУПРЕЖДЕНИЕ] Удаление избыточного файла и отключение покадрового логирования для оставшихся прогонов.")
                            if os.path.isfile(log_filename):
                                try:
                                    os.remove(log_filename)
                                except Exception:
                                    pass
                            args.log_frames = False
                    except Exception:
                        pass

                # Сводный вывод в консоль
                print("\n" + "-" * 80)
                print(f"ИТОГ ПРОГОНА [{run_idx}]:")
                print(f"  * Подкласс: {subclass}")
                print(f"  * Масса Target (Scenic): {target_mass:.1f} кг")
                print(f"  * Режим водителя после реакции: {manager.driver_post_shock}")
                print(f"  * Время реакции водителя: {manager.shock_duration_sec:.3f} с ({manager.shock_duration_ticks} тиков)")
                print(f"  * Удар ТС-ТС: {has_side_hit} " +
                      (f"(Delta-V Ego: {manager.delta_v_ego:.2f} м/с, Target: {manager.delta_v_target:.2f} м/с, S_side: S{manager.sev_side_val})" if has_side_hit else ""))
                if subclass == SUBCLASS_A:
                    print(f"  * Удар в ограждение: {has_barrier_hit} " +
                          (f"({manager.barrier_impact_type}, {manager.barrier_impact_speed:.2f} м/с, актер: {barrier_actor}, S_barrier: S{manager.sev_barrier_val})" if has_barrier_hit else ""))
                    if ditch_actor != 'None':
                        print(f"  * Перекрёстный вылет в кювет: {ditch_actor} "
                              f"(Скорость: {dep_speed:.2f} м/с, Угол beta: {sideslip:.1f}°, Механизм: '{mechanism}', S_ditch: S{sev_ditch_val})")
                else:
                    print(f"  * Вылет в кювет: {ditch_actor} " +
                          (f"(Скорость: {dep_speed:.2f} м/с, Угол beta: {sideslip:.1f}°, Механизм: '{mechanism}', S_ditch: S{sev_ditch_val})" if ditch_actor != 'None' else ""))
                print(f"  * Итоговая тяжесть прогона: >>> {severity} <<<")
                print(f"  * Исход: {outcome}")
                print("-" * 80)

                time.sleep(0.3)

    print("\n" + "=" * 80)
    print("КАМПАНИЯ СИМУЛЯЦИЙ ЗАВЕРШЕНА!")
    print(f"Результаты записаны в: {output_csv_path}")
    print("Для статистического анализа по правилу >10% ISO 26262-3 запустите:")
    print(f"  python analyze_results.py --input {output_csv_path}")
    print("=" * 80)


if __name__ == '__main__':
    main()
