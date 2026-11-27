"""
Single Source of Truth для физических констант, порогов Severity S0-S3,
параметров Config F, геометрии трассы/кювета, модели водителя, схем CSV и путей проекта.
"""

from __future__ import annotations

import os

# ------------------------------------------------------------------------------
# Пути проекта и симулятора
# ------------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BASE_DIR = PROJECT_ROOT
SCENIC_PATH = os.path.join(PROJECT_ROOT, "LCC_blindSpot.scenic")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
CSV_PATH = os.path.join(RESULTS_DIR, "lcc_hara_results.csv")
FRAME_LOGS_DIR = os.path.join(RESULTS_DIR, "frame_logs")

DEFAULT_MAPS_DIR = os.path.join(PROJECT_ROOT, "maps", "assets")
CANONICAL_XODR_FILENAME = "map_ditch.xodr"
CARLA_MAP_NAME = "map_ditch"
DEFAULT_TEMPLATE_PATH = os.path.join(PROJECT_ROOT, "templates", "report_template.md")

CARLA_API_PATH = r"C:\CARLA\PythonAPI\carla"

# Лимиты дискового пространства для покадровых логов
MAX_FRAME_LOGS_BYTES = 1024 * 1024 * 1024  # 1 ГБ
MIN_DISK_FREE_BYTES = int(1.5 * 1024 * 1024 * 1024)  # 1.5 ГБ

# ------------------------------------------------------------------------------
# Физические параметры участников сценария и Config F (CARLA / PhysX)
# ------------------------------------------------------------------------------
# Ego: vehicle.chevrolet.impala со спец. массой защищаемого ОА (аналог Aurus Senat)
EGO_MASS = 3490.0            # Масса ТС Ego (кг)

# Target: vehicle.seat.leon (стохастическая масса из Scenic scene.params['target_mass_kg'])
TARGET_MASS_DEFAULT = 1480.0  # Резервная масса Target (кг), если параметр Scenic не передан

# Пропорциональный EBD max_brake_torque (Config F: ~65% передняя / ~35% задняя ось)
EGO_FRONT_BRAKE_TORQUE_NM = 3800.0
EGO_REAR_BRAKE_TORQUE_NM = 2050.0
TARGET_FRONT_BRAKE_NM_PER_KG = 1.00
TARGET_REAR_BRAKE_NM_PER_KG = 0.54
USE_SWEEP_WHEEL_COLLISION = True

# Коэффициент восстановления при скользящем боковом контакте (Brach, Brach & Pongetti 2014, SAE 2014-01-0469)
RESTITUTION_E = 0.15

# Безразмерные множители сцепления UE4 UTireConfig::FrictionScale (Config F)
ASPHALT_TIRE_FRICTION_SCALE = 1.05   # Сухой чистый асфальт (~0.81-0.86 g, FMVSS 135 / NCAP)
GRAVEL_TIRE_FRICTION_SCALE = 0.57    # Укрепленная гравийная обочина (~0.46-0.48 g; отношение 0.57 / 1.05 = 0.543)

# Крейсерская скорость потока (100 км/ч)
CRUISE_SPEED_MPS = 100.0 / 3.6  # 27.78 м/с

# Параметры PhysX substepping и горизонта симуляции
DEFAULT_TIMESTEP_SEC = 0.01
PHYSX_MAX_SUBSTEP_DT = 0.01
PHYSX_MAX_SUBSTEPS = 10
SIMULATION_HORIZON_SEC = 45.0

# ------------------------------------------------------------------------------
# Геометрия трассы map_ditch, барьера, обочины и бровки кювета
# ------------------------------------------------------------------------------
LANE_LEFT_CENTER_Y = 1.875    # Центр левой полосы (Lane -1)
LANE_RIGHT_CENTER_Y = 5.625   # Центр правой полосы (Lane -2)
BARRIER_FILTER_MAX_Y = 3.5    # Пространственный фильтр контакта с левым ограждением (Y <= 3.5 м)
BARRIER_HOLD_CUTOFF_Y = 1.05  # Порог принудительного сброса газа у левого ограждения

# Обочина (Y >= 7.50 м) и триггерный бокс static.trigger.friction:
# левая граница бокса сдвинута на Y = 8.50 м (компенсация ~1.0 м полуширины кузова
# для срабатывания при реальном выходе колёс за Y >= 7.50 м);
# правая граница расширена до Y = 32.00 м (исключение ложного EndOverlap в кювете).
SHOULDER_START_Y = 7.50
FRICTION_TRIGGER_BOX_LEFT_Y = 8.50
FRICTION_TRIGGER_BOX_RIGHT_Y = 32.00
FRICTION_TRIGGER_CENTER_Y = 20.25
FRICTION_TRIGGER_EXTENT_X_CM = 100000.0
FRICTION_TRIGGER_EXTENT_Y_CM = 1175.0
FRICTION_TRIGGER_EXTENT_Z_CM = 200.0
FRICTION_TRIGGER_X_OFFSET_M = 500.0

# Бровка откоса кювета (строгий метод граничных условий) и границы рельефа карты
DITCH_BOUNDARY_Y = 9.75
OUT_OF_BOUNDS_Y = 28.0
OUT_OF_BOUNDS_Z = -2.0

# ------------------------------------------------------------------------------
# Идентификаторы дорожных подклассов и веса ODD
# ------------------------------------------------------------------------------
SUBCLASS_A = "SUBCLASS_A_BARRIER"  # Маневр влево: риск удара о жёсткое ограждение (W_A = 0.20)
SUBCLASS_B = "SUBCLASS_B_DITCH"    # Маневр вправо: риск схода в кювет на бровке Y = 9.75 м (W_B = 0.80)
DEFAULT_WEIGHT_A = 0.20
DEFAULT_WEIGHT_B = 0.80

# ------------------------------------------------------------------------------
# Пороги классификации Severity (S0..S3) — ISO 26262-3 / SAE J2980:2023
# ------------------------------------------------------------------------------
# Первичный боковой удар ТС-ТС (Delta-V, м/с)
SIDE_DV_S1_MPS = 1.0
SIDE_DV_S2_MPS = 3.5
SIDE_DV_S3_MPS = 8.0

# Вторичный удар в жёсткое ограждение (V_barrier, м/с)
BARRIER_FRONTAL_COS_THRESHOLD = 0.707
BARRIER_FRONTAL_S1_MPS = 6.0
BARRIER_FRONTAL_S2_MPS = 11.0
BARRIER_FRONTAL_S3_MPS = 18.0
BARRIER_SIDESWIPE_S1_MPS = 2.0
BARRIER_SIDESWIPE_S2_MPS = 6.0
BARRIER_SIDESWIPE_S3_MPS = 12.0

# Граничные условия кювета Variant B на Y = 9.75 м
DITCH_NON_TRACKING_BETA_DEG = 15.0
DITCH_S3_SKID_VDEP_MPS = 18.0
DITCH_S3_SKID_BETA_DEG = 25.0
DITCH_S3_SPIN_BETA_DEG = 20.0
DITCH_S3_SPIN_OMEGA_Z_DEG_S = 45.0
DITCH_VDEP_TRACKING_MIN_MPS = 8.0
DITCH_S3_NOSE_VPERP_MPS = 18.0
DITCH_S2_TRIP_VDEP_MPS = 14.0
DITCH_S2_TRIP_VLAT_MPS = 5.5
DITCH_S2_SKID_VDEP_MPS = 11.0
DITCH_S2_SKID_BETA_DEG = 20.0
DITCH_S2_BOTTOM_VPERP_MPS = 12.0
DITCH_S1_TRACKING_VPERP_MPS = 2.0
DITCH_S1_SKID_VLAT_MPS = 2.0

# Параметры ПИД перестроения и правила Safe Pass (Фаза 2)
PID_MAX_THROTTLE = 1.0
SAFE_PASS_LANE_TOL_M = 0.35
SAFE_PASS_VY_TOL_MPS = 0.35
SAFE_PASS_YAW_TOL_DEG = 2.5
SAFE_PASS_HOLD_SEC = 1.0

# Кумулятивный порог репрезентативности ISO 26262-3 / SAE J2980
ISO_REPRESENTATIVE_THRESHOLD_PCT = 10.0

# ------------------------------------------------------------------------------
# Параметры модели задержки реакции и пост-ударного управления водителя
# (Green 2000, Olson & Sivak 1986, Taoka 1989, SAE J2980 5.2.3.2(d), DOT HS 809 875)
# ------------------------------------------------------------------------------
REACTION_TIME_MEDIAN_SEC = 1.05
REACTION_TIME_LOG_SIGMA = 0.22
REACTION_TIME_MIN_SEC = 0.65
REACTION_TIME_MAX_SEC = 1.60
HOLD_TIME_MIN_SEC = 0.20
HOLD_TIME_MAX_SEC = 0.35
ACTIVE_MAX_STEER_RATE_PER_SEC = 2.5
BRAKE_RISE_TIME_SEC = 0.25

# ------------------------------------------------------------------------------
# Точная схема 25 столбцов CSV-файла результатов HARA
# ------------------------------------------------------------------------------
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
    'scenario_outcome',
]

# Точная схема 61 столбца покадровой телеметрии frame_logs
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
    'target_in_ditch',
]

# ------------------------------------------------------------------------------
# Классы тяжести, матрица ASIL и палитры визуализации
# ------------------------------------------------------------------------------
SEVERITY_CLASSES = ['S0', 'S1', 'S2', 'S3']
SEVERITY_RANK = {'S0': 0, 'S1': 1, 'S2': 2, 'S3': 3}
SEVERITY_FROM_RANK = {0: 'S0', 1: 'S1', 2: 'S2', 3: 'S3'}

SEVERITY_COLORS = {
    'S0': '#2ecc71',  # Зеленый (без травм)
    'S1': '#f1c40f',  # Золотисто-желтый (легкие травмы)
    'S2': '#e67e22',  # Оранжевый (тяжелые травмы)
    'S3': '#e74c3c',  # Красный (критические/фатальные травмы)
}

SUBCLASS_COLORS = {
    SUBCLASS_A: '#2980b9',        # Синий
    SUBCLASS_B: '#8e44ad',        # Фиолетовый
    'ODD_WEIGHTED': '#2c3e50',    # Темно-синий/графитовый
}

SUBCLASS_NAMES = {
    SUBCLASS_A: 'Подкласс А (Ограждение)',
    SUBCLASS_B: 'Подкласс B (Кювет)',
    'ODD_WEIGHTED': 'Взвешенный ODD',
}

ASIL_MATRIX = {
    ('S1', 'E3', 'C3'): 'ASIL A',
    ('S1', 'E4', 'C2'): 'ASIL A',
    ('S1', 'E4', 'C3'): 'ASIL B',
    ('S2', 'E2', 'C3'): 'ASIL A',
    ('S2', 'E3', 'C2'): 'ASIL A',
    ('S2', 'E3', 'C3'): 'ASIL B',
    ('S2', 'E4', 'C1'): 'ASIL A',
    ('S2', 'E4', 'C2'): 'ASIL B',
    ('S2', 'E4', 'C3'): 'ASIL C',
    ('S3', 'E1', 'C3'): 'ASIL A',
    ('S3', 'E2', 'C2'): 'ASIL A',
    ('S3', 'E2', 'C3'): 'ASIL B',
    ('S3', 'E3', 'C1'): 'ASIL A',
    ('S3', 'E3', 'C2'): 'ASIL B',
    ('S3', 'E3', 'C3'): 'ASIL C',
    ('S3', 'E4', 'C1'): 'ASIL B',
    ('S3', 'E4', 'C2'): 'ASIL C',
    ('S3', 'E4', 'C3'): 'ASIL D',
}
