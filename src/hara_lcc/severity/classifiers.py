"""
Чистые физико-кинематические классификаторы тяжести S0..S3 (ISO 26262-3 / SAE J2980:2023).
Используются одновременно в онлайн-симуляции (simulation) и оффлайн-аналитике (analysis)
без дублирования формул и порогов.
"""

from __future__ import annotations

from dataclasses import dataclass

from hara_lcc.config import (
    BARRIER_FRONTAL_COS_THRESHOLD,
    BARRIER_FRONTAL_S1_MPS,
    BARRIER_FRONTAL_S2_MPS,
    BARRIER_FRONTAL_S3_MPS,
    BARRIER_SIDESWIPE_S1_MPS,
    BARRIER_SIDESWIPE_S2_MPS,
    BARRIER_SIDESWIPE_S3_MPS,
    DITCH_NON_TRACKING_BETA_DEG,
    DITCH_S1_TRACKING_VPERP_MPS,
    DITCH_S2_BOTTOM_VPERP_MPS,
    DITCH_S2_SKID_BETA_DEG,
    DITCH_S2_SKID_VDEP_MPS,
    DITCH_S2_TRIP_VDEP_MPS,
    DITCH_S2_TRIP_VLAT_MPS,
    DITCH_S3_NOSE_VPERP_MPS,
    DITCH_S3_SKID_BETA_DEG,
    DITCH_S3_SKID_VDEP_MPS,
    DITCH_S3_SPIN_BETA_DEG,
    DITCH_S3_SPIN_OMEGA_Z_DEG_S,
    DITCH_VDEP_TRACKING_MIN_MPS,
    EGO_MASS,
    RESTITUTION_E,
    SIDE_DV_S1_MPS,
    SIDE_DV_S2_MPS,
    SIDE_DV_S3_MPS,
    TARGET_MASS_DEFAULT,
)


@dataclass(frozen=True)
class SideCollisionMetrics:
    """Результат расчёта первичного бокового столкновения ТС–ТС по закону сохранения импульса."""

    v_closing_lat: float
    delta_v_ego: float
    delta_v_target: float
    effective_side_dv: float
    sev_side: int


@dataclass(frozen=True)
class BarrierImpactMetrics:
    """Результат расчёта вторичного удара о жёсткое левое ограждение."""

    v_barrier: float
    cos_alpha: float
    impact_type: str
    sev_barrier: int


def classify_side_severity(effective_side_dv: float) -> int:
    """Калиброванная шкала SAE J2980:2023 Table B1 для первичного бокового удара ТС–ТС."""
    if effective_side_dv > SIDE_DV_S3_MPS:
        return 3
    if effective_side_dv > SIDE_DV_S2_MPS:
        return 2
    if effective_side_dv > SIDE_DV_S1_MPS:
        return 1
    return 0


def compute_side_collision_metrics(
    v_closing_lat: float,
    m_ego: float = EGO_MASS,
    m_target: float = TARGET_MASS_DEFAULT,
    restitution: float = RESTITUTION_E,
) -> SideCollisionMetrics:
    """
    Вычисление переданных Delta-V при первичном боковом контакте ТС–ТС по закону сохранения импульса
    с фактической массой Target и коэффициентом восстановления e = 0.15.
    """
    v_lat = abs(float(v_closing_lat))
    m_e = float(m_ego)
    m_t = float(m_target)
    e = float(restitution)

    delta_v_ego = (m_t / (m_e + m_t)) * (1.0 + e) * v_lat
    delta_v_target = (m_e / (m_e + m_t)) * (1.0 + e) * v_lat
    effective_side_dv = max(delta_v_ego, delta_v_target)
    sev_side = classify_side_severity(effective_side_dv)

    return SideCollisionMetrics(
        v_closing_lat=v_lat,
        delta_v_ego=delta_v_ego,
        delta_v_target=delta_v_target,
        effective_side_dv=effective_side_dv,
        sev_side=sev_side,
    )


def classify_barrier_severity(v_barrier: float, impact_type: str) -> int:
    """
    Классификация вторичного удара об ограждение с учётом угла атаки:
    - Frontal/Rear (> 45°): 6 / 11 / 18 м/с;
    - Side-swipe (<= 45°):  2 / 6 / 12 м/с.
    """
    itype_lower = str(impact_type).lower()
    if 'frontal' in itype_lower or 'rear' in itype_lower:
        if v_barrier > BARRIER_FRONTAL_S3_MPS:
            return 3
        if v_barrier > BARRIER_FRONTAL_S2_MPS:
            return 2
        if v_barrier > BARRIER_FRONTAL_S1_MPS:
            return 1
        return 0

    if v_barrier > BARRIER_SIDESWIPE_S3_MPS:
        return 3
    if v_barrier > BARRIER_SIDESWIPE_S2_MPS:
        return 2
    if v_barrier > BARRIER_SIDESWIPE_S1_MPS:
        return 1
    return 0


def compute_barrier_impact(
    v_actor_x: float,
    v_actor_y: float,
    fw_actor_x: float,
    fw_actor_y: float,
    lane_forward_x: float,
    lane_forward_y: float,
    lane_right_x: float,
    lane_right_y: float,
) -> BarrierImpactMetrics:
    """
    Расчёт нормальной скорости впечатывания строго ВНУТРЬ левого ограждения
    (отскок от барьера >= 0 обнуляется) и угла атаки относительно оси дороги.
    """
    v_inward_lr = (v_actor_x * lane_right_x) + (v_actor_y * lane_right_y)
    v_barrier = max(0.0, -v_inward_lr)
    cos_alpha = (fw_actor_x * lane_forward_x) + (fw_actor_y * lane_forward_y)

    if abs(cos_alpha) < BARRIER_FRONTAL_COS_THRESHOLD:
        impact_type = 'Frontal/Rear'
    else:
        impact_type = 'Side-swipe'

    sev_barrier = classify_barrier_severity(v_barrier, impact_type)
    return BarrierImpactMetrics(
        v_barrier=v_barrier,
        cos_alpha=cos_alpha,
        impact_type=impact_type,
        sev_barrier=sev_barrier,
    )


def classify_ditch_boundary(
    v_dep: float,
    v_perp_road: float | None,
    v_lat_car: float | None,
    beta: float | None,
    omega_z: float | None,
) -> tuple[int, str]:
    """
    Чистый классификатор тяжести и механизма схода в кювет строго на бровке Y = 9.75 м (Вариант B).
    Возвращает (actor_sev [0..3], mechanism_name).
    Безопасно обрабатывает None для совместимости с неполными/легаси CSV без изменения логики для float.
    """
    is_non_tracking = (beta is not None and beta >= DITCH_NON_TRACKING_BETA_DEG)

    # S3: Критические / фатальные травмы (MAIS 5-6 > 10%, широкоугольный срыв beta >= 25° / быстрый спин или жёсткий упор носом)
    is_s3_skid = (
        (v_dep > DITCH_S3_SKID_VDEP_MPS)
        and (beta is not None)
        and (
            (beta >= DITCH_S3_SKID_BETA_DEG)
            or (beta >= DITCH_S3_SPIN_BETA_DEG and omega_z is not None and omega_z > DITCH_S3_SPIN_OMEGA_Z_DEG_S)
        )
    )
    is_s3_nose = (v_dep > DITCH_VDEP_TRACKING_MIN_MPS) and (v_perp_road is not None and v_perp_road > DITCH_S3_NOSE_VPERP_MPS)

    # S2: Тяжёлые травмы (MAIS 3-6 > 10%, инициация soil-trip Cooperrider V_lat > 5.5 м/с при beta >= 15°,
    # выраженный non-tracking срыв NCHRP 665 beta >= 20° или удар о дно кювета V_perp > 8.0 м/с)
    is_s2_trip = (v_dep > DITCH_S2_TRIP_VDEP_MPS) and is_non_tracking and (v_lat_car is not None and v_lat_car > DITCH_S2_TRIP_VLAT_MPS)
    is_s2_skid = (v_dep > DITCH_S2_SKID_VDEP_MPS) and (beta is not None and beta >= DITCH_S2_SKID_BETA_DEG)
    is_s2_bottom = (v_dep > DITCH_VDEP_TRACKING_MIN_MPS) and (v_perp_road is not None and v_perp_road > DITCH_S2_BOTTOM_VPERP_MPS)

    # S1: Лёгкие и средние травмы (Alberta RDG Table HA.4 SI=1.8 / FHWA-HRT-21-045: проходимый съезд или занос beta >= 15°)
    is_s1_tracking = (v_dep > DITCH_VDEP_TRACKING_MIN_MPS and (v_perp_road is not None and v_perp_road > DITCH_S1_TRACKING_VPERP_MPS))
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


def select_worst_ditch_metrics(ditch_metrics_by_actor: dict) -> tuple:
    """
    Выбор наихудшего исхода среди съехавших за бровку Y = 9.75 м ТС (Ego и/или Target).
    Возвращает кортеж:
    (sev_ditch_val, ditch_departure_actor, dep_speed, road_perp_speed,
     enc_angle, lat_skid, sideslip, yaw_rate, mechanism)
    """
    if not ditch_metrics_by_actor:
        return 0, 'None', '', '', '', '', '', '', ''

    if 'ego' in ditch_metrics_by_actor and 'target' in ditch_metrics_by_actor:
        ditch_actor_str = 'both'
    elif 'ego' in ditch_metrics_by_actor:
        ditch_actor_str = 'ego'
    else:
        ditch_actor_str = 'target'

    worst_actor = None
    worst_tuple = (-1, -1.0, -1.0)

    for actor, data in ditch_metrics_by_actor.items():
        sev = data['actor_sev']
        hazard_score = max(data['v_lat_car'] / DITCH_S2_TRIP_VLAT_MPS, data['v_perp_road'] / DITCH_S3_NOSE_VPERP_MPS)
        cand_tuple = (sev, hazard_score, data['v_dep'])
        if cand_tuple > worst_tuple:
            worst_tuple = cand_tuple
            worst_actor = actor

    best_data = ditch_metrics_by_actor[worst_actor]

    return (
        best_data['actor_sev'],
        ditch_actor_str,
        round(best_data['v_dep'], 2),
        round(best_data['v_perp_road'], 2),
        round(best_data['theta_enc'], 1),
        round(best_data['v_lat_car'], 2),
        round(best_data['beta'], 1),
        round(best_data['omega_z'], 1),
        best_data['mechanism'],
    )
