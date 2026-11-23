"""
Изолированный модуль пост-ударного управления ТС (фазы T_hold / T_react, passive и active
контроллер с ограничением скорости руления, линейной рампой торможения и бортовой модуляцией
Kamm + Sideslip ABS без мировых координатных оракулов loc_y).

Архитектурно подготовлен к подключению произвольных профилей водителя для этапа Controllability (C).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from hara_lcc.config import (
    ACTIVE_MAX_STEER_RATE_PER_SEC,
    BRAKE_RISE_TIME_SEC,
    LANE_LEFT_CENTER_Y,
    LANE_RIGHT_CENTER_Y,
    SUBCLASS_A,
)


def normalize_deg(angle: float) -> float:
    """Нормализация углов Эйлера к диапазону [-180, 180] градусов."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


@dataclass(frozen=True)
class PostImpactState:
    """Состояние ТС на текущем тике пост-ударной фазы."""

    actor_name: str
    subclass: str
    time_since_impact: float
    dt: float
    loc_y: float
    yaw_deg: float
    vy: float
    impact_steer: float
    impact_throttle: float
    current_steer: float
    current_brake: float
    reaction_sec: float
    hold_sec: float
    in_ditch_or_barrier: bool
    beta_deg: float = 0.0


@dataclass(frozen=True)
class DriverCommand:
    """Выходная команда модели водителя (газ, руль, тормоз)."""

    throttle: float
    steer: float
    brake: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.throttle, self.steer, self.brake)


class PostImpactDriverModel(Protocol):
    """Интерфейс подключаемой модели поведения водителя после первичного контакта."""

    mode_name: str

    def compute_command(self, state: PostImpactState) -> DriverCommand:
        ...


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
    max_steer_rate_per_sec: float = ACTIVE_MAX_STEER_RATE_PER_SEC,
    brake_rise_time_sec: float = BRAKE_RISE_TIME_SEC,
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
        target_road_y = LANE_RIGHT_CENTER_Y if actor_name == 'ego' else LANE_LEFT_CENTER_Y
    else:
        target_road_y = LANE_LEFT_CENTER_Y if actor_name == 'ego' else LANE_RIGHT_CENTER_Y

    lateral_err = loc_y - target_road_y
    yaw_rad = math.radians(normalize_deg(yaw_deg))
    desired_steer = max(-1.0, min(1.0, -(0.40 * lateral_err + 0.15 * vy + 0.50 * yaw_rad)))

    max_step = max_steer_rate_per_sec * dt
    steer_delta = desired_steer - current_steer
    new_steer = max(-1.0, min(1.0, current_steer + max(-max_step, min(max_step, steer_delta))))

    is_danger_zone = (abs(lateral_err) > 1.2) or (abs(beta_deg) > 4.0)
    desired_brake = 1.0 if is_danger_zone else 0.7
    brake_step = dt / brake_rise_time_sec
    new_brake = min(desired_brake, current_brake + brake_step)
    if new_brake > 0.0:
        kamm_steer = max(0.40, 1.0 - 0.55 * min(1.0, abs(new_steer) / 0.6))
        beta_scale = max(0.45, 1.0 - max(0.0, abs(beta_deg) - 5.0) / 18.0)
        new_brake = min(new_brake, kamm_steer * beta_scale)

    return 0.0, float(new_steer), float(new_brake)


class PassiveDriverModel:
    """Пассивный профиль водителя после T_react (накат с релаксацией руля для оценки Severity S)."""

    mode_name = 'passive'

    def compute_command(self, state: PostImpactState) -> DriverCommand:
        thr, steer, brake = compute_post_impact_control(
            actor_name=state.actor_name,
            subclass=state.subclass,
            driver_post_shock=self.mode_name,
            time_since_impact=state.time_since_impact,
            dt=state.dt,
            loc_y=state.loc_y,
            yaw_deg=state.yaw_deg,
            vy=state.vy,
            impact_steer=state.impact_steer,
            impact_throttle=state.impact_throttle,
            current_steer=state.current_steer,
            current_brake=state.current_brake,
            reaction_sec=state.reaction_sec,
            hold_sec=state.hold_sec,
            in_ditch_or_barrier=state.in_ditch_or_barrier,
            beta_deg=state.beta_deg,
        )
        return DriverCommand(throttle=thr, steer=steer, brake=brake)


class ActiveDriverModel:
    """Активный профиль водителя после T_react (контрруление + торможение Kamm+Sideslip ABS для Controllability C)."""

    mode_name = 'active'

    def __init__(
        self,
        max_steer_rate_per_sec: float = ACTIVE_MAX_STEER_RATE_PER_SEC,
        brake_rise_time_sec: float = BRAKE_RISE_TIME_SEC,
    ):
        self.max_steer_rate_per_sec = float(max_steer_rate_per_sec)
        self.brake_rise_time_sec = float(brake_rise_time_sec)

    def compute_command(self, state: PostImpactState) -> DriverCommand:
        thr, steer, brake = compute_post_impact_control(
            actor_name=state.actor_name,
            subclass=state.subclass,
            driver_post_shock=self.mode_name,
            time_since_impact=state.time_since_impact,
            dt=state.dt,
            loc_y=state.loc_y,
            yaw_deg=state.yaw_deg,
            vy=state.vy,
            impact_steer=state.impact_steer,
            impact_throttle=state.impact_throttle,
            current_steer=state.current_steer,
            current_brake=state.current_brake,
            reaction_sec=state.reaction_sec,
            hold_sec=state.hold_sec,
            in_ditch_or_barrier=state.in_ditch_or_barrier,
            beta_deg=state.beta_deg,
            max_steer_rate_per_sec=self.max_steer_rate_per_sec,
            brake_rise_time_sec=self.brake_rise_time_sec,
        )
        return DriverCommand(throttle=thr, steer=steer, brake=brake)


def create_driver_model(mode: str, **kwargs) -> PostImpactDriverModel:
    """Фабрика моделей пост-ударного управления водителя ('passive' | 'active')."""
    normalized = str(mode).strip().lower()
    if normalized == 'passive':
        return PassiveDriverModel()
    if normalized == 'active':
        return ActiveDriverModel(**kwargs)
    raise ValueError(f"Unsupported driver post-shock mode: {mode!r}")
