"""Тесты изолированной модели водителя: T_react/T_hold, passive/active, лимит руля, рампа тормоза, non-oracle ABS."""

from __future__ import annotations

import math
import random

import pytest

from hara_lcc.config import (
    ACTIVE_MAX_STEER_RATE_PER_SEC,
    BRAKE_RISE_TIME_SEC,
    HOLD_TIME_MAX_SEC,
    HOLD_TIME_MIN_SEC,
    REACTION_TIME_MAX_SEC,
    REACTION_TIME_MIN_SEC,
    SUBCLASS_A,
    SUBCLASS_B,
)
from hara_lcc.driver import (
    ActiveDriverModel,
    PassiveDriverModel,
    PostImpactState,
    compute_post_impact_control,
    create_driver_model,
    normalize_deg,
    sample_reaction_and_hold_times,
)


def test_reaction_and_hold_time_bounds_and_determinism():
    rng1 = random.Random(42)
    rng2 = random.Random(42)
    samples1 = [sample_reaction_and_hold_times(rng1) for _ in range(500)]
    samples2 = [sample_reaction_and_hold_times(rng2) for _ in range(500)]

    assert samples1 == samples2
    for t_react, t_hold in samples1:
        assert REACTION_TIME_MIN_SEC <= t_react <= REACTION_TIME_MAX_SEC
        assert HOLD_TIME_MIN_SEC <= t_hold <= min(HOLD_TIME_MAX_SEC, t_react)


def test_hold_and_reaction_phases_and_passive_decay():
    dt = 0.01
    # Фаза удержания (t <= T_hold), на дороге
    thr, steer, brake = compute_post_impact_control(
        actor_name='ego', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=0.15, dt=dt, loc_y=3.0, yaw_deg=5.0, vy=1.0,
        impact_steer=0.25, impact_throttle=0.60, current_steer=0.25, current_brake=0.0,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=3.0,
    )
    assert (thr, steer, brake) == (0.60, 0.25, 0.0)

    # Фаза удержания при контакте с барьером/кюветом — принудительный сброс газа
    thr_cut, steer_cut, brake_cut = compute_post_impact_control(
        actor_name='ego', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=0.15, dt=dt, loc_y=10.0, yaw_deg=5.0, vy=1.0,
        impact_steer=0.25, impact_throttle=0.60, current_steer=0.25, current_brake=0.0,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=True, beta_deg=3.0,
    )
    assert (thr_cut, steer_cut, brake_cut) == (0.0, 0.25, 0.0)

    # Между T_hold и T_react — выбег и релаксация руля 0.95^(dt/0.01)
    thr_c, steer_c, brake_c = compute_post_impact_control(
        actor_name='ego', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=0.50, dt=0.02, loc_y=3.0, yaw_deg=5.0, vy=1.0,
        impact_steer=0.25, impact_throttle=0.60, current_steer=0.20, current_brake=0.0,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=3.0,
    )
    assert thr_c == 0.0
    assert brake_c == 0.0
    assert steer_c == pytest.approx(0.20 * math.pow(0.95, 2.0))

    # После T_react в режиме passive — продолжение выбега без тормоза
    thr_p, steer_p, brake_p = compute_post_impact_control(
        actor_name='ego', subclass=SUBCLASS_B, driver_post_shock='passive',
        time_since_impact=1.50, dt=dt, loc_y=8.0, yaw_deg=15.0, vy=3.0,
        impact_steer=0.25, impact_throttle=0.60, current_steer=0.20, current_brake=0.0,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=10.0,
    )
    assert thr_p == 0.0
    assert brake_p == 0.0
    assert steer_p == pytest.approx(0.20 * 0.95)


def test_active_steer_rate_limit_and_brake_ramp():
    dt = 0.01
    thr, new_steer, new_brake = compute_post_impact_control(
        actor_name='target', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=1.20, dt=dt, loc_y=8.5, yaw_deg=25.0, vy=4.0,
        impact_steer=0.0, impact_throttle=0.5, current_steer=0.0, current_brake=0.0,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=0.0,
    )
    assert thr == 0.0
    # Макс. приращение руля за тик: 2.5 * 0.01 = 0.025
    assert abs(new_steer - 0.0) <= ACTIVE_MAX_STEER_RATE_PER_SEC * dt + 1e-12
    assert new_steer == pytest.approx(-ACTIVE_MAX_STEER_RATE_PER_SEC * dt)
    # Рампа тормоза за первый тик: dt / 0.25 = 0.04 (ниже потолка Kamm)
    assert new_brake == pytest.approx(dt / BRAKE_RISE_TIME_SEC)


def test_non_oracle_kamm_sideslip_abs_invariant_to_loc_y():
    """
    Проверяем, что модуляция торможения Kamm+Sideslip ABS зависит исключительно от бортовых величин
    (|new_steer| и |beta_deg|) и НЕ содержит координатных оракулов по мировой координате loc_y.
    """
    dt = 0.01
    # Одинаковая lateral_err = +2.0 м, но разные мировые loc_y:
    # 1) SubB ego (target_road_y = 1.875) -> loc_y = 3.875 (на асфальте)
    # 2) SubB target (target_road_y = 5.625) -> loc_y = 7.625 (на гравийной обочине >= 7.50)
    # 3) SubA ego (target_road_y = 5.625) -> loc_y = 7.625 (на обочине)
    _, steer_asphalt, brake_asphalt = compute_post_impact_control(
        actor_name='ego', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=2.0, dt=dt, loc_y=1.875 + 2.0, yaw_deg=0.0, vy=0.0,
        impact_steer=0.0, impact_throttle=0.0, current_steer=-0.40, current_brake=0.95,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=14.0,
    )
    _, steer_shoulder, brake_shoulder = compute_post_impact_control(
        actor_name='target', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=2.0, dt=dt, loc_y=5.625 + 2.0, yaw_deg=0.0, vy=0.0,
        impact_steer=0.0, impact_throttle=0.0, current_steer=-0.40, current_brake=0.95,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=14.0,
    )
    assert steer_asphalt == pytest.approx(steer_shoulder)
    assert brake_asphalt == pytest.approx(brake_shoulder)

    # Прямая сверка аналитической формулы Kamm * Sideslip
    kamm_steer = max(0.40, 1.0 - 0.55 * min(1.0, abs(steer_asphalt) / 0.6))
    beta_scale = max(0.45, 1.0 - max(0.0, abs(14.0) - 5.0) / 18.0)
    assert brake_asphalt == pytest.approx(kamm_steer * beta_scale)

    # При росте заноса beta_deg тормоз снижается
    _, _, brake_low_beta = compute_post_impact_control(
        actor_name='target', subclass=SUBCLASS_B, driver_post_shock='active',
        time_since_impact=2.0, dt=dt, loc_y=5.625 + 2.0, yaw_deg=0.0, vy=0.0,
        impact_steer=0.0, impact_throttle=0.0, current_steer=-0.40, current_brake=0.95,
        reaction_sec=1.0, hold_sec=0.25, in_ditch_or_barrier=False, beta_deg=4.5,
    )
    assert brake_shoulder < brake_low_beta


def test_driver_model_factory_and_protocol():
    passive = create_driver_model("passive")
    active = create_driver_model("active")
    assert isinstance(passive, PassiveDriverModel)
    assert isinstance(active, ActiveDriverModel)

    state = PostImpactState(
        actor_name='ego', subclass=SUBCLASS_A, time_since_impact=1.5, dt=0.01,
        loc_y=4.0, yaw_deg=-10.0, vy=-1.0, impact_steer=-0.1, impact_throttle=0.5,
        current_steer=-0.1, current_brake=0.1, reaction_sec=1.0, hold_sec=0.25,
        in_ditch_or_barrier=False, beta_deg=6.0,
    )
    p_cmd = passive.compute_command(state)
    a_cmd = active.compute_command(state)
    assert p_cmd.brake == 0.0
    assert a_cmd.brake > 0.0
    assert normalize_deg(540.0) == 180.0
    assert normalize_deg(-190.0) == 170.0

    with pytest.raises(ValueError):
        create_driver_model("unknown_mode")
