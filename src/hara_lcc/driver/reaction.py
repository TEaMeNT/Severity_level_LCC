"""
Стохастическая модель времени реакции водителя (T_react) и мышечного удержания (T_hold).
Green 2000, Olson & Sivak 1986, Taoka 1989, SAE J2980:2023 Clause 5.2.3.2(d).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random

from hara_lcc.config import (
    HOLD_TIME_MAX_SEC,
    HOLD_TIME_MIN_SEC,
    REACTION_TIME_LOG_SIGMA,
    REACTION_TIME_MAX_SEC,
    REACTION_TIME_MEDIAN_SEC,
    REACTION_TIME_MIN_SEC,
)


@dataclass(frozen=True)
class DriverTimingConfig:
    """Параметры распределения времени реакции и мышечного удержания водителя."""

    reaction_median_sec: float = REACTION_TIME_MEDIAN_SEC
    reaction_log_sigma: float = REACTION_TIME_LOG_SIGMA
    reaction_min_sec: float = REACTION_TIME_MIN_SEC
    reaction_max_sec: float = REACTION_TIME_MAX_SEC
    hold_min_sec: float = HOLD_TIME_MIN_SEC
    hold_max_sec: float = HOLD_TIME_MAX_SEC


def sample_reaction_and_hold_times(
    rng=random,
    timing: DriverTimingConfig | None = None,
) -> tuple[float, float]:
    """
    Сэмплирование времени реакции водителя (T_react) и времени мышечного удержания (T_hold).
    T_react сэмплируется из усечённого логнормального распределения (Green 2000, Taoka 1989)
    для трезвого внимательного водителя (SAE J2980:2023 Clause 5.2.3.2(d)).
    T_hold — инженерная эвристика кратковременного удержания руля/газа до сброса педали.
    """
    cfg = timing or DriverTimingConfig()
    raw_react = math.exp(rng.gauss(math.log(cfg.reaction_median_sec), cfg.reaction_log_sigma))
    t_react = min(cfg.reaction_max_sec, max(cfg.reaction_min_sec, raw_react))
    t_hold = rng.uniform(cfg.hold_min_sec, min(cfg.hold_max_sec, t_react))
    return t_react, t_hold
