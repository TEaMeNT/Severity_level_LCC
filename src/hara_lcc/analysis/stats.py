"""
Статистические функции, доверительные интервалы (Клоппер–Пирсон, Уилсон) и хелперы форматирования ISO 26262-3.
"""

from __future__ import annotations

import math

import numpy as np
import scipy.stats as stats

from hara_lcc.severity.iso26262 import calc_clopper_pearson_ci, format_iso_pct


def format_iso_margin(margin: float) -> str:
    """Форматирование положительного запаса над порогом 10.0% с 3 знаками при 0 < margin < 0.05."""
    if 0.0 < margin < 0.05:
        return f"+{margin:.3f}%"
    return f"+{margin:.1f}%"


def plural_ru(n: int, forms: tuple[str, str, str]) -> str:
    """Склонение существительных в русском языке: (1 'прогон', 2 'прогона', 5 'прогонов')."""
    n_abs = abs(int(n))
    if 11 <= (n_abs % 100) <= 19:
        return forms[2]
    last = n_abs % 10
    if last == 1:
        return forms[0]
    if 2 <= last <= 4:
        return forms[1]
    return forms[2]


def calc_stats(values) -> dict:
    """Вычисление базовых статистических параметров числового ряда."""
    vals = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not vals:
        return {
            'count': 0, 'mean': 0.0, 'median': 0.0, 'std': 0.0,
            'min': 0.0, 'max': 0.0, 'p90': 0.0,
        }
    arr = np.array(vals)
    return {
        'count': int(len(arr)),
        'mean': float(np.mean(arr)),
        'median': float(np.median(arr)),
        'std': float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
        'p90': float(np.percentile(arr, 90)),
    }


def calc_wilson_ci(k: int, n: int, confidence: float = 0.95) -> tuple[float, float]:
    """
    Расчет доверительного интервала Уилсона (Wilson score interval) для биномиальной доли k/n.
    Возвращает (нижняя_граница, верхняя_граница) в долях единицы [0..1].
    """
    if n <= 0:
        return 0.0, 0.0
    p = k / n
    z = float(stats.norm.ppf(1.0 - (1.0 - confidence) / 2.0))
    z2 = z * z
    denom = 1.0 + z2 / n
    center = (p + z2 / (2.0 * n)) / denom
    half = (z / denom) * math.sqrt((p * (1.0 - p) / n) + (z2 / (4.0 * n * n)))
    return max(0.0, center - half), min(1.0, center + half)
