"""
Агрегация HARA по ISO 26262-3:2018 / SAE J2980:2023:
- предикаты выборки ДТП (N_crash), ошибок спавна и валидных событий;
- кумулятивное правило >10% (S0..S3);
- взвешивание ODD (W_A, W_B) и вердикт Worst-Case Subclass;
- количественная оценка управляемости Controllability (C);
- матрица назначения ASIL (ISO 26262-3 Table 4).
"""

from __future__ import annotations

import scipy.stats as stats

from hara_lcc.config import (
    ASIL_MATRIX,
    DEFAULT_WEIGHT_A,
    DEFAULT_WEIGHT_B,
    ISO_REPRESENTATIVE_THRESHOLD_PCT,
    SEVERITY_CLASSES,
    SEVERITY_FROM_RANK,
    SEVERITY_RANK,
)


def format_iso_pct(val: float) -> str:
    """Форматирование процентов ISO 26262: 2 знака в пограничной зоне (9.95, 10.05)%, иначе 1 знак."""
    if 9.95 < val < 10.05:
        return f"{val:.2f}%"
    return f"{val:.1f}%"


def calc_clopper_pearson_ci(k: int, n: int, confidence: float = 0.95) -> tuple[float, float]:
    """
    Расчет точного биномиального доверительного интервала Клоппера-Пирсона (Clopper-Pearson exact CI).
    Возвращает (нижняя_граница, верхняя_граница) в долях единицы [0..1].
    """
    if n <= 0:
        return 0.0, 0.0
    alpha = 1.0 - confidence
    low = float(stats.beta.ppf(alpha / 2.0, k, n - k + 1)) if k > 0 else 0.0
    high = float(stats.beta.ppf(1.0 - alpha / 2.0, k + 1, n - k)) if k < n else 1.0
    return low, high


def _to_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    if not val:
        return False
    return str(val).strip().lower() in ('true', '1', 't', 'yes', 'y')


def determine_asil(s_class: str, e_class: str, c_class: str) -> str:
    """Определение ASIL строго по матрице ISO 26262-3:2018 Table 4 / SAE J2980:2023 Table 7."""
    s_norm = str(s_class).strip().upper()
    e_norm = str(e_class).strip().upper()
    c_norm = str(c_class).strip().upper()
    if s_norm not in ('S1', 'S2', 'S3') or e_norm not in ('E1', 'E2', 'E3', 'E4') or c_norm not in ('C1', 'C2', 'C3'):
        return 'QM'
    return ASIL_MATRIX.get((s_norm, e_norm, c_norm), 'QM')


def is_crash_run(r: dict) -> bool:
    """
    Предикат состоявшегося ДТП (Accident-Involved Run) по ISO 26262-3 (Clause 6) / SAE J2980 (Section 5.2.2.2).
    Прогон считается аварией, если произошел хотя бы один физический контакт:
    - первичный боковой контакт ТС-ТС (side_collision == True);
    - вторичный удар в отбойник (barrier_collision == True);
    - сход / вылет в кювет (ditch_departure_actor not in ('None', '', None)).
    """
    if _to_bool(r.get('side_collision')):
        return True
    if _to_bool(r.get('barrier_collision')):
        return True
    ditch_actor = str(r.get('ditch_departure_actor', 'None')).strip()
    if ditch_actor and ditch_actor.lower() not in ('none', ''):
        return True
    return False


def is_spawn_error_run(r: dict) -> bool:
    """Предикат ошибки спавна актора в CARLA (SimulationCreationError / Unable to spawn)."""
    outcome = str(r.get('scenario_outcome', '')).strip().lower()
    return ('ошибка спавна' in outcome) or ('unable to spawn' in outcome) or ('simulationcreationerror' in outcome)


def is_valid_barrier_hit(r: dict) -> bool:
    """Единый предикат валидного удара об отбойник (ISO 26262-3 / SAE J2980)."""
    return (
        bool(r.get('barrier_collision'))
        and r.get('barrier_impact_mps') is not None
        and r.get('barrier_impact_mps') > 0.01
        and str(r.get('barrier_collision_actor', 'None')) not in ('None', '')
    )


def is_valid_ditch_departure(r: dict) -> bool:
    """Единый предикат валидного съезда/вылета в кювет (ISO 26262-3 / SAE J2980)."""
    return (
        str(r.get('ditch_departure_actor', 'None')) not in ('None', '')
        and r.get('departure_speed_mps') is not None
    )


def evaluate_iso_severity(counts: dict, total_crashes: int, total_runs: int = None, spawn_errors: int = 0) -> dict:
    """
    Оценка итогового класса тяжести по правилу кумулятивного порога >10%
    строго по выборке состоявшихся ДТП (Accident-Involved Runs).
    ISO 26262-3 (Annex B, Table B.1) / SAE J2980:2023 (Appendix B).
    """
    if total_runs is None:
        total_runs = total_crashes
    safe_pass = max(0, total_runs - total_crashes - spawn_errors)
    conflict_rate = (total_crashes / max(1, total_runs)) * 100.0

    if total_crashes == 0:
        return {
            'total': 0,
            'total_runs': total_runs,
            'total_crashes': 0,
            'safe_pass_runs': safe_pass,
            'spawn_error_runs': spawn_errors,
            'conflict_rate': conflict_rate,
            'counts': {s: 0 for s in SEVERITY_CLASSES},
            'percentages': {s: 0.0 for s in SEVERITY_CLASSES},
            'cum_percentages': {'S3': 0.0, '>=S2': 0.0, '>=S1': 0.0},
            'assigned_class': 'S0',
            'decision_step': 'Шаг 4: Нет данных (S0)',
            'decision_reason': 'Отсутствуют прогоны с состоявшимися ДТП для анализа.',
        }

    pcts = {s: (counts.get(s, 0) / total_crashes) * 100.0 for s in SEVERITY_CLASSES}
    p_s3 = pcts['S3']
    p_ge_s2 = pcts['S3'] + pcts['S2']
    p_ge_s1 = pcts['S3'] + pcts['S2'] + pcts['S1']

    cum_pcts = {
        'S3': p_s3,
        '>=S2': p_ge_s2,
        '>=S1': p_ge_s1,
    }

    # Алгоритм кумулятивной проверки сверху вниз (от S3 к S1) строго по P(S_i | ДТП)
    if p_s3 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S3'
        decision_step = 'Шаг 1: P(S3 | ДТП) > 10%'
        decision_reason = f"Доля критических/фатальных исходов среди ДТП P(S3 | ДТП) = {format_iso_pct(p_s3)} превышает нормативный порог 10.0%."
    elif p_ge_s2 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P(≥S2 | ДТП) > 10%'
        decision_reason = (
            f"Кумулятивная вероятность тяжелых травм среди ДТП P(≥S2 | ДТП) = {format_iso_pct(p_ge_s2)} "
            f"(S3: {format_iso_pct(p_s3)}, S2: {format_iso_pct(pcts['S2'])}) превышает нормативный порог 10.0%."
        )
    elif p_ge_s1 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P(≥S1 | ДТП) > 10%'
        decision_reason = (
            f"Кумулятивная вероятность травм легкой/средней тяжести среди ДТП P(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
            f"превышает нормативный порог 10.0%."
        )
    else:
        assigned_class = 'S0'
        decision_step = 'Шаг 4: P(≥S1 | ДТП) ≤ 10%'
        decision_reason = (
            f"Суммарная вероятность травм любого уровня среди ДТП P(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
            f"не превышает 10.0% (столкновения без травм AIS 0)."
        )

    return {
        'total': total_crashes,
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'safe_pass_runs': safe_pass,
        'spawn_error_runs': spawn_errors,
        'conflict_rate': conflict_rate,
        'counts': counts,
        'percentages': pcts,
        'cum_percentages': cum_pcts,
        'assigned_class': assigned_class,
        'decision_step': decision_step,
        'decision_reason': decision_reason,
    }


def evaluate_weighted_odd(
    analysis_a: dict,
    analysis_b: dict,
    weight_a: float = DEFAULT_WEIGHT_A,
    weight_b: float = DEFAULT_WEIGHT_B,
) -> dict:
    """
    Взвешенная агрегация вероятностей по всему пространству условий ODD (W_A = 0.20, W_B = 0.80)
    строго по выборке ДТП с применением кумулятивного правила >10% ISO 26262-3.
    """
    total_w = weight_a + weight_b
    w_a = weight_a / total_w
    w_b = weight_b / total_w

    pct_a = analysis_a['percentages']
    pct_b = analysis_b['percentages']

    weighted_pcts = {}
    for s in SEVERITY_CLASSES:
        weighted_pcts[s] = w_a * pct_a.get(s, 0.0) + w_b * pct_b.get(s, 0.0)

    p_s3 = weighted_pcts['S3']
    p_ge_s2 = weighted_pcts['S3'] + weighted_pcts['S2']
    p_ge_s1 = weighted_pcts['S3'] + weighted_pcts['S2'] + weighted_pcts['S1']

    cum_pcts = {
        'S3': p_s3,
        '>=S2': p_ge_s2,
        '>=S1': p_ge_s1,
    }

    if p_s3 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S3'
        decision_step = 'Шаг 1: P_ODD(S3 | ДТП) > 10%'
        decision_reason = f"Взвешенная вероятность среди ДТП P_ODD(S3 | ДТП) = {format_iso_pct(p_s3)} превышает нормативный порог 10.0%."
    elif p_ge_s2 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P_ODD(≥S2 | ДТП) > 10%'
        decision_reason = (
            f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S2 | ДТП) = {format_iso_pct(p_ge_s2)} "
            f"превышает нормативный порог 10.0%."
        )
    elif p_ge_s1 > ISO_REPRESENTATIVE_THRESHOLD_PCT:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P_ODD(≥S1 | ДТП) > 10%'
        decision_reason = (
            f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
            f"превышает нормативный порог 10.0%."
        )
    else:
        assigned_class = 'S0'
        decision_step = 'Шаг 4: P_ODD(≥S1 | ДТП) ≤ 10%'
        decision_reason = f"Взвешенная вероятность вреда среди ДТП P_ODD(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} не превышает 10.0%."

    total_runs = analysis_a.get('total_runs', 0) + analysis_b.get('total_runs', 0)
    total_crashes = analysis_a['total_crashes'] + analysis_b['total_crashes']

    return {
        'weights': {'A': w_a, 'B': w_b},
        'total': total_crashes,
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'percentages': weighted_pcts,
        'cum_percentages': cum_pcts,
        'assigned_class': assigned_class,
        'decision_step': decision_step,
        'decision_reason': decision_reason,
    }


def evaluate_controllability(records_active: list[dict], harm_threshold: str = 'S2') -> dict:
    """
    Количественная оценка управляемости (Controllability, C) по результатам
    кампании '--driver-post-shock active' (SAE J2980:2023 Clause 5.2.3).
    Считает долю прогонов P_avoid, в которых водители предотвратили вред >= harm_threshold,
    и строит 95% биномиальный ДИ Клоппера–Пирсона.
    """
    thr_rank = SEVERITY_RANK.get(harm_threshold, 2)
    valid_runs = [r for r in records_active if not is_spawn_error_run(r)]
    conflict_runs = [r for r in valid_runs if is_crash_run(r)]

    n_valid = len(valid_runs)
    k_avoid_valid = sum(1 for r in valid_runs if SEVERITY_RANK.get(r['severity'], 0) < thr_rank)
    p_avoid_valid = (k_avoid_valid / n_valid * 100.0) if n_valid > 0 else 0.0
    ci_valid = calc_clopper_pearson_ci(k_avoid_valid, n_valid, confidence=0.95)

    n_crashes = len(conflict_runs)
    k_avoid_crashes = sum(1 for r in conflict_runs if SEVERITY_RANK.get(r['severity'], 0) < thr_rank)
    p_avoid_crashes = (k_avoid_crashes / n_crashes * 100.0) if n_crashes > 0 else 0.0
    ci_crashes = calc_clopper_pearson_ci(k_avoid_crashes, n_crashes, confidence=0.95)

    ref_n = n_crashes if n_crashes > 0 else n_valid
    ref_p = p_avoid_crashes if n_crashes > 0 else p_avoid_valid
    ref_ci = ci_crashes if n_crashes > 0 else ci_valid

    if ref_n == 0:
        c_class = 'C3'
        ci_note = 'нет данных'
    else:
        if ref_p >= 99.0:
            c_class = 'C1'
        elif ref_p >= 90.0:
            c_class = 'C2'
        else:
            c_class = 'C3'

        ci_low_pct, ci_high_pct = ref_ci[0] * 100.0, ref_ci[1] * 100.0
        if c_class == 'C2' and ci_low_pct < 90.0:
            ci_note = f"95% CI [{ci_low_pct:.1f}%, {ci_high_pct:.1f}%] пересекает 90% (погранично с C3)"
        elif c_class == 'C3' and ci_high_pct >= 90.0:
            ci_note = f"95% CI [{ci_low_pct:.1f}%, {ci_high_pct:.1f}%] пересекает 90% (требуется больший объём выборок)"
        else:
            ci_note = f"95% CI [{ci_low_pct:.1f}%, {ci_high_pct:.1f}%]"

    return {
        'n_valid': n_valid,
        'k_avoid_valid': k_avoid_valid,
        'p_avoid_valid': p_avoid_valid,
        'ci_valid': ci_valid,
        'n_crashes': n_crashes,
        'k_avoid_crashes': k_avoid_crashes,
        'p_avoid_crashes': p_avoid_crashes,
        'ci_crashes': ci_crashes,
        'assigned_c': c_class,
        'ci_note': ci_note,
    }


def compute_worst_case_verdict(class_a: str, class_b: str) -> str:
    """Определение консервативного вердикта Worst-Case Subclass = max(S_A, S_B)."""
    rank_a = SEVERITY_RANK.get(class_a, 0)
    rank_b = SEVERITY_RANK.get(class_b, 0)
    return SEVERITY_FROM_RANK[max(rank_a, rank_b)]
