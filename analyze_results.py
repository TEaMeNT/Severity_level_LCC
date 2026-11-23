# ==============================================================================
# HARA STATISTICAL ANALYTICS & ISO 26262 / SAE J2980:2023 DECISION MODULE
# Скрипт: analyze_results.py (Ревизия 3.1: Выборки ДТП N_crash, подклассы A и B,
# строгий метод граничных условий Y = 9.75 м и количественная оценка C)
# Стандарты: ISO 26262-3:2018 (Clause 6, Annex B), SAE J2980:2023 (5.2.3, Annex B)
# ==============================================================================

import os
import sys
import math
import csv
import argparse
import glob
import re
from collections import defaultdict

# Настройка кодировки стандартного вывода на Windows
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

import numpy as np
import scipy.stats as stats
import matplotlib
matplotlib.use('Agg')  # Headless backend для автономной генерации графиков
import matplotlib.pyplot as plt

# Настройка шрифтов для корректного отображения кириллицы на Windows/Linux
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Segoe UI', 'Tahoma', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# ------------------------------------------------------------------------------
# Константы, ранжирование и цветовая палитра
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
    'SUBCLASS_A_BARRIER': '#2980b9',  # Синий
    'SUBCLASS_B_DITCH': '#8e44ad',    # Фиолетовый
    'ODD_WEIGHTED': '#2c3e50',        # Темно-синий/графитовый
}

SUBCLASS_NAMES = {
    'SUBCLASS_A_BARRIER': 'Подкласс А (Ограждение)',
    'SUBCLASS_B_DITCH': 'Подкласс B (Кювет)',
    'ODD_WEIGHTED': 'Взвешенный ODD'
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


def determine_asil(s_class: str, e_class: str, c_class: str) -> str:
    """Определение ASIL строго по матрице ISO 26262-3:2018 Table 4 / SAE J2980:2023 Table 7."""
    s_norm = str(s_class).strip().upper()
    e_norm = str(e_class).strip().upper()
    c_norm = str(c_class).strip().upper()
    if s_norm not in ('S1', 'S2', 'S3') or e_norm not in ('E1', 'E2', 'E3', 'E4') or c_norm not in ('C1', 'C2', 'C3'):
        return 'QM'
    return ASIL_MATRIX.get((s_norm, e_norm, c_norm), 'QM')


def format_iso_pct(val: float) -> str:
    """Форматирование процентов ISO 26262: 2 знака в пограничной зоне (9.95, 10.05)%, иначе 1 знак."""
    if 9.95 < val < 10.05:
        return f"{val:.2f}%"
    return f"{val:.1f}%"


def format_iso_margin(margin: float) -> str:
    """Форматирование положительного запаса над порогом 10.0% с 3 знаками при 0 < margin < 0.05."""
    if 0.0 < margin < 0.05:
        return f"+{margin:.3f}%"
    return f"+{margin:.1f}%"


def to_float(val, default=None):
    """Безопасное преобразование строки в float."""
    if val is None:
        return default
    val_str = str(val).strip()
    if not val_str or val_str.lower() == 'none':
        return default
    try:
        return float(val_str)
    except ValueError:
        return default


def to_bool(val):
    """Безопасное преобразование строки в bool."""
    if isinstance(val, bool):
        return val
    if not val:
        return False
    val_str = str(val).strip().lower()
    return val_str in ('true', '1', 't', 'yes', 'y')


def is_crash_run(r: dict) -> bool:
    """
    Предикат состоявшегося ДТП (Accident-Involved Run) по ISO 26262-3 (Clause 6) / SAE J2980 (Section 5.2.2.2).
    Прогон считается аварией, если произошел хотя бы один физический контакт:
    - первичный боковой контакт ТС-ТС (side_collision == True);
    - вторичный удар в отбойник (barrier_collision == True);
    - сход / вылет в кювет (ditch_departure_actor not in ('None', '', None)).
    """
    if to_bool(r.get('side_collision')):
        return True
    if to_bool(r.get('barrier_collision')):
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


def calc_stats(values):
    """Вычисление базовых статистических параметров числового ряда."""
    vals = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not vals:
        return {
            'count': 0, 'mean': 0.0, 'median': 0.0, 'std': 0.0,
            'min': 0.0, 'max': 0.0, 'p90': 0.0
        }
    arr = np.array(vals)
    return {
        'count': int(len(arr)),
        'mean': float(np.mean(arr)),
        'median': float(np.median(arr)),
        'std': float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
        'p90': float(np.percentile(arr, 90))
    }


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


# ------------------------------------------------------------------------------
# Загрузка и нормализация CSV данных
# ------------------------------------------------------------------------------
def load_simulation_results(input_path: str) -> list[dict]:
    """
    Загрузка результатов моделирования из CSV-файла или директории.
    Поддерживает как новую 25-колоночную схему, так и легаси CSV.
    """
    files_to_read = []
    if os.path.isdir(input_path):
        target_csv = os.path.join(input_path, "lcc_hara_results.csv")
        if os.path.isfile(target_csv):
            files_to_read.append(target_csv)
        else:
            files_to_read.extend(glob.glob(os.path.join(input_path, "*.csv")))
    elif os.path.isfile(input_path):
        files_to_read.append(input_path)
    else:
        raise FileNotFoundError(f"Указанный путь не найден: {input_path}")

    if not files_to_read:
        raise FileNotFoundError(f"Не найдено ни одного CSV-файла по пути: {input_path}")

    records = []
    for fpath in files_to_read:
        with open(fpath, mode='r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Нормализация имени подкласса (унификация A / B с поддержкой легаси C)
                subclass_raw = row.get('subclass', '').strip()
                if 'barrier' in subclass_raw.lower() or subclass_raw == 'A':
                    subclass = 'SUBCLASS_A_BARRIER'
                elif 'ditch' in subclass_raw.lower() or subclass_raw in ('B', 'C'):
                    subclass = 'SUBCLASS_B_DITCH'
                else:
                    subclass = subclass_raw

                # Извлечение численных параметров для верификации классов
                eff_side_dv = to_float(row.get('effective_side_dv_mps'))
                if eff_side_dv is not None:
                    if eff_side_dv > 8.0:
                        sev_side = 'S3'
                    elif eff_side_dv > 3.5:
                        sev_side = 'S2'
                    elif eff_side_dv > 1.0:
                        sev_side = 'S1'
                    else:
                        sev_side = 'S0'
                else:
                    raw_s = row.get('sev_side', 'S0').strip().upper()
                    sev_side = raw_s if raw_s in SEVERITY_CLASSES else 'S0'

                barrier_hit = to_bool(row.get('barrier_collision'))
                v_barrier = to_float(row.get('barrier_impact_mps'))
                b_type = row.get('barrier_impact_type', '').strip()
                if barrier_hit and v_barrier is not None and v_barrier > 0:
                    if 'frontal' in b_type.lower() or 'rear' in b_type.lower():
                        if v_barrier > 18.0:
                            sev_barrier = 'S3'
                        elif v_barrier > 11.0:
                            sev_barrier = 'S2'
                        elif v_barrier > 6.0:
                            sev_barrier = 'S1'
                        else:
                            sev_barrier = 'S0'
                    else:  # Side-swipe
                        if v_barrier > 12.0:
                            sev_barrier = 'S3'
                        elif v_barrier > 6.0:
                            sev_barrier = 'S2'
                        elif v_barrier > 2.0:
                            sev_barrier = 'S1'
                        else:
                            sev_barrier = 'S0'
                else:
                    raw_b = row.get('sev_barrier', 'S0').strip().upper()
                    sev_barrier = raw_b if raw_b in SEVERITY_CLASSES else 'S0'

                v_dep = to_float(row.get('departure_speed_mps'))
                v_perp = to_float(row.get('road_perp_speed_mps'))
                v_lat = to_float(row.get('vehicle_lat_skid_mps', row.get('max_lateral_slip_mps')))
                beta = to_float(row.get('sideslip_angle_deg'))
                omega_z = to_float(row.get('departure_yaw_rate_deg_s'))
                ditch_actor = row.get('ditch_departure_actor', 'None')
                if ditch_actor not in ('None', '') and v_dep is not None and v_dep > 0:
                    is_non_tracking = (beta is not None and beta >= 15.0)
                    is_s3_skid = (v_dep > 18.0) and (beta is not None) and (
                        (beta >= 25.0) or (beta >= 20.0 and omega_z is not None and omega_z > 45.0)
                    )
                    is_s3_nose = (v_dep > 8.0) and (v_perp is not None and v_perp > 12.0)
                    is_s2_trip = (v_dep > 14.0) and is_non_tracking and (v_lat is not None and v_lat > 5.5)
                    is_s2_skid = (v_dep > 11.0) and (beta is not None and beta >= 20.0)
                    is_s2_bottom = (v_dep > 8.0) and (v_perp is not None and v_perp > 8.0)
                    is_s1_tracking = (v_dep > 8.0 and (v_perp is not None and v_perp > 2.0))
                    is_s1_skid = is_non_tracking

                    if is_s3_skid or is_s3_nose:
                        sev_ditch = 'S3'
                    elif is_s2_trip or is_s2_skid or is_s2_bottom:
                        sev_ditch = 'S2'
                    elif is_s1_tracking or is_s1_skid:
                        sev_ditch = 'S1'
                    else:
                        sev_ditch = 'S0'
                else:
                    raw_d = row.get('sev_ditch', 'S0').strip().upper()
                    sev_ditch = raw_d if raw_d in SEVERITY_CLASSES else 'S0'

                # Итоговая тяжесть прогона как наихудший класс из трех компонентов
                final_rank = max(SEVERITY_RANK[sev_side], SEVERITY_RANK[sev_barrier], SEVERITY_RANK[sev_ditch])
                severity = SEVERITY_FROM_RANK[final_rank]

                rec = {
                    'run_id': row.get('run_id', ''),
                    'subclass': subclass,
                    'driver_post_shock_mode': row.get('driver_post_shock_mode', 'passive'),
                    'shock_duration_sec': to_float(row.get('shock_duration_sec')),
                    'side_collision': to_bool(row.get('side_collision')),
                    'delta_v_ego_mps': to_float(row.get('delta_v_ego_mps')),
                    'delta_v_target_mps': to_float(row.get('delta_v_target_mps')),
                    'effective_side_dv_mps': eff_side_dv,
                    'sev_side': sev_side,
                    'barrier_collision': barrier_hit,
                    'barrier_collision_actor': row.get('barrier_collision_actor', 'None'),
                    'barrier_impact_type': b_type,
                    'barrier_impact_mps': v_barrier,
                    'sev_barrier': sev_barrier,
                    'ditch_departure_actor': ditch_actor,
                    'departure_speed_mps': v_dep,
                    'road_perp_speed_mps': v_perp,
                    'encroachment_angle_deg': to_float(row.get('encroachment_angle_deg')),
                    'vehicle_lat_skid_mps': v_lat,
                    'sideslip_angle_deg': beta,
                    'departure_yaw_rate_deg_s': omega_z,
                    'ditch_mechanism': row.get('ditch_mechanism', ''),
                    'sev_ditch': sev_ditch,
                    'severity': severity,
                    'scenario_outcome': row.get('scenario_outcome', '')
                }
                rec['is_crash'] = is_crash_run(rec)
                rec['is_spawn_error'] = is_spawn_error_run(rec)
                records.append(rec)

    return records


# ------------------------------------------------------------------------------
# Математическая оценка по правилу >10% (ISO 26262-3 / SAE J2980:2023) строго по N_crash
# ------------------------------------------------------------------------------
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
            'decision_reason': 'Отсутствуют прогоны с состоявшимися ДТП для анализа.'
        }

    pcts = {s: (counts.get(s, 0) / total_crashes) * 100.0 for s in SEVERITY_CLASSES}
    p_s3 = pcts['S3']
    p_ge_s2 = pcts['S3'] + pcts['S2']
    p_ge_s1 = pcts['S3'] + pcts['S2'] + pcts['S1']

    cum_pcts = {
        'S3': p_s3,
        '>=S2': p_ge_s2,
        '>=S1': p_ge_s1
    }

    # Алгоритм кумулятивной проверки сверху вниз (от S3 к S1) строго по P(S_i | ДТП)
    if p_s3 > 10.0:
        assigned_class = 'S3'
        decision_step = 'Шаг 1: P(S3 | ДТП) > 10%'
        decision_reason = f"Доля критических/фатальных исходов среди ДТП P(S3 | ДТП) = {format_iso_pct(p_s3)} превышает нормативный порог 10.0%."
    elif p_ge_s2 > 10.0:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P(≥S2 | ДТП) > 10%'
        decision_reason = (f"Кумулятивная вероятность тяжелых травм среди ДТП P(≥S2 | ДТП) = {format_iso_pct(p_ge_s2)} "
                           f"(S3: {format_iso_pct(p_s3)}, S2: {format_iso_pct(pcts['S2'])}) превышает нормативный порог 10.0%.")
    elif p_ge_s1 > 10.0:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P(≥S1 | ДТП) > 10%'
        decision_reason = (f"Кумулятивная вероятность травм легкой/средней тяжести среди ДТП P(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
                           f"превышает нормативный порог 10.0%.")
    else:
        assigned_class = 'S0'
        decision_step = 'Шаг 4: P(≥S1 | ДТП) ≤ 10%'
        decision_reason = (f"Суммарная вероятность травм любого уровня среди ДТП P(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
                           f"не превышает 10.0% (столкновения без травм AIS 0).")

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
        'decision_reason': decision_reason
    }


def evaluate_weighted_odd(analysis_a: dict, analysis_b: dict, weight_a: float = 0.20, weight_b: float = 0.80) -> dict:
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
        '>=S1': p_ge_s1
    }

    if p_s3 > 10.0:
        assigned_class = 'S3'
        decision_step = 'Шаг 1: P_ODD(S3 | ДТП) > 10%'
        decision_reason = f"Взвешенная вероятность среди ДТП P_ODD(S3 | ДТП) = {format_iso_pct(p_s3)} превышает нормативный порог 10.0%."
    elif p_ge_s2 > 10.0:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P_ODD(≥S2 | ДТП) > 10%'
        decision_reason = (f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S2 | ДТП) = {format_iso_pct(p_ge_s2)} "
                           f"превышает нормативный порог 10.0%.")
    elif p_ge_s1 > 10.0:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P_ODD(≥S1 | ДТП) > 10%'
        decision_reason = (f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S1 | ДТП) = {format_iso_pct(p_ge_s1)} "
                           f"превышает нормативный порог 10.0%.")
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
        'decision_reason': decision_reason
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


def annotate_ditch_points(ax, points: list[dict], x_key: str, y_key: str, x_span: float, y_span: float, is_panel_a: bool = False):
    """
    Автоматическое разведение подписей #run_id из кандидатов с тонкими выносными линиями.
    Использует 2D Bounding Box collision detection для полного устранения наложений.
    Для S0/S1 в плотном кластере на Панели А выполняет адаптивное прореживание.
    """
    candidates = [
        (6, 6), (6, -10), (-24, 6), (-24, -10),
        (0, 10), (0, -13), (10, 0), (-28, 0),
        (12, 8), (-28, -12), (12, -12), (-30, 8)
    ]
    pts_norm = []
    for pt in points:
        x_val = pt[x_key] if pt[x_key] is not None else 0.0
        y_val = pt[y_key] if pt[y_key] is not None else 0.0
        pts_norm.append((x_val / max(0.1, x_span), y_val / max(0.1, y_span)))

    placed_boxes = []
    placed_points = []

    # Динамический приоритет критическим исходам: сначала S3, затем S2 и максимальные скорости/скольжение
    def get_rank(pt):
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        v_perp = pt.get('road_perp_speed_mps') or 0.0
        v_lat = pt.get('vehicle_lat_skid_mps') or 0.0
        return (-SEVERITY_RANK.get(sev, 0), -(v_perp / 12.0 + v_lat / 5.5))

    sorted_indices = sorted(range(len(points)), key=lambda i: get_rank(points[i]))

    for idx in sorted_indices:
        pt = points[idx]
        x_val = pt[x_key] if pt[x_key] is not None else 0.0
        y_val = pt[y_key] if pt[y_key] is not None else 0.0
        nx, ny = pts_norm[idx]

        # Прореживание плотного кластера для S0/S1; для S3/S2 сохраняем все подписи
        dist_thresh = 0.055 if is_panel_a else 0.048
        sev_lbl = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        is_key_point = sev_lbl in ('S3', 'S2')
        if not is_key_point:
            too_close = False
            for px, py in placed_points:
                if math.hypot(nx - px, ny - py) < dist_thresh:
                    too_close = True
                    break
            if too_close:
                continue

        best_cand = candidates[0]
        best_score = -1e9
        best_box = (0, 0, 0, 0)

        lbl_w = 0.046
        lbl_h = 0.024

        for cdx, cdy in candidates:
            dx_norm = cdx / 460.0
            dy_norm = cdy / 360.0

            if cdx > 0:
                bx0 = nx + dx_norm
                bx1 = bx0 + lbl_w
            elif cdx < 0:
                bx1 = nx + dx_norm
                bx0 = bx1 - lbl_w
            else:
                bx0 = nx - lbl_w / 2.0
                bx1 = nx + lbl_w / 2.0

            if cdy > 0:
                by0 = ny + dy_norm
                by1 = by0 + lbl_h
            elif cdy < 0:
                by1 = ny + dy_norm
                by0 = by1 - lbl_h
            else:
                by0 = ny - lbl_h / 2.0
                by1 = ny + lbl_h / 2.0

            # Штраф за границы графика
            boundary_penalty = 0.0
            if bx0 < 0.01:
                boundary_penalty += (0.01 - bx0) * 500.0
            if bx1 > 0.99:
                boundary_penalty += (bx1 - 0.99) * 500.0
            if by0 < 0.01:
                boundary_penalty += (0.01 - by0) * 500.0
            if by1 > 0.99:
                boundary_penalty += (by1 - 0.99) * 500.0

            # Пересечение с уже размещенными подписями (2D Box Overlap)
            overlap_label_penalty = 0.0
            for px0, py0, px1, py1 in placed_boxes:
                ox = max(0.0, min(bx1, px1) - max(bx0, px0))
                oy = max(0.0, min(by1, py1) - max(by0, py0))
                if ox > 0.0 and oy > 0.0:
                    overlap_label_penalty += (ox * oy) * 10000.0 + 10.0

            # Пересечение с точками данных
            point_cover_penalty = 0.0
            for j, (pnx, pny) in enumerate(pts_norm):
                if j == idx:
                    continue
                if (bx0 - 0.008 <= pnx <= bx1 + 0.008) and (by0 - 0.008 <= pny <= by1 + 0.008):
                    point_cover_penalty += 25.0

            # Бонус за дистанцию
            cx = (bx0 + bx1) / 2.0
            cy = (by0 + by1) / 2.0
            d_pts = min([math.hypot(cx - pnx, cy - pny) for j, (pnx, pny) in enumerate(pts_norm) if j != idx] or [1.0])
            d_lbls = min([math.hypot(cx - (px0+px1)/2, cy - (py0+py1)/2) for px0, py0, px1, py1 in placed_boxes] or [1.0])

            score = (d_pts * 2.0 + d_lbls * 1.5) - (boundary_penalty + overlap_label_penalty + point_cover_penalty)

            if score > best_score:
                best_score = score
                best_cand = (cdx, cdy)
                best_box = (bx0, by0, bx1, by1)

        cdx, cdy = best_cand
        ha = 'left' if cdx > 0 else ('right' if cdx < 0 else 'center')
        va = 'bottom' if cdy > 0 else ('top' if cdy < 0 else 'center')

        placed_boxes.append(best_box)
        placed_points.append((nx, ny))

        ax.annotate(
            f"#{pt['run_id']}",
            xy=(x_val, y_val),
            xytext=(cdx, cdy),
            textcoords="offset points",
            ha=ha, va=va,
            fontsize=7.5, fontweight='bold', color='#2c3e50',
            arrowprops=dict(arrowstyle='-', color='gray', lw=0.6, alpha=0.7)
        )


# ------------------------------------------------------------------------------
# Построение 5 обновленных инженерных графиков (results/plots/, 300 DPI)
# ------------------------------------------------------------------------------
def safe_savefig(path: str, **kwargs):
    """Безопасное сохранение графика с повторными попытками при временных блокировках Windows."""
    import time
    for attempt in range(5):
        try:
            plt.savefig(path, **kwargs)
            return
        except OSError:
            if attempt < 4:
                time.sleep(0.3)
            else:
                raise


def generate_plots(records: list[dict], analysis_a: dict, analysis_b: dict,
                   analysis_odd: dict, s_worst: str, output_dir: str):
    """
    Построение 5 публикационных инженерных графиков строго по выборке состоявшихся ДТП (N_crash)
    и отображению воронки конфликта (N_total).
    """
    plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    n_crash_total = analysis_a['total_crashes'] + analysis_b['total_crashes']
    n_total_all = len(records)
    w_a = analysis_odd['weights']['A']
    w_b = analysis_odd['weights']['B']

    # ==========================================================================
    # ГРАФИК 1: 01_severity_distribution.png
    # Фактическое распределение исходов S0–S3 среди состоявшихся ДТП
    # ==========================================================================
    fig, ax = plt.subplots(figsize=(10.5, 6), dpi=300)
    x = np.arange(len(SEVERITY_CLASSES))
    width = 0.26

    vals_a = [analysis_a['percentages'][s] for s in SEVERITY_CLASSES]
    vals_b = [analysis_b['percentages'][s] for s in SEVERITY_CLASSES]
    vals_odd = [analysis_odd['percentages'][s] for s in SEVERITY_CLASSES]

    rects_a = ax.bar(x - width, vals_a, width, label=f"Подкласс А (Ограждение, n={analysis_a['total_crashes']})",
                     color='#2980b9', edgecolor='black', linewidth=0.8, alpha=0.9)
    rects_b = ax.bar(x, vals_b, width, label=f"Подкласс B (Кювет, n={analysis_b['total_crashes']})",
                     color='#8e44ad', edgecolor='black', linewidth=0.8, alpha=0.9)
    rects_odd = ax.bar(x + width, vals_odd, width, label=f"Взвешенный ODD ({w_a:.2f}A + {w_b:.2f}B)",
                       color='#2c3e50', edgecolor='black', linewidth=0.8, alpha=0.9)

    for rect, s in zip(rects_a, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_a['counts'].get(s, 0)
        ax.annotate(f"{h:.1f}%\n(n={cnt})",
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold', color='#1a5276')

    for rect, s in zip(rects_b, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_b['counts'].get(s, 0)
        ax.annotate(f"{h:.1f}%\n(n={cnt})",
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold', color='#5b2c6f')

    for rect, s in zip(rects_odd, SEVERITY_CLASSES):
        h = rect.get_height()
        ax.annotate(f"{h:.1f}%",
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold', color='#2c3e50')

    max_h = max(vals_a + vals_b + vals_odd + [10.0])
    ax.set_ylim(0, max_h * 1.25)

    ax.set_ylabel('Доля от числа ДТП (%)', fontsize=12, fontweight='bold')
    ax.set_title(f'Распределение тяжести последствий (S0–S3) в состоявшихся ДТП (n = {n_crash_total})',
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xticks(x)
    ax.set_xticklabels(['S0\n(Удар без травм)', 'S1\n(Легкие травмы)', 'S2\n(Тяжелые травмы)', 'S3\n(Опасные для жизни)'],
                       fontsize=10, fontweight='bold')
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, fontsize=10)
    plt.tight_layout()
    plot1_path = os.path.join(plots_dir, "01_severity_distribution.png")
    safe_savefig(plot1_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ==========================================================================
    # ГРАФИК 2: 02_cumulative_iso_decision.png
    # Воронка принятия решения по кумулятивному правилу >10% ISO 26262-3 среди ДТП
    # ==========================================================================
    fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=300)
    steps = [
        'Шаг 1: P(S3 | ДТП)\n[Критические травмы]',
        'Шаг 2: P(≥S2 | ДТП)\n[Тяжелые и критические]',
        'Шаг 3: P(≥S1 | ДТП)\n[Любые травмы]'
    ]
    x = np.arange(len(steps))
    width = 0.25

    vals_a_cum = [analysis_a['cum_percentages']['S3'], analysis_a['cum_percentages']['>=S2'], analysis_a['cum_percentages']['>=S1']]
    vals_b_cum = [analysis_b['cum_percentages']['S3'], analysis_b['cum_percentages']['>=S2'], analysis_b['cum_percentages']['>=S1']]
    vals_odd_cum = [analysis_odd['cum_percentages']['S3'], analysis_odd['cum_percentages']['>=S2'], analysis_odd['cum_percentages']['>=S1']]

    r_a = ax.bar(x - width, vals_a_cum, width, label=f"Подкласс А (Ограждение, итог: {analysis_a['assigned_class']})",
                 color='#2980b9', edgecolor='black', linewidth=0.8)
    r_b = ax.bar(x, vals_b_cum, width, label=f"Подкласс B (Кювет, итог: {analysis_b['assigned_class']})",
                 color='#8e44ad', edgecolor='black', linewidth=0.8)
    r_odd = ax.bar(x + width, vals_odd_cum, width, label=f"Взвешенный ODD (итог: {analysis_odd['assigned_class']})",
                   color='#2c3e50', edgecolor='black', linewidth=0.8)

    ax.axhspan(0, 10.0, color='gray', alpha=0.15, label='Зона отсечки нерепрезентативных исходов ≤ 10%')
    ax.axhline(10.0, color='#e74c3c', linestyle='--', linewidth=2.0, zorder=5,
               label='Порог репрезентативности 10.0% (ISO 26262-3 / SAE J2980:2023)')

    for rect in list(r_a) + list(r_b) + list(r_odd):
        h = rect.get_height()
        ax.annotate(format_iso_pct(h),
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_ylabel('Накопленная вероятность при ДТП (%)', fontsize=12, fontweight='bold')
    ax.set_title('Определение класса тяжести по правилу >10% (ISO 26262-3 / SAE J2980:2023)',
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xticks(x)
    ax.set_xticklabels(steps, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 120)
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.legend(loc='upper left', frameon=True, fontsize=9.5)
    plt.tight_layout()
    plot2_path = os.path.join(plots_dir, "02_cumulative_iso_decision.png")
    safe_savefig(plot2_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ==========================================================================
    # ГРАФИК 3: 03_ditch_phase_portrait.png
    # Двухпанельный фазовый портрет граничных условий в кювете на бровке Y = 9.75 м
    # ==========================================================================
    fig, (ax_trip, ax_perp) = plt.subplots(1, 2, figsize=(16.0, 6.8), dpi=300)
    ditch_points = [r for r in records if is_valid_ditch_departure(r)]
    n_ditch_b = sum(1 for pt in ditch_points if pt['subclass'] == 'SUBCLASS_B_DITCH')
    n_ditch_a_cross = sum(1 for pt in ditch_points if pt['subclass'] == 'SUBCLASS_A_BARRIER')

    # --- ПАНЕЛЬ А: Механизм 1 (Soil-Trip Rollover, Вариант B: V_dep vs beta) ---
    max_v_dep_a = max([pt['departure_speed_mps'] or 0.0 for pt in ditch_points] + [30.0])
    x_max_a = max(32.0, max_v_dep_a * 1.05)
    max_beta = max([pt['sideslip_angle_deg'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_a = max(42.0, max_beta * 1.12)

    xs_grid = np.linspace(0.0, x_max_a, 400)
    # Нижняя граница зоны S2: beta >= 20° при V_dep > 11, либо кривая Cooperrider (V_dep > 14, beta >= 15°, V_lat > 5.5)
    v_trip_start = 5.5 / math.sin(math.radians(20.0))  # ~16.08 м/с (пересечение Cooperrider с beta = 20°)
    v_trip_flat = 5.5 / math.sin(math.radians(15.0))   # ~21.25 м/с (выход Cooperrider на горизонталь 15°)

    def _s2_lower_beta(v_arr):
        beta_trip = np.degrees(np.arcsin(np.minimum(1.0, 5.5 / np.maximum(v_arr, 5.5))))
        return np.where(
            v_arr <= 11.0, y_max_a,
            np.where(v_arr <= 14.0, 20.0,
                     np.minimum(20.0, np.maximum(15.0, beta_trip)))
        )

    ys_s2_lo = _s2_lower_beta(xs_grid)
    ys_s3_lo = np.where(xs_grid > 18.0, 25.0, y_max_a)

    # Заливка прямоугольных/ступенчатых зон классификатора Вариант B
    # S0 / S1 tracking: beta < 15°
    ax_trip.fill_between(xs_grid, 0.0, 15.0, color='#2ecc71', alpha=0.09,
                         label='S0/S1 (без срыва, β < 15°)')
    # S1 skid: 15° <= beta < ys_s2_lo (где S2 ещё не наступило)
    s1_mask = (ys_s2_lo > 15.0)
    ax_trip.fill_between(xs_grid, 15.0, np.minimum(ys_s2_lo, y_max_a), where=s1_mask,
                         color='#f1c40f', alpha=0.18,
                         label='S1: Занос без опрокидывания (β ≥ 15° вне зоны S2)')
    # S2: между ys_s2_lo и ys_s3_lo (при V_dep > 11)
    s2_mask = (xs_grid > 11.0) & (ys_s3_lo > ys_s2_lo)
    ax_trip.fill_between(xs_grid, ys_s2_lo, ys_s3_lo, where=s2_mask,
                         color='#e67e22', alpha=0.22,
                         label='S2: Soil-Trip (V_dep>14, β≥15°, V_lat>5.5) и срыв β≥20° (V_dep>11)')
    # S3: V_dep > 18 и beta >= 25°
    ax_trip.fill_between(xs_grid, 25.0, y_max_a, where=(xs_grid > 18.0),
                         color='#e74c3c', alpha=0.22,
                         label='S3: Тяжёлый срыв (V_dep>18, β≥25° или β≥20° & ω_z>45°/с)')

    # Пороги и кривая инициации Cooperrider beta_trip(V_dep) = arcsin(min(1, 5.5/V_dep))
    ax_trip.axhline(15.0, color='#b7950b', linestyle=':', linewidth=1.2)
    ax_trip.hlines(20.0, 11.0, x_max_a, color='#d35400', linestyle='--', linewidth=1.2)
    ax_trip.hlines(25.0, 18.0, x_max_a, color='#c0392b', linestyle='-', linewidth=1.5)
    ax_trip.vlines(11.0, 20.0, y_max_a, color='#d35400', linestyle='--', linewidth=1.1)
    ax_trip.vlines(14.0, 15.0, y_max_a, color='#d35400', linestyle=':', linewidth=1.0, alpha=0.75)
    ax_trip.vlines(18.0, 25.0, y_max_a, color='#c0392b', linestyle='--', linewidth=1.2)

    xs_coop = np.linspace(14.0, v_trip_flat, 150)
    ys_coop = np.degrees(np.arcsin(np.minimum(1.0, 5.5 / xs_coop)))
    ax_trip.plot(xs_coop, ys_coop, color='#a04000', linestyle='-', linewidth=2.0, zorder=6,
                 label='Кривая Cooperrider: β = arcsin(5.5 / V_dep) → 15° при V_dep≥21.25')
    ax_trip.plot([v_trip_flat, x_max_a], [15.0, 15.0], color='#a04000', linestyle='-', linewidth=2.0, zorder=6)

    # Поясняющие подписи порогов в свободных зонах Панели А
    ax_trip.text(0.6, 15.6, 'β = 15° (S1)', color='#7d6608', fontsize=7.5, ha='left')
    ax_trip.text(x_max_a - 0.5, 20.6, 'β = 20° (S2: V_dep > 11 м/с)', color='#a04000', fontsize=7.5, ha='right')
    ax_trip.text(x_max_a - 0.5, 25.6, 'β = 25° (S3: V_dep > 18 м/с)', color='#922b21', fontsize=7.5, ha='right', fontweight='bold')
    ax_trip.text(11.2, 39.0, '11 м/с', color='#a04000', fontsize=7.5, rotation=90, va='bottom')
    ax_trip.text(18.2, 39.0, '18 м/с', color='#c0392b', fontsize=7.5, rotation=90, va='bottom')
    ax_trip.annotate(
        'Cooperrider: 21.25 м/с → 15°',
        xy=(v_trip_flat, 15.0),
        xytext=(15.0, 10.6),
        ha='center', va='center', fontsize=7.2, color='#a04000', fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='#a04000', lw=0.9)
    )

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        beta_val = pt['sideslip_angle_deg'] if pt['sideslip_angle_deg'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        is_cross_a = (pt['subclass'] == 'SUBCLASS_A_BARRIER')
        ax_trip.scatter(
            v_dep, beta_val, color=c,
            edgecolors='#4d5656' if is_cross_a else 'black',
            linewidths=1.8 if is_cross_a else 0.9,
            linestyles='--' if is_cross_a else '-',
            s=100 if is_cross_a else 85,
            alpha=0.88, marker=marker, zorder=10
        )

    ax_trip.scatter([], [], color='white', marker='o', edgecolors='black', linewidths=0.9, s=75,
                    label=f'Целевой съезд SubB (n={n_ditch_b})')
    ax_trip.scatter([], [], color='white', marker='o', edgecolors='#4d5656', linewidths=1.8, linestyles='--', s=85,
                    label=f'Перекрёстный съезд SubA (пунктир рамки, n={n_ditch_a_cross})')

    annotate_ditch_points(ax_trip, ditch_points, 'departure_speed_mps', 'sideslip_angle_deg', x_max_a, y_max_a, is_panel_a=True)

    ax_trip.set_xlabel('Общая скорость в начале кювета V_dep [м/с]', fontsize=11, fontweight='bold')
    ax_trip.set_ylabel('Угол бокового заноса β [град]', fontsize=11, fontweight='bold')
    ax_trip.set_title('А. Риск опрокидывания при боковом заносе (V_dep, β)', fontsize=12, fontweight='bold')
    ax_trip.set_xlim(0, x_max_a)
    ax_trip.set_ylim(0, y_max_a)
    ax_trip.grid(True, linestyle=':', alpha=0.5)
    ax_trip.legend(loc='upper right', frameon=True, fontsize=7.3)

    # --- ПАНЕЛЬ Б: Механизм 2 (Nose-In / Bottom Impact) ---
    x_max_b = x_max_a
    max_v_perp = max([pt['road_perp_speed_mps'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_b = max(18.5, max_v_perp * 1.28)

    ax_perp.fill_between([8.0, x_max_b], 12.0, y_max_b, color='#e74c3c', alpha=0.20,
                         label='S3: Влет в дно кювета (Severe Nose-In, V⊥ > 12 м/с)')
    ax_perp.fill_between([8.0, x_max_b], 8.0, 12.0, color='#e67e22', alpha=0.20,
                         label='S2: Жесткий удар подрамником (Bottom Impact, 8 < V⊥ ≤ 12 м/с)')
    ax_perp.fill_between([8.0, x_max_b], 2.0, 8.0, color='#f1c40f', alpha=0.18,
                         label='S1: Преодолимый съезд по откосу (Traversable, 2 < V⊥ ≤ 8 м/с)')
    ax_perp.fill_between([0.0, 8.0], 2.0, y_max_b, color='#2ecc71', alpha=0.10)
    ax_perp.fill_between([0.0, x_max_b], 0.0, 2.0, color='#2ecc71', alpha=0.10,
                         label='S0: Касательный / низкоскоростной съезд (V⊥ ≤ 2 м/с или V_dep ≤ 8 м/с)')

    ax_perp.axhline(12.0, color='#c0392b', linestyle='--', linewidth=1.2)
    ax_perp.axhline(8.0, color='#d35400', linestyle='--', linewidth=1.2)
    ax_perp.axhline(2.0, color='#b7950b', linestyle='--', linewidth=1.2)
    ax_perp.axvline(8.0, color='#27ae60', linestyle=':', linewidth=1.2)

    ax_perp.text(x_max_b - 0.5, 12.25, 'Порог S3: V⊥ = 12.0 м/с', color='#c0392b', fontsize=8, ha='right')
    ax_perp.text(0.6, 8.25, 'Порог S2: V⊥ = 8.0 м/с', color='#d35400', fontsize=8, ha='left')
    ax_perp.text(0.6, 2.25, 'Порог S1: V⊥ = 2.0 м/с', color='#b7950b', fontsize=8, ha='left')
    ax_perp.text(8.2, 0.4, 'V_dep = 8.0 м/с', color='#27ae60', fontsize=8, rotation=90)
    ax_perp.text(4.0, 10.0, 'S0: V_dep ≤ 8 м/с', color='#1e8449', fontsize=8, ha='center', va='center', fontweight='bold')

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        v_perp = pt['road_perp_speed_mps'] if pt['road_perp_speed_mps'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        is_cross_a = (pt['subclass'] == 'SUBCLASS_A_BARRIER')
        ax_perp.scatter(
            v_dep, v_perp, color=c,
            edgecolors='#4d5656' if is_cross_a else 'black',
            linewidths=1.8 if is_cross_a else 0.9,
            linestyles='--' if is_cross_a else '-',
            s=100 if is_cross_a else 85,
            alpha=0.88, marker=marker, zorder=10
        )

    ax_perp.scatter([], [], color='gray', marker='o', edgecolors='black', label='Съезд Ego')
    ax_perp.scatter([], [], color='gray', marker='s', edgecolors='black', label='Съезд Target')
    ax_perp.scatter([], [], color='gray', marker='^', edgecolors='black', label='Съезд обоих ТС (Both)')
    ax_perp.scatter([], [], color='white', marker='o', edgecolors='black', linewidths=0.9,
                    label=f'Целевой съезд SubB (n={n_ditch_b})')
    ax_perp.scatter([], [], color='white', marker='o', edgecolors='#4d5656', linewidths=1.8, linestyles='--',
                    label=f'Перекрёстный съезд SubA (пунктир рамки, n={n_ditch_a_cross})')
    ax_perp.scatter([], [], color='none', marker='', label='Цвет маркера — итоговый sev_ditch = max(Механизм А, Механизм Б)')

    annotate_ditch_points(ax_perp, ditch_points, 'departure_speed_mps', 'road_perp_speed_mps', x_max_b, y_max_b, is_panel_a=False)

    ax_perp.set_xlabel('Общая скорость в начале кювета V_dep [м/с]', fontsize=11, fontweight='bold')
    ax_perp.set_ylabel('Скорость сближения с кюветом V⊥ [м/с]', fontsize=11, fontweight='bold')
    ax_perp.set_title('Б. Риск жесткого удара о дно кювета', fontsize=12, fontweight='bold')
    ax_perp.set_xlim(0, x_max_b)
    ax_perp.set_ylim(0, y_max_b)
    ax_perp.grid(True, linestyle=':', alpha=0.5)
    ax_perp.legend(loc='upper left', frameon=True, fontsize=7.4)

    fig.suptitle(
        f'Все сходы в кювет (SubB: {n_ditch_b} + перекрёстные SubA: {n_ditch_a_cross}, всего n = {len(ditch_points)})',
        fontsize=14, fontweight='bold'
    )
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plot3_path = os.path.join(plots_dir, "03_ditch_phase_portrait.png")
    safe_savefig(plot3_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ==========================================================================
    # ГРАФИК 4: 04_subclass_a_dv_cascade.png
    # Двухпанельный каскад скоростей в Подклассе А строго по состоявшимся ДТП
    # ==========================================================================
    sub_a_crashes = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER' and is_crash_run(r)]
    fig, (ax_tt, ax_bar) = plt.subplots(2, 1, figsize=(15, 8.5), dpi=300, sharex=True)

    if sub_a_crashes:
        indices = np.arange(len(sub_a_crashes))
        dv_ego = [r['delta_v_ego_mps'] if r['delta_v_ego_mps'] is not None else 0.0 for r in sub_a_crashes]
        dv_target = [r['delta_v_target_mps'] if r['delta_v_target_mps'] is not None else 0.0 for r in sub_a_crashes]
        v_barrier = [r['barrier_impact_mps'] if r['barrier_impact_mps'] is not None else 0.0 for r in sub_a_crashes]

        # --- ВЕРХНЯЯ ПАНЕЛЬ: Первичный удар ТС-ТС ---
        w_tt = 0.35
        ax_tt.bar(indices - w_tt/2, dv_ego, w_tt, label='Delta-V Ego (Chevrolet Impala, 3490 кг)', color='#5dade2', edgecolor='black', linewidth=0.7)
        ax_tt.bar(indices + w_tt/2, dv_target, w_tt, label='Delta-V Target (Seat Leon, var. mass)', color='#2471a3', edgecolor='black', linewidth=0.7)

        ax_tt.axhline(8.0, color='#c0392b', linestyle='--', linewidth=1.2, label='Порог S3 бокового удара (8.0 м/с)')
        ax_tt.axhline(3.5, color='#d35400', linestyle='--', linewidth=1.2, label='Порог S2 бокового удара (3.5 м/с)')
        ax_tt.axhline(1.0, color='#f39c12', linestyle='--', linewidth=1.2, label='Порог S1 бокового удара (1.0 м/с)')

        max_dv = max(dv_ego + dv_target + [8.0])
        ax_tt.set_ylim(0, max(10.5, max_dv * 1.35))
        for idx, r in enumerate(sub_a_crashes):
            top_val = max(dv_ego[idx], dv_target[idx])
            s_primary = r['sev_side']
            if top_val > 0:
                ax_tt.annotate(f"{top_val:.1f} ({s_primary})", xy=(idx, top_val), xytext=(0, 4), textcoords="offset points",
                               ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#1b4f72', rotation=90)

        ax_tt.set_ylabel('Delta-V ТС-ТС [м/с]', fontsize=11, fontweight='bold')
        ax_tt.set_title(f'А. Первичный боковой удар между автомобилями на полосе (n = {len(sub_a_crashes)})',
                        fontsize=12, fontweight='bold')
        ax_tt.grid(axis='y', linestyle=':', alpha=0.6)
        ax_tt.legend(loc='upper right', frameon=True, fontsize=8.5)

        # --- НИЖНЯЯ ПАНЕЛЬ: Вторичный удар об ограждение ---
        bar_colors = [SEVERITY_COLORS.get(r['sev_barrier'], '#2ecc71') for r in sub_a_crashes]
        is_ditch_s2_flags = [(r.get('severity') == 'S2' and r.get('sev_barrier') != 'S2') for r in sub_a_crashes]
        bar_edges = ['#d35400' if flag else 'black' for flag in is_ditch_s2_flags]
        bar_lws = [2.0 if flag else 0.8 for flag in is_ditch_s2_flags]
        w_b = 0.55
        bars_b = ax_bar.bar(indices, v_barrier, w_b, color=bar_colors, edgecolor=bar_edges, linewidth=bar_lws,
                            label='Скорость удара V_barrier (цвет по классу sev_barrier)')

        l_fr_s3 = ax_bar.axhline(18.0, color='#922b21', linestyle=':', linewidth=1.4, label='FR Порог S3 (18.0 м/с, > 45°)')
        l_ss_s3 = ax_bar.axhline(12.0, color='#c0392b', linestyle='--', linewidth=1.4, label='SS Порог S3 (12.0 м/с, ≤ 45°)')
        l_fr_s2 = ax_bar.axhline(11.0, color='#b9770e', linestyle=':', linewidth=1.4, label='FR Порог S2 (11.0 м/с, > 45°)')
        l_ss_s2 = ax_bar.axhline(6.0, color='#d35400', linestyle='--', linewidth=1.4, label='SS Порог S2 (6.0 м/с, ≤ 45°)')

        short_actor_map = {'target': 'T', 'ego': 'E', 'both': 'B', 'none': 'None'}
        has_ditch_s2_override = any(is_ditch_s2_flags)
        for idx, r in enumerate(sub_a_crashes):
            v_b = v_barrier[idx]
            is_ditch_override = is_ditch_s2_flags[idx]
            if is_valid_barrier_hit(r) and v_b > 0.01:
                act = str(r['barrier_collision_actor']).lower()
                short_actor = short_actor_map.get(act, r['barrier_collision_actor'])
                itype = 'SS' if r['barrier_impact_type'] == 'Side-swipe' else 'FR'
                sev_b = r['sev_barrier']
                txt = f"{v_b:.1f} м/с ({sev_b}, {itype}-{short_actor})"
                if is_ditch_override:
                    txt += " [S2: кювет]"
                ax_bar.annotate(
                    txt, xy=(idx, v_b), xytext=(0, 5), textcoords="offset points",
                    ha='center', va='bottom', fontsize=7.5, fontweight='bold',
                    color='#d35400' if is_ditch_override else '#2c3e50', rotation=90
                )
            elif is_ditch_override:
                ax_bar.annotate(
                    "0.0 м/с [S2: кювет]", xy=(idx, 0.0), xytext=(0, 6), textcoords="offset points",
                    ha='center', va='bottom', fontsize=7.5, fontweight='bold',
                    color='#d35400', rotation=90
                )

        max_v_b = max(v_barrier + [18.0])
        ax_bar.set_ylim(0, max(24.0, max_v_b * 1.45))

        ax_bar.set_xlabel(f'Номер прогона с ДТП в Подклассе А (n = {len(sub_a_crashes)})', fontsize=11, fontweight='bold')
        ax_bar.set_ylabel('Скорость удара V_barrier [м/с]', fontsize=11, fontweight='bold')
        ax_bar.set_title(f'Б. Вторичный удар о жёсткое ограждение с учетом угла удара (n = {len(sub_a_crashes)})',
                         fontsize=12, fontweight='bold')
        ax_bar.set_xticks(indices)
        ax_bar.set_xticklabels([f"#{r['run_id']}" for r in sub_a_crashes], fontsize=7.5, fontweight='bold', rotation=90)
        ax_bar.grid(axis='y', linestyle=':', alpha=0.6)

        import matplotlib.patches as mpatches
        p_s0 = mpatches.Patch(facecolor=SEVERITY_COLORS['S0'], edgecolor='black', label='Удар S0')
        p_s1 = mpatches.Patch(facecolor=SEVERITY_COLORS['S1'], edgecolor='black', label='Удар S1')
        p_s2 = mpatches.Patch(facecolor=SEVERITY_COLORS['S2'], edgecolor='black', label='Удар S2')
        p_s3 = mpatches.Patch(facecolor=SEVERITY_COLORS['S3'], edgecolor='black', label='Удар S3')
        p_ditch_s2 = mpatches.Patch(facecolor='white', edgecolor='#d35400', linewidth=2.0,
                                    label='S2: кювет (перекрёстный уход при sev_barrier≠S2)')
        p_actors = mpatches.Patch(facecolor='none', edgecolor='none', label='Участники: T (Target, var. mass), E (Ego 3490 кг), B (Both)')
        has_s0_bar = any(r.get('sev_barrier') == 'S0' for r in sub_a_crashes)
        handles = ([p_s0] if has_s0_bar else []) + [p_s1, p_s2, p_s3] + ([p_ditch_s2] if has_ditch_s2_override else []) + [l_fr_s3, l_ss_s3, l_fr_s2, l_ss_s2, p_actors]
        ax_bar.legend(handles=handles, loc='upper right', frameon=True, fontsize=8, ncol=2)
    else:
        ax_tt.text(0.5, 0.5, 'Нет прогонов с ДТП для Подкласса А', ha='center', va='center')
        ax_bar.text(0.5, 0.5, 'Нет прогонов с ДТП для Подкласса А', ha='center', va='center')

    fig.suptitle(f'Сравнение скоростей первичного и вторичного ударов (Подкласс А, n = {len(sub_a_crashes)})',
                 fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plot4_path = os.path.join(plots_dir, "04_subclass_a_dv_cascade.png")
    safe_savefig(plot4_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ==========================================================================
    # ГРАФИК 5: 05_kinematics_vs_severity_boxplots.png
    # ДАШБОРД: Эскалация тяжести при ДТП и воронка реализации кинематического конфликта
    # ==========================================================================
    fig, (ax_esc, ax_asym) = plt.subplots(1, 2, figsize=(16.5, 6.8), dpi=300)

    # --- ПАНЕЛЬ А: Эскалация тяжести при ДТП (Взвешенный ODD: 0.20·A + 0.80·B) ---
    crash_records = [r for r in records if is_crash_run(r)]
    n_crash = len(crash_records)
    crash_a = [r for r in crash_records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    crash_b = [r for r in crash_records if r['subclass'] == 'SUBCLASS_B_DITCH']
    n_ca = max(1, len(crash_a))
    n_cb = max(1, len(crash_b))
    weights_odd = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights_odd.get('A', 0.20)
    w_b = weights_odd.get('B', 0.80)

    stage1_counts = {s: sum(1 for r in crash_records if r['sev_side'] == s) for s in SEVERITY_CLASSES}
    stage2_counts = {s: sum(1 for r in crash_records if r['severity'] == s) for s in SEVERITY_CLASSES}

    stage1_pcts = [
        w_a * (sum(1 for r in crash_a if r['sev_side'] == s) / n_ca * 100.0) +
        w_b * (sum(1 for r in crash_b if r['sev_side'] == s) / n_cb * 100.0)
        for s in SEVERITY_CLASSES
    ]
    stage2_pcts = [analysis_odd['percentages'].get(s, 0.0) for s in SEVERITY_CLASSES]

    x_esc = np.arange(len(SEVERITY_CLASSES))
    w_esc = 0.35

    r_s1 = ax_esc.bar(x_esc - w_esc/2, stage1_pcts, w_esc,
                      label=f'Этап 1: Первичный боковой контакт ({w_a:.2f}·A + {w_b:.2f}·B)',
                      color='#3498db', edgecolor='black', linewidth=0.8)
    r_s2 = ax_esc.bar(x_esc + w_esc/2, stage2_pcts, w_esc,
                      label=f'Этап 2: Итог после ограждения / кювета ({w_a:.2f}·A + {w_b:.2f}·B)',
                      color='#e74c3c', edgecolor='black', linewidth=0.8)

    for rect, s in zip(r_s1, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage1_counts[s]
        ax_esc.annotate(f"{h:.1f}%\n(n={cnt})",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 4), textcoords="offset points",
                        ha='center', va='bottom', fontsize=8.8, fontweight='bold', color='#1b4f72')

    for rect, s in zip(r_s2, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage2_counts[s]
        ax_esc.annotate(f"{h:.1f}%\n(n={cnt})",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 4), textcoords="offset points",
                        ha='center', va='bottom', fontsize=8.8, fontweight='bold', color='#922b21')

    max_esc = max(stage1_pcts + stage2_pcts + [10.0])
    ax_esc.set_ylim(0, max_esc * 1.25)
    ax_esc.set_ylabel(f'Взвешенная доля по ODD ({w_a:.2f}·A + {w_b:.2f}·B), %', fontsize=10.5, fontweight='bold')
    ax_esc.set_title(f'А. Тяжесть травм: первичный контакт vs итог ДТП\n(Взвешенный ODD: {w_a:.2f}·A + {w_b:.2f}·B, n = {n_crash})',
                     fontsize=11, fontweight='bold', pad=10)
    ax_esc.set_xticks(x_esc)
    ax_esc.set_xticklabels(['S0 (Без травм)', 'S1 (Легкие)', 'S2 (Тяжелые)', 'S3 (Фатальные)'],
                           fontsize=9.5, fontweight='bold')
    ax_esc.grid(axis='y', linestyle=':', alpha=0.6)
    ax_esc.legend(loc='upper right', frameon=True, fontsize=8.2)

    # --- ПАНЕЛЬ Б: Воронка реализации конфликта в слепой зоне и распределение пострадавших ---
    actors_cat = [
        'Оба ТС\n(Both)',
        'Только Target\n(Seat Leon)',
        'Только Ego\n(Impala 3490 кг)',
        'Только контакт\nТС–ТС (без\nбарьера/кювета)',
        'Разъехались\nбез удара'
    ]

    sub_a_runs = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    sub_b_runs = [r for r in records if r['subclass'] == 'SUBCLASS_B_DITCH']
    tot_a = max(1, len(sub_a_runs))
    tot_b = max(1, len(sub_b_runs))

    def _norm_actor(val):
        return str(val or '').strip().lower()

    counts_a_act = {
        'both': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'both'),
        'target': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'target'),
        'ego': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'ego'),
        'car_only': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) not in ('both', 'target', 'ego')),
        'safe': sum(1 for r in sub_a_runs if not is_crash_run(r) and not is_spawn_error_run(r))
    }

    counts_b_act = {
        'both': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'both'),
        'target': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'target'),
        'ego': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'ego'),
        'car_only': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) not in ('both', 'target', 'ego')),
        'safe': sum(1 for r in sub_b_runs if not is_crash_run(r) and not is_spawn_error_run(r))
    }

    order_keys = ['both', 'target', 'ego', 'car_only', 'safe']
    pcts_a_act = [(counts_a_act[k] / tot_a) * 100.0 for k in order_keys]
    pcts_b_act = [(counts_b_act[k] / tot_b) * 100.0 for k in order_keys]

    x_act = np.arange(len(actors_cat))
    w_act = 0.36

    r_a_act = ax_asym.bar(x_act - w_act/2, pcts_a_act, w_act, label=f'Подкласс А (Ограждение, N={len(sub_a_runs)})',
                          color='#2980b9', edgecolor='black', linewidth=0.8)
    r_b_act = ax_asym.bar(x_act + w_act/2, pcts_b_act, w_act, label=f'Подкласс B (Кювет, N={len(sub_b_runs)})',
                          color='#8e44ad', edgecolor='black', linewidth=0.8)

    for rect, k in zip(r_a_act, order_keys):
        h = rect.get_height()
        cnt = counts_a_act[k]
        ax_asym.annotate(f"{h:.1f}%\n(n={cnt})",
                         xy=(rect.get_x() + rect.get_width() / 2, h),
                         xytext=(0, 4), textcoords="offset points",
                         ha='center', va='bottom', fontsize=8.2, fontweight='bold', color='#1b4f72')

    for rect, k in zip(r_b_act, order_keys):
        h = rect.get_height()
        cnt = counts_b_act[k]
        ax_asym.annotate(f"{h:.1f}%\n(n={cnt})",
                         xy=(rect.get_x() + rect.get_width() / 2, h),
                         xytext=(0, 4), textcoords="offset points",
                         ha='center', va='bottom', fontsize=8.2, fontweight='bold', color='#512e5f')

    max_asym = max(pcts_a_act + pcts_b_act + [10.0])
    ax_asym.set_ylim(0, max_asym * 1.25)
    ax_asym.set_ylabel('Доля от всех заездов (%)', fontsize=10.5, fontweight='bold')
    ax_asym.set_title(f'Б. Итоги всех заездов и участники ДТП (N = {n_total_all}, сумма = 100%)',
                      fontsize=11, fontweight='bold', pad=10)
    ax_asym.set_xticks(x_act)
    ax_asym.set_xticklabels(actors_cat, fontsize=8.2, fontweight='bold')
    ax_asym.grid(axis='y', linestyle=':', alpha=0.6)
    ax_asym.legend(loc='upper right', frameon=True, fontsize=8.2)

    fig.suptitle('Рост тяжести травм при вторичных ударах и общая статистика заездов',
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.02, 1, 0.93])
    plt.subplots_adjust(wspace=0.22)
    plot5_path = os.path.join(plots_dir, "05_kinematics_vs_severity_boxplots.png")
    safe_savefig(plot5_path, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"  [OK] Все 5 обновленных графиков успешно сохранены в: {plots_dir}")


# ------------------------------------------------------------------------------
# Шаблонизация и генерация Markdown-отчета summary_report.md
# ------------------------------------------------------------------------------
DEFAULT_REPORT_TEMPLATE = r"""# ОТЧЕТ ПО АНАЛИЗУ ОПАСНОСТЕЙ И ОЦЕНКЕ РИСКОВ (HARA)
## Функция: Пассивный контроль слепых зон при смене полосы (Lane Change Control, LCC / BSD — без вмешательства в управление)
### Стандарты: **ISO 26262-3:2018** (Clause 6, Annex B), **SAE J2980:2023** (5.2.3, Annex B)

---

## 1. РЕЗЮМЕ ОЦЕНКИ И ИТОГОВЫЕ ВЕРДИКТЫ ТЯЖЕСТИ (SEVERITY)

* **Исследуемый функциональный отказ:** Необнаружение попутного транспортного средства в слепой зоне при начале маневра перестроения на загородной автомагистрали ($100\text{ км/ч}$).
* **Общий объем стохастического ансамбля сценариев:** $N_{total} = {total_runs}$ прогонов (Подкласс А: $N_A = {runs_a}$, Подкласс B: $N_B = {runs_b}$).
* **Репрезентативная выборка состоявшихся ДТП (Accident-Involved Runs):** $N_{crash} = {total_crashes}$ прогонов (Подкласс А: $n_A = {crashes_a}$, Подкласс B: $n_B = {crashes_b}$). Классификация тяжести последствий (Severity, S) по стандарту ISO 26262-3 / SAE J2980:2023 выносится строго по данной выборке.

**Таблица физических параметров сценария и дорожных условий ODD:**

| Параметр моделирования | Физическое значение | Обоснование / Нормативный стандарт |
| :--- | :---: | :--- |
| **Масса Ego (Chevrolet Impala)** | 3490 кг | Спец. масса защищаемого ОА представительского класса (аналог Aurus Senat) |
| **Масса Target (Seat Leon)** | `TruncatedNormal(1480, 220, [1100, 2050])` кг | Массовый C-класс; $m_{ego}/m_{tgt} \in [1.70; 3.17]$ (EEA 2023/2024, автопарк РФ) |
| **Начальная крейсерская скорость** | 100 км/ч (27.78 м/с) | Типовая крейсерская скорость загородной автомагистрали |
| **Профиль кювета (Подкласс B)** | Обочина $2.25$ м (консерват. отн. ГОСТ $3.0\text{–}3.5$ м), откос 1:4, $Z = -1.20$ м | Водоотводный кювет по СП 34.13330.2021 п. 7.27 / AASHTO RDG; граничные условия $Y = 9.75$ м (оба подкласса) |
| **Ограждение (Подкласс А)** | Жёсткое вертикальное ограждение $0.80$ м ($Y \le 0.0$ м) | Вертикальная жёсткая стенка (ГОСТ 33128-2014), учёт проекции скорости строго внутрь барьера |
| **Сцепление и тормоза (`Config F`)** | `FrictionScale`: `1.05` (асфальт, ~`0.81–0.86 g`), `0.57` (обочина/кювет $Y \ge 7.50$ м до `32 м`, ~`0.46–0.48 g`); EBD `3800/2050 Н·м` (`Ego`) и `1.00/0.54·m` (`Target`), `use_sweep_wheel_collision = True` | Пропорциональный EBD (~65/35%), sweep-коллайдеры колес, инвариантное замедление `Target` и сохранение управляемости без оракулов `loc_y` (FMVSS 135 / SAE 830612) |
| **Модель реакции и удержания** | $T_{react} \in [0.65, 1.60]$ с (мед. $1.05$ с), $T_{hold} \in [0.20, 0.35]$ с | Усечённо-логнормальная задержка (Green 2000; Taoka 1989; SAE J2980:2023 5.2.3.2(d)) |
| **Шаг и горизонт симуляции** | $dt = 0.01$ с (100 Гц, Substepping 100 Гц), горизонт $45$ с ($4500$ тиков) | Полный разгон `Impala 3490 кг` до $100\text{ км/ч}$ и завершение пост-ударной фазы без обрезки |

### 1.1. Многоуровневые вердикты по стандарту ISO 26262-3 / SAE J2980:2023:
1. **Вердикт для Подкласса А (Ограждение слева, $n_A = {crashes_a}$):** **Класс {assigned_class_a}**
   * *Обоснование:* {decision_reason_a}
2. **Вердикт для Подкласса B (Кювет справа, $n_B = {crashes_b}$):** **Класс {assigned_class_b}**
   * *Обоснование:* {decision_reason_b}
3. **Консервативный вердикт по сценарию (Worst-Case Subclass):** **Класс {s_worst}**
   * *Определение:* $S_{worst} = \max(S_A, S_B) = \max({assigned_class_a}, {assigned_class_b}) = {s_worst_bold}$.
   * *Область применения:* Используется при раздельном ведении подклассов дорожных условий в таблице HARA (где веса ${weight_a}$ и ${weight_b}$ уже учтены в классе Экспозиции $E$).
4. **Взвешенный вердикт по объединенному ODD (Пул ODD, $N_{crash} = {total_crashes}$):** **Класс {assigned_class_odd}**
   * *Определение:* По условным взвешенным вероятностям $P_{ODD}(S_i \mid \text{ДТП}) = {weight_a} \cdot P_A(S_i \mid \text{ДТП}) + {weight_b} \cdot P_B(S_i \mid \text{ДТП})$.
   * *Обоснование:* {decision_reason_odd}

### 1.2. Разрешение парадокса весов ODD (Double Weighting Paradox)
{odd_paradox_content}

### 1.3. Статистика реализации кинематического конфликта в слепой зоне (Pre-Crash Conflict Funnel)
{conflict_funnel_content}

### 1.4. Детальный анализ критических исходов S3 и доверительные интервалы
{s3_analysis_content}

### 1.5. Количественная оценка управляемости (Controllability, C) по кампании `active`
{controllability_section}

### 1.6. Итоговая матрица классификации опасного события и назначение ASIL (ISO 26262-3)
{asil_matrix_content}

---

## 2. СВОДНАЯ ТАБЛИЦА РАСПРЕДЕЛЕНИЯ И КУМУЛЯТИВНЫХ ВЕРОЯТНОСТЕЙ

{table_severity_distribution}

---

## 3. СВОДНЫЙ ПРОТОКОЛ ПРИМЕНЕНИЯ ПРАВИЛА >10% (ISO 26262-3 / SAE J2980:2023)

Кумулятивная проверка порога репрезентативности $10.0\%$ проводится строго сверху вниз по выборке состоявшихся ДТП:
{protocol_steps_content}

---

## 4. СВОДНАЯ СТАТИСТИКА КИНЕМАТИЧЕСКИХ ПАРАМЕТРОВ СИМУЛЯЦИЙ

{table_kinematics_stats}

---

## 5. АНАЛИЗ МЕХАНИЗМОВ АВАРИЙ И АСИММЕТРИИ УЯЗВИМОСТИ УЧАСТНИКОВ

### 5.1. Подкласс А (Удары о жёсткое ограждение $0.80\text{ м}$)
{barrier_mechanisms_content}

### 5.2. Подкласс B (Механизмы схода в кювет на бровке $Y = 9.75\text{ м}$)
{ditch_mechanisms_content}

### 5.3. Асимметрия уязвимости при соотношении масс ($m_{ego}/m_{tgt} \in [1.70; 3.17]$)
При столкновении тяжёлого седана Ego ($3490\text{ кг}$) с легковым Target (`Seat Leon`, $1100\text{–}2050\text{ кг}$):
{actor_asymmetry_content}

---

## 6. ОБНОВЛЕННЫЕ ИНЖЕНЕРНЫЕ ГРАФИКИ

В директории `results/plots/` сформированы 5 публикационных графиков (300 DPI):
1. `01_severity_distribution.png` — Фактическое распределение исходов S0–S3 среди состоявшихся ДТП ($N_{crash} = {total_crashes}$).
2. `02_cumulative_iso_decision.png` — Воронка кумулятивного правила >10% для состоявшихся ДТП с серой зоной отсечки и нормативным порогом 10.0%.
3. `03_ditch_phase_portrait.png` — Двухпанельный фазовый портрет кювета: Механизм 1 (Soil-Trip) и Механизм 2 (Nose-In Impact) по всем сходам в кювет (SubB: {ditch_departures_b} + перекрёстные SubA: {ditch_departures_a_cross}, всего $n = {ditch_departures_total}$) с разметкой #run_id.
4. `04_subclass_a_dv_cascade.png` — Двухпанельный каскад скоростей удара в Подклассе А строго по состоявшимся ДТП ($n = {crashes_a}$) без пустых пропусков (ТС–ТС vs Ограждение).
5. `05_kinematics_vs_severity_boxplots.png` — Дашборд эскалации тяжести при ДТП (Панель А, $N_{crash} = {total_crashes}$) и воронка реализации конфликта в слепой зоне с распределением участников (Панель Б, $N_{total} = {total_runs}$).

---
*Отчет сформирован автоматически модулем `analyze_results.py` на основе физического моделирования CARLA / Scenic.*
"""


def render_template(template_str: str, context: dict) -> str:
    """
    Безопасная подстановка переменных {var_name} в шаблон Markdown.
    Игнорирует фигурные скобки KaTeX/LaTeX, заменяя только совпадения с ключами context.
    """
    pattern = re.compile(r'\{([a-zA-Z0-9_]+)\}')

    def _replace(match):
        key = match.group(1)
        if key in context:
            return str(context[key])
        return match.group(0)

    return pattern.sub(_replace, template_str)


def resolve_template_path(specified_path: str = None) -> str:
    """
    Определение пути к файлу шаблона отчета report_template.md.
    Приоритет поиска:
      1. Явно указанный путь (CLI-аргумент --template)
      2. templates/report_template.md
      3. report_template.md (в корне проекта)
    Если ни один файл не существует, автоматически создается templates/report_template.md
    с дефолтным содержимым DEFAULT_REPORT_TEMPLATE.
    """
    if specified_path:
        if os.path.isfile(specified_path):
            return specified_path
        else:
            print(f"  [WARN] Указанный файл шаблона '{specified_path}' не найден. Поиск шаблона по умолчанию...")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_paths = [
        os.path.join(base_dir, "templates", "report_template.md"),
        os.path.join(base_dir, "report_template.md"),
        os.path.join("templates", "report_template.md"),
        "report_template.md"
    ]
    for p in default_paths:
        if os.path.isfile(p):
            return p

    # Создание базового шаблона по умолчанию
    fallback_path = os.path.join(base_dir, "templates", "report_template.md")
    os.makedirs(os.path.dirname(fallback_path), exist_ok=True)
    with open(fallback_path, mode='w', encoding='utf-8') as f:
        f.write(DEFAULT_REPORT_TEMPLATE.strip() + "\n")
    print(f"  [INFO] Базовый шаблон отчета создан: {fallback_path}")
    return fallback_path


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


def _build_conflict_funnel_content(analysis_a: dict, analysis_b: dict) -> str:
    """Формирование текстового блока воронки реализации кинематического конфликта (Раздел 1.3)."""
    total_runs = analysis_a['total_runs'] + analysis_b['total_runs']
    total_crashes = analysis_a['total_crashes'] + analysis_b['total_crashes']
    total_safe = analysis_a['safe_pass_runs'] + analysis_b['safe_pass_runs']
    spawn_err_a = analysis_a.get('spawn_error_runs', 0)
    spawn_err_b = analysis_b.get('spawn_error_runs', 0)
    total_spawn_err = spawn_err_a + spawn_err_b
    p_total_conflict = (total_crashes / max(1, total_runs)) * 100.0
    p_total_safe = (total_safe / max(1, total_runs)) * 100.0
    p_total_err = (total_spawn_err / max(1, total_runs)) * 100.0

    err_str_a = f" Ошибок спавна акторов (`SimulationCreationError`): **{spawn_err_a}** ({spawn_err_a/max(1, analysis_a['total_runs'])*100:.1f}%)." if spawn_err_a > 0 else ""
    err_str_b = f" Ошибок спавна акторов (`SimulationCreationError`): **{spawn_err_b}** ({spawn_err_b/max(1, analysis_b['total_runs'])*100:.1f}%)." if spawn_err_b > 0 else ""
    err_str_tot = f", а в **{total_spawn_err} прогонах** ({p_total_err:.1f}%) зафиксирован отказ генерации сцены (`SimulationCreationError: Unable to spawn object`)" if total_spawn_err > 0 else ""

    safe_word = plural_ru(total_safe, ('прогон', 'прогона', 'прогонов'))
    safe_verb = 'завершился' if (total_safe % 10 == 1 and total_safe % 100 != 11) else 'завершились'
    lines = [
        f"* **Подкласс А (Ограждение слева):** Сгенерировано **{analysis_a['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_A$): **{analysis_a['total_crashes']}** ({analysis_a['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_a['safe_pass_runs']}** ({analysis_a['safe_pass_runs']/max(1, analysis_a['total_runs'])*100:.1f}%).{err_str_a}",
        f"* **Подкласс B (Кювет справа):** Сгенерировано **{analysis_b['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_B$): **{analysis_b['total_crashes']}** ({analysis_b['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_b['safe_pass_runs']}** ({analysis_b['safe_pass_runs']/max(1, analysis_b['total_runs'])*100:.1f}%).{err_str_b}",
        f"* **Суммарный ансамбль по ODD:** Из $N_{{total}} = {total_runs}$ стохастических прогонов состоялось **$N_{{crash}} = {total_crashes}$ ДТП** ({p_total_conflict:.1f}%), **{total_safe} {safe_word}** ({p_total_safe:.1f}%) {safe_verb} безопасным опережением без физического контакта ТС{err_str_tot}."
    ]

    if total_safe > 0:
        lines.extend([
            "",
            "**Физическая природа безаварийных разъездов (Safe Pass):**",
            "В соответствии с ODD диапазон продольного смещения спавна Target (`Seat Leon`, $4.19\\text{ м}$) относительно Ego (`Chevrolet Impala`, $5.36\\text{ м}$) охватывает всю область перекрытия слепой зоны (`target_offset_y` от $-5.680$ до $-1.415\\text{ м}$). При генерации Target в дальней кормовой части диапазона динамика перестроения Ego на скорости $100\\text{ км/ч}$ ($27.78\\text{ м/с}$) в ряде сцен позволяет Ego завершить смещение на смежную полосу с опережением и без физического контакта кузовов."
        ])
    else:
        lines.extend([
            "",
            "**Реализация конфликта:** В текущей выборке все валидные сценарии привели к физическому столкновению (`Safe Pass = 0`)."
        ])
    return "\n".join(lines)


def _build_severity_table(analysis_a: dict, analysis_b: dict, analysis_odd: dict, s_worst: str) -> str:
    """Построение сводной таблицы распределения и кумулятивных вероятностей (Раздел 2)."""
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    lines = [
        r"| Дорожный профиль ODD | Вес в ODD | N_total (Конфликт %) | N_crash | S0 (Удар без травм, AIS 0) | S1 (Легкие) | S2 (Тяжелые) | S3 (Фатальные) | P(S3 \| ДТП) | P(≥S2 \| ДТП) | P(≥S1 \| ДТП) | Вердикт ISO 26262 |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    # Строка Подкласса А
    c_a = analysis_a['counts']
    p_a = analysis_a['percentages']
    cp_a = analysis_a['cum_percentages']
    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    lines.append(f"| **Подкласс А (Ограждение)** | {w_a:.2f} | {n_tot_a_str} | {analysis_a['total_crashes']} | "
                 f"{c_a.get('S0',0)} ({p_a.get('S0',0.0):.1f}%) | "
                 f"{c_a.get('S1',0)} ({p_a.get('S1',0.0):.1f}%) | "
                 f"{c_a.get('S2',0)} ({p_a.get('S2',0.0):.1f}%) | "
                 f"{c_a.get('S3',0)} ({p_a.get('S3',0.0):.1f}%) | "
                 f"{format_iso_pct(cp_a['S3'])} | {format_iso_pct(cp_a['>=S2'])} | {format_iso_pct(cp_a['>=S1'])} | **{analysis_a['assigned_class']}** |")

    # Строка Подкласса B
    c_b = analysis_b['counts']
    p_b = analysis_b['percentages']
    cp_b = analysis_b['cum_percentages']
    n_tot_b_str = f"{analysis_b['total_runs']} ({analysis_b['conflict_rate']:.1f}%)"
    lines.append(f"| **Подкласс B (Кювет)** | {w_b:.2f} | {n_tot_b_str} | {analysis_b['total_crashes']} | "
                 f"{c_b.get('S0',0)} ({p_b.get('S0',0.0):.1f}%) | "
                 f"{c_b.get('S1',0)} ({p_b.get('S1',0.0):.1f}%) | "
                 f"{c_b.get('S2',0)} ({p_b.get('S2',0.0):.1f}%) | "
                 f"{c_b.get('S3',0)} ({p_b.get('S3',0.0):.1f}%) | "
                 f"**{format_iso_pct(cp_b['S3'])}** | **{format_iso_pct(cp_b['>=S2'])}** | {format_iso_pct(cp_b['>=S1'])} | **{analysis_b['assigned_class']}** |")

    # Строка Взвешенного ODD
    p_odd = analysis_odd['percentages']
    cp_odd = analysis_odd['cum_percentages']
    tot_runs = analysis_odd['total_runs']
    tot_crashes = analysis_odd['total_crashes']
    p_conf_odd = (tot_crashes / max(1, tot_runs)) * 100.0
    n_tot_odd_str = f"{tot_runs} ({p_conf_odd:.1f}%)"
    lines.append(f"| **ВЗВЕШЕННЫЙ ODD** | **1.00** | **{n_tot_odd_str}** | **{tot_crashes}** | "
                 f"{p_odd.get('S0',0.0):.1f}% | "
                 f"{p_odd.get('S1',0.0):.1f}% | "
                 f"{p_odd.get('S2',0.0):.1f}% | "
                 f"{p_odd.get('S3',0.0):.1f}% | "
                 f"{format_iso_pct(cp_odd['S3'])} | **{format_iso_pct(cp_odd['>=S2'])}** | {format_iso_pct(cp_odd['>=S1'])} | **{analysis_odd['assigned_class']}** |")

    # Итоговая строка Worst-Case
    lines.append(f"| **WORST-CASE SUBCLASS** | - | **{n_tot_odd_str}** | **{tot_crashes}** | "
                 f"- | - | - | - | "
                 f"- | - | - | **{s_worst}** |")

    return "\n".join(lines)


def _build_iso_steps_content(analysis_a: dict, analysis_b: dict, analysis_odd: dict) -> str:
    """Построение компактной сводной аналитики применения правила >10% (Раздел 3) для любого класса S0–S3."""
    def format_subclass_summary(name: str, an: dict, ci_str: str = "") -> list[str]:
        cp = an['cum_percentages']
        p3 = cp.get('S3', 0.0)
        p2 = cp.get('>=S2', 0.0)
        p1 = cp.get('>=S1', 0.0)
        crashes = an['total_crashes']
        assigned = an['assigned_class']

        p3_str = format_iso_pct(p3).replace('%', '\\%')
        p2_str = format_iso_pct(p2).replace('%', '\\%')
        p1_str = format_iso_pct(p1).replace('%', '\\%')

        step_lines = [f"* **{name} (выборка ДТП $n = {crashes}$):**"]
        if crashes == 0:
            step_lines.append("  * Выборки состоявшихся ДТП отсутствуют (по умолчанию **Класс S0**).")
            step_lines.append(f"  * **Итоговый вердикт:** **Класс {assigned}**.")
            return step_lines

        if assigned == 'S3':
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 1**: $P(S3 \\mid \\text{{ДТП}}) = {p3_str}{ci_str} > 10.0\\%$ "
                f"— порог превышен на **{format_iso_margin(p3 - 10.0)}**."
            )
        elif assigned == 'S2':
            step_lines.append(
                f"  * $P(S3 \\mid \\text{{ДТП}}) = {p3_str}{ci_str} \\le 10.0\\%$ — нормативный порог фатальных травм не достигнут."
            )
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 2**: $P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_str} > 10.0\\%$ "
                f"— порог преодолен с запасом **{format_iso_margin(p2 - 10.0)}**."
            )
        elif assigned == 'S1':
            step_lines.append(
                f"  * $P(S3 \\mid \\text{{ДТП}}) = {p3_str}{ci_str} \\le 10.0\\%$, "
                f"$P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_str} \\le 10.0\\%$ — пороги S3 и S2 не достигнуты."
            )
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 3**: $P(\\ge S1 \\mid \\text{{ДТП}}) = {p1_str} > 10.0\\%$ "
                f"— порог преодолен на **{format_iso_margin(p1 - 10.0)}**."
            )
        else:
            step_lines.append(
                f"  * Кумулятивные вероятности $P(S3) = {p3_str}{ci_str}$, $P(\\ge S2) = {p2_str}$ и "
                f"$P(\\ge S1 \\mid \\text{{ДТП}}) = {p1_str} \\le 10.0\\%$ — ни один порог вреда не превышен (**Шаг 4**)."
            )

        step_lines.append(f"  * **Итоговый вердикт:** **Класс {assigned}**.")
        return step_lines

    # Расчет доверительного интервала для S3 в Подклассе B
    k_s3_b = analysis_b['counts'].get('S3', 0)
    n_b = analysis_b['total_crashes']
    ci_low_b, ci_high_b = calc_clopper_pearson_ci(k_s3_b, n_b)
    ci_b_str = f" (95% CI: [{ci_low_b*100:.2f}%, {ci_high_b*100:.2f}%])"

    lines = []
    lines.extend(format_subclass_summary("Подкласс А (Ограждение слева)", analysis_a))
    lines.append("")
    lines.extend(format_subclass_summary("Подкласс B (Кювет справа)", analysis_b, ci_b_str))
    lines.append("")
    lines.extend(format_subclass_summary(f"Взвешенный ODD (Пул ODD, $N_{{crash}} = {analysis_odd['total_crashes']}$)", analysis_odd))

    return "\n".join(lines)


def _build_kinematics_table(records: list[dict]) -> str:
    """Построение сводной таблицы кинематических параметров (Раздел 4)."""
    stats_react = calc_stats([r['shock_duration_sec'] for r in records if r['shock_duration_sec'] is not None])
    stats_dv_side = calc_stats([r['effective_side_dv_mps'] for r in records if r['effective_side_dv_mps'] is not None])
    stats_barrier_v = calc_stats([r['barrier_impact_mps'] for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER' and is_valid_barrier_hit(r)])
    stats_dep_v = calc_stats([r['departure_speed_mps'] for r in records if is_valid_ditch_departure(r)])
    stats_perp_v = calc_stats([r['road_perp_speed_mps'] for r in records if is_valid_ditch_departure(r)])
    stats_beta = calc_stats([r['sideslip_angle_deg'] for r in records if is_valid_ditch_departure(r)])
    stats_yaw = calc_stats([r['departure_yaw_rate_deg_s'] for r in records if is_valid_ditch_departure(r)])

    lines = [
        "| Физический параметр | Число замеров | Среднее | Медиана | Станд. откл. (Std) | Минимум | Максимум | 90-й перцентиль |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    def format_row(name, s, unit):
        return f"| {name} [{unit}] | {s['count']} | {s['mean']:.2f} | {s['median']:.2f} | {s['std']:.2f} | {s['min']:.2f} | {s['max']:.2f} | {s['p90']:.2f} |"

    lines.append(format_row("Время реакции водителя $T_{react}$", stats_react, "с"))
    lines.append(format_row("Эффективная $\\Delta V$ бокового удара ТС-ТС", stats_dv_side, "м/с"))
    lines.append(format_row("Скорость вторичного удара в ограждение $V_{barrier}$", stats_barrier_v, "м/с"))
    lines.append(format_row("Полная скорость схода в кювет $V_{dep}$", stats_dep_v, "м/с"))
    lines.append(format_row("Нормальная скорость ухода в кювет $V_{\\perp}$", stats_perp_v, "м/с"))
    lines.append(format_row("Угол бокового заноса кузова $\\beta$", stats_beta, "град"))
    lines.append(format_row("Угловая скорость рыскания $\\omega_z$", stats_yaw, "град/с"))

    return "\n".join(lines)


def _build_odd_paradox_content(analysis_a: dict, analysis_b: dict, analysis_odd: dict, s_worst: str) -> str:
    """Формирование вывода о соотношении вердиктов Worst-Case и взвешенного ODD (Раздел 1.2)."""
    p2_b = analysis_b['cum_percentages']['>=S2']
    p2_a = analysis_a['cum_percentages']['>=S2']
    s_a = analysis_a['assigned_class']
    s_b = analysis_b['assigned_class']
    s_odd = analysis_odd['assigned_class']
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    if s_worst == s_odd:
        text = (
            f"Оба метода агрегации дают согласованный итоговый класс тяжести ($S_{{worst}} = S_{{ODD}} = {s_worst}$): "
            f"в Подклассе А (ограждение) получено **$S_A = {s_a}$** ($P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_a:.1f}\\%$), "
            f"в Подклассе B (кювет) — **$S_B = {s_b}$** ($P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_b:.1f}\\%$). "
            f"В соответствии с ISO 26262-3 (Clause 6) в рабочую таблицу HARA подклассы заносятся раздельными строками "
            f"(`1.1.A` и `1.1.B`), поскольку дорожные веса $W_A = {w_a:.2f}$ и $W_B = {w_b:.2f}$ уже учитываются в экспозиции $E$."
        )
    else:
        text = (
            f"Консервативный вердикт по наихудшему подклассу ($S_{{worst}} = \\max({s_a}, {s_b}) = {s_worst}$) расходится "
            f"со взвешенным вердиктом объединенного пула ($S_{{ODD}} = {s_odd}$ при весах $W_A = {w_a:.2f}$, $W_B = {w_b:.2f}$; "
            f"$P_A(\\ge S2) = {p2_a:.1f}\\%$, $P_B(\\ge S2) = {p2_b:.1f}\\%$). Во избежание двойного взвешивания вероятностей ODD "
            f"(Double Weighting Paradox) стандарт ISO 26262-3 (Clause 6) предписывает заносить подклассы `1.1.A` и `1.1.B` раздельными строками HARA "
            f"или принимать консервативный верхний класс $S_{{worst}} = {s_worst}$."
        )
    return text


def _build_s3_analysis_content(analysis_b: dict, records_b: list[dict]) -> str:
    """Формирование детального физико-статистического разбора исходов S3 и доверительных интервалов (Раздел 1.4)."""
    k_s3 = analysis_b['counts'].get('S3', 0)
    n_crashes = analysis_b['total_crashes']
    p_s3 = (k_s3 / max(1, n_crashes)) * 100.0
    ci_low, ci_high = calc_clopper_pearson_ci(k_s3, n_crashes, confidence=0.95)
    s3_runs = [r for r in records_b if is_crash_run(r) and r.get('severity') == 'S3']

    rollover_cnt = sum(1 for r in s3_runs if 'rollover' in str(r.get('ditch_mechanism', '')).lower())
    nose_in_cnt = sum(1 for r in s3_runs if 'nose-in' in str(r.get('ditch_mechanism', '')).lower())
    other_cnt = len(s3_runs) - rollover_cnt - nose_in_cnt

    mech_parts = []
    if rollover_cnt > 0:
        mech_parts.append(f"{rollover_cnt} {plural_ru(rollover_cnt, ('заезд', 'заезда', 'заездов'))} критического бокового срыва (High-Speed Soil-Trip Rollover Risk)")
    if nose_in_cnt > 0:
        mech_parts.append(f"{nose_in_cnt} {plural_ru(nose_in_cnt, ('заезд', 'заезда', 'заездов'))} крутого продольного утыкания (Severe Ditch Nose-In Impact при $V_\\perp > 12\\text{{ м/с}}$)")
    if other_cnt > 0:
        mech_parts.append(f"{other_cnt} {plural_ru(other_cnt, ('заезд', 'заезда', 'заездов'))} прочих механизмов")

    if k_s3 == 0:
        s3_mech_sentence = "В данной выборке состоявшихся ДТП критические исходы S3 полностью отсутствуют ($k_{S3} = 0$)."
    elif k_s3 == 1:
        s3_mech_sentence = f"Единственный заезд S3 зафиксирован по граничному условию на бровке $Y = 9.75\\text{{ м}}$: {' и '.join(mech_parts)}."
    else:
        s3_mech_sentence = f"Все {k_s3} {plural_ru(k_s3, ('заезд', 'заезда', 'заездов'))} S3 зафиксированы по граничному условию на бровке $Y = 9.75\\text{{ м}}$: {' и '.join(mech_parts)}."

    if ci_high * 100.0 < 10.0:
        ci_verdict = (
            f"Верхняя граница 95% доверительного интервала (**{ci_high*100:.2f}%**) строго не достигает нормативного порога **10.0%** "
            f"(ISO 26262-3 / SAE J2980). Это статистически доказывает, что класс S3 является нерепрезентативным для данного ODD. {s3_mech_sentence}"
        )
    elif p_s3 <= 10.0:
        ci_verdict = (
            f"Точечная оценка вероятности $P(S3 \\mid \\text{{ДТП}}) = {p_s3:.2f}\\%$ не превышает порога **10.0%** "
            f"(95% доверительный интервал Клоппера–Пирсона: $[{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$). {s3_mech_sentence}"
        )
    else:
        ci_verdict = (
            f"Точечная оценка вероятности $P(S3 \\mid \\text{{ДТП}}) = {p_s3:.2f}\\%$ превышает нормативный порог **10.0%** "
            f"(95% ДИ Клоппера–Пирсона: $[{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$) — класс S3 статистически репрезентативен. {s3_mech_sentence}"
        )

    lines = [
        "Для оценки статистической репрезентативности фатальных и критических исходов $S3$ в Подклассе B (сход в кювет) проведен расчет точного 95% биномиального доверительного интервала Клоппера–Пирсона (Clopper–Pearson exact confidence interval):",
        "",
        f"* **Точечная оценка вероятности S3 при ДТП:** $P(S3 \\mid \\text{{ДТП}}) = \\frac{{{k_s3}}}{{{n_crashes}}} = {p_s3:.2f}\\%$.",
        f"* **Точный 95% доверительный интервал:** $\\text{{CI}}_{{95\\%}} = [{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$.",
        "",
        "**Физико-статистическое заключение по S3:**",
        ci_verdict
    ]

    if s3_runs:
        lines.append("")
        lines.append("**Параметры критических заездов с исходом S3:**")
        lines.append("| ID заезда | Подкласс | Участник схода | $V_{dep}$ [м/с] | $V_\\perp$ [м/с] | Угол заноса $\\beta$ | Скорость рыскания $\\omega_z$ | Механизм динамики | Severity |")
        lines.append("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- | :---: |")
        for r in s3_runs:
            dep_v = f"{float(r['departure_speed_mps']):.2f}" if r.get('departure_speed_mps') else "—"
            perp_v = f"{float(r['road_perp_speed_mps']):.2f}" if r.get('road_perp_speed_mps') else "—"
            beta = f"{float(r['sideslip_angle_deg']):.1f}°" if r.get('sideslip_angle_deg') else "—"
            yaw = f"{float(r['departure_yaw_rate_deg_s']):.1f}°/с" if r.get('departure_yaw_rate_deg_s') else "—"
            mech = r.get('ditch_mechanism', '—')
            lines.append(f"| #{r.get('run_id')} | {r.get('subclass')} | {r.get('ditch_departure_actor')} | {dep_v} | {perp_v} | {beta} | {yaw} | {mech} | **S3** |")

    return "\n".join(lines)


def _build_controllability_content(ctrl_a: dict = None, ctrl_b: dict = None, ctrl_all: dict = None) -> str:
    """Формирование раздела количественной оценки управляемости C по кампании active (SAE J2980:2023 Clause 5.2.3)."""
    if ctrl_all is None:
        return (
            "Отдельный CSV-файл кампании `--driver-post-shock active` (`--active-input`) не передан в данный запуск аналитики. "
            "Предварительные инженерные оценки управляемости в матрице ниже (`C2` при $\\ge 90\\%$ предотвращении S2+ и `C3` при $< 90\\%$) "
            "приведены по нормативным критериям SAE J2980:2023 Clause 5.2.3 и подлежат уточнению по итогам кампании `active`."
        )

    lines = [
        "По результатам моделирования активного контрруления и торможения водителей после фазы реакции и удержания ($T_{react} + T_{hold}$) "
        "оценена доля инцидентов $P_{\\text{avoid}}(< S2 \\mid \\text{конфликт})$, в которых удалось предотвратить тяжелый вред ($S2/S3$):",
        "",
        "| Выборка (active) | Валидных сцен | ДТП ($n_{\\text{crash}}$) | Предотвратили $S2+$ ($k/n$) | $P_{\\text{avoid}}$ (%) | 95% ДИ Клоппера–Пирсона | Класс C | Статус ДИ |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |"
    ]

    for label, ct in [
        ("Подкласс А (Ограждение)", ctrl_a),
        ("Подкласс B (Кювет)", ctrl_b),
        ("**Объединенная кампания `active`**", ctrl_all),
    ]:
        if ct is None:
            continue
        n_ref = ct['n_crashes'] if ct['n_crashes'] > 0 else ct['n_valid']
        k_ref = ct['k_avoid_crashes'] if ct['n_crashes'] > 0 else ct['k_avoid_valid']
        p_ref = ct['p_avoid_crashes'] if ct['n_crashes'] > 0 else ct['p_avoid_valid']
        ci = ct['ci_crashes'] if ct['n_crashes'] > 0 else ct['ci_valid']
        lines.append(
            f"| {label} | {ct['n_valid']} | {ct['n_crashes']} | {k_ref} / {n_ref} | "
            f"**{p_ref:.1f}%** | [{ci[0]*100:.1f}%, {ci[1]*100:.1f}%] | **{ct['assigned_c']}** | {ct['ci_note']} |"
        )

    lines.extend([
        "",
        "*Критерии назначения C (ISO 26262-3:2018 Table B.4 / SAE J2980:2023 Clause 5.2.3):* "
        "**C1** ($P_{\\text{avoid}} \\ge 99\\%$), **C2** ($90\\% \\le P_{\\text{avoid}} < 99\\%$), **C3** ($P_{\\text{avoid}} < 90\\%$)."
    ])
    return "\n".join(lines)


def _build_asil_matrix_content(s_worst: str, s_odd: str, analysis_a: dict = None, analysis_b: dict = None,
                               analysis_odd: dict = None, ctrl_a: dict = None, ctrl_b: dict = None,
                               ctrl_all: dict = None) -> str:
    """Формирование итоговой матрицы классификации опасного события и назначения ASIL (Раздел 1.6)."""
    p_ge_s2_odd = analysis_odd['cum_percentages']['>=S2'] if analysis_odd else 0.0
    p_ge_s2_b = analysis_b['cum_percentages']['>=S2'] if analysis_b else 0.0
    p_ge_s2_a = analysis_a['cum_percentages']['>=S2'] if analysis_a else 0.0

    if analysis_b and analysis_b.get('total_crashes', 0) > 0:
        k_s3_b = analysis_b['counts'].get('S3', 0)
        n_b = analysis_b['total_crashes']
        _, ci_high_b = calc_clopper_pearson_ci(k_s3_b, n_b, confidence=0.95)
        p_s3_b = analysis_b['cum_percentages']['S3']
        if ci_high_b * 100.0 < 10.0:
            s3_excl_str = f"Класс S3 исключен статистически ($P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\%$, верхняя граница 95% CI: ${ci_high_b*100:.2f}\\% < 10\\%$)."
        elif p_s3_b <= 10.0:
            s3_excl_str = f"Доля S3 в Подклассе B составляет $P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\% \\le 10\\%$ (95% CI до ${ci_high_b*100:.2f}\\%$)."
        else:
            s3_excl_str = f"Доля S3 в Подклассе B составляет $P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\% > 10\\%$ (95% CI до ${ci_high_b*100:.2f}\\%$)."
    else:
        s3_excl_str = "Статистическая оценка S3 формируется после прогонов в Подклассе B."

    asil_e3_c2 = determine_asil(s_worst, 'E3', 'C2')
    asil_e3_c3 = determine_asil(s_worst, 'E3', 'C3')
    asil_e4_c2 = determine_asil(s_worst, 'E4', 'C2')
    asil_e4_c3 = determine_asil(s_worst, 'E4', 'C3')

    if ctrl_all is not None:
        c_meas = ctrl_all['assigned_c']
        c_row_label = f"**{c_meas}** (замер `active`) / C2–C3"
        p_av = ctrl_all['p_avoid_crashes'] if ctrl_all['n_crashes'] > 0 else ctrl_all['p_avoid_valid']
        c_row_desc = (
            f"• **Измеренный класс {c_meas} (`active`):** предотвращение S2+ в **{p_av:.1f}%** случаев "
            f"({ctrl_all['ci_note']}).<br>"
            "• **C2** — реакция $\\ge 90\\%$ водителей; **C3** — пассивный выбег / $<90\\%$ удержания."
        )
        asil_meas_e3 = determine_asil(s_worst, 'E3', c_meas)
        measured_row = (
            f"| **Измеренный (`active`, E3 / {c_meas})** | **{s_worst}** | **E3** | **{c_meas}** | **{asil_meas_e3}** | "
            f"Прямой результат моделирования активной управляемости в ODD |"
        )
    else:
        c_row_label = "**C2** / **C3** (ожид. `--active-input`)"
        c_row_desc = (
            "• **C2** — нормативная реакция большинства водителей ($\\ge 90\\%$) по парированию заноса после фазы $T_{react}+T_{hold}$.<br>"
            "• **C3** — пассивный водитель без контрруления либо невозможность удержания курса ($<90\\%$)."
        )
        measured_row = None

    lines = [
        f"На основе класса тяжести **{s_worst}** (SAE J2980:2023 / ISO 26262-3) формируется матрица классификации опасного события по трем параметрам риска:",
        "",
        "| Параметр риска ISO 26262-3 | Назначенный класс | Обоснование и инженерный контекст |",
        "| :--- | :---: | :--- |",
        "| **Опасное событие (Hazardous Event)** | — | **Столкновение с попутным ТС в слепой зоне при перестроении на скорости 100 км/ч (27.78 м/с) с последующим ударом в ограждение или сходом в кювет** |",
        f"| **Тяжесть (Severity, S)** | **{s_worst}** | Обосновано по результатам моделирования в CARLA: $P(\\ge S2 \\mid \\text{{ДТП}}) = {format_iso_pct(p_ge_s2_odd).replace('%', chr(92) + '%')}$ во взвешенном ODD (${format_iso_pct(p_ge_s2_b).replace('%', chr(92) + '%')}$ в кювете B, ${format_iso_pct(p_ge_s2_a).replace('%', chr(92) + '%')}$ у ограждения A). {s3_excl_str} |",
        "| **Экспозиция (Exposure, E)** | **E3** / **E4** | • **E3** — средняя частота возникновения опасной ситуации по ISO 26262-3 Table B.3 (от нескольких раз в год до нескольких раз в месяц, $1\\text{–}10\\%$ времени движения; по частотному расчёту ODD проекта $\\approx 20\\text{–}80\\text{ раз/год}$ или $1.67\\text{–}6.7\\text{ раз/мес}$).<br>• **E4** — консервативная оценка высокой интенсивности трафика ($>10\\%$ длительности поездок / почти в каждой поездке в плотном потоке). |",
        f"| **Контролируемость (Controllability, C)** | {c_row_label} | {c_row_desc} |",
        "",
        "#### Сводная матрица определения уровня полноты безопасности (ASIL Determination по ISO 26262-3 Table 4):",
        "",
        "| Эксплуатационный профиль | Severity (S) | Exposure (E) | Controllability (C) | Целевой ASIL | Инженерная интерпретация |",
        "| :--- | :---: | :---: | :---: | :---: | :--- |",
    ]
    if measured_row is not None:
        lines.append(measured_row)
    lines.extend([
        f"| **Профиль E3 / C2** | **{s_worst}** | **E3** | **C2** | **{asil_e3_c2}** | Типовая загородная трасса, контрруление $\\ge 90\\%$ водителей |",
        f"| **Консервативный E3 / C3** | **{s_worst}** | **E3** | **C3** | **{asil_e3_c3}** | Типовая частота перестроений, удержание $< 90\\%$ / пассивный режим |",
        f"| **Плотный трафик E4 / C2** | **{s_worst}** | **E4** | **C2** | **{asil_e4_c2}** | Высокая интенсивность потока, парирование заноса большинством водителей |",
        f"| **Наихудший случай E4 / C3** | **{s_worst}** | **E4** | **C3** | **{asil_e4_c3}** | Максимальная экспозиция потока и критически низкая управляемость |",
        "",
        "**Рекомендации по утверждению требований функциональной безопасности (Safety Goals):**",
        f"1. Базовый целевой уровень безопасности для функции LCC / BSD в номинальной частоте ODD (E3, C2) составляет **{asil_e3_c2}**.",
        f"2. При подтверждении **C3** в кампании `active` или консервативном сценарии E4 целевой уровень повышается до **{asil_e3_c3}** / **{asil_e4_c3}**."
    ])
    return "\n".join(lines)


def _build_mechanisms_and_asymmetry_content(records: list[dict], analysis_a: dict, analysis_b: dict) -> tuple[str, str, str]:
    """Формирование текстовых блоков механизмов аварий и асимметрии масс (Раздел 5)."""
    sub_a_crashes = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER' and is_crash_run(r)]
    sub_b_crashes = [r for r in records if r['subclass'] == 'SUBCLASS_B_DITCH' and is_crash_run(r)]

    barrier_types = defaultdict(int)
    for r in sub_a_crashes:
        if is_valid_barrier_hit(r) and r['barrier_impact_type']:
            barrier_types[f"{r['barrier_impact_type']} ({r['barrier_collision_actor']})"] += 1

    ditch_mechanisms = defaultdict(int)
    for r in sub_b_crashes:
        if is_valid_ditch_departure(r) and r['ditch_mechanism']:
            ditch_mechanisms[r['ditch_mechanism']] += 1

    # 5.1 Ограждение
    barrier_lines = []
    if barrier_types:
        for b_name, count in sorted(barrier_types.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_a['total_crashes']) * 100
            pct_total = count / max(1, analysis_a['total_runs']) * 100
            barrier_lines.append(f"* **{b_name}:** {count} {plural_ru(count, ('прогон', 'прогона', 'прогонов'))} ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
        stats_bar_v = calc_stats([r['barrier_impact_mps'] for r in sub_a_crashes if is_valid_barrier_hit(r)])
        max_vb = stats_bar_v['max']
        barrier_lines.append(
            f"* *Методологическая сноска по Подклассу А:* оценка $S_A = {analysis_a['assigned_class']}$ дана по доударной нормальной скорости "
            f"(`Side-swipe` $\\le {max_vb:.2f}\\text{{ м/с}}$) для гладкого сертифицированного ограждения без зацепа "
            f"(резкая остановка кузова у стенки в CARLA является артефактом твёрдотельного трения PhysX `0.72–1.42`)."
        )
    else:
        barrier_lines.append("* Удары в ограждение не зафиксированы.")
    barrier_content = "\n".join(barrier_lines)

    # 5.2 Кювет
    ditch_lines = []
    if ditch_mechanisms:
        for m_name, count in sorted(ditch_mechanisms.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_b['total_crashes']) * 100
            pct_total = count / max(1, analysis_b['total_runs']) * 100
            ditch_lines.append(f"* **{m_name}:** {count} {plural_ru(count, ('прогон', 'прогона', 'прогонов'))} ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
    else:
        ditch_lines.append("* Вылеты за бровку проезжей части Y = 9.75 м не зафиксированы.")
    ditch_content = "\n".join(ditch_lines)

    # 5.3 Асимметрия уязвимости
    asym_lines = []
    sub_a_runs = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    sub_b_runs = [r for r in records if r['subclass'] == 'SUBCLASS_B_DITCH']

    if sub_a_runs:
        a_hits = [r for r in sub_a_runs if is_valid_barrier_hit(r)]
        a_tgt_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('target', 'both'))
        a_ego_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('ego', 'both'))
        a_both = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'both')
        a_tgt_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'target')
        a_ego_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'ego')

        def loc_zaezd(n: int) -> str:
            return 'заезде' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'заездах'

        def loc_sluchai(n: int) -> str:
            return 'случае' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'случаях'

        def loc_progon(n: int) -> str:
            return 'прогоне' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'прогонах'

        detail_parts = []
        if a_both > 0:
            detail_parts.append(f"в {a_both} {loc_zaezd(a_both)} ударились оба ТС")
        if a_tgt_only > 0:
            detail_parts.append(f"в {a_tgt_only} — только Target")
        if a_ego_only > 0:
            detail_parts.append(f"в {a_ego_only} — только Ego")
        details_a_str = f" ({', '.join(detail_parts)})" if detail_parts else ""

        pct_a_bar = (len(a_hits) / max(1, len(sub_a_runs))) * 100.0
        pct_a_crash = (len(a_hits) / max(1, analysis_a['total_crashes'])) * 100.0
        a_cross_deps = [r for r in sub_a_runs if is_valid_ditch_departure(r)]
        cross_str = (
            f" Перекрёстный вылет за правую бровку кювета $Y = 9.75\\text{{ м}}$ в Подклассе А зафиксирован в **{len(a_cross_deps)}** {loc_progon(len(a_cross_deps))}."
            if a_cross_deps else ""
        )
        asym_lines.append(
            f"* В Подклассе А контакт с ограждением зафиксирован в **{pct_a_bar:.1f}%** прогонов "
            f"({len(a_hits)}/{len(sub_a_runs)} от N_total, {pct_a_crash:.1f}% от выборки ДТП). "
            f"Из них Ego контактировал в {a_ego_inv} {loc_sluchai(a_ego_inv)}, а Target — в {a_tgt_inv}{details_a_str}.{cross_str}"
        )

    if sub_b_runs:
        def loc_z_b(n: int) -> str:
            return 'заезде' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'заездах'

        def loc_sl_b(n: int) -> str:
            return 'случае' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'случаях'

        b_deps = [r for r in sub_b_runs if is_valid_ditch_departure(r)]
        b_tgt_inv = sum(1 for r in b_deps if r.get('ditch_departure_actor') in ('target', 'both'))
        b_tgt_only = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'target')
        b_ego_inv = sum(1 for r in b_deps if r.get('ditch_departure_actor') in ('ego', 'both'))
        b_ego_only = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'ego')
        b_both = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'both')

        single_actor_details = []
        if b_tgt_only > 0:
            single_actor_details.append(f"в {b_tgt_only} {loc_z_b(b_tgt_only)} с полосы сошел только Target")
        if b_ego_only > 0:
            single_actor_details.append(f"в {b_ego_only} {loc_z_b(b_ego_only)} — только Ego")

        if not b_deps:
            single_str = ""
        elif single_actor_details:
            single_str = f" При этом {', '.join(single_actor_details)} (в остальных {b_both} {loc_z_b(b_both)} в кювет сошли оба ТС)."
        else:
            single_str = f" Во всех {b_both} {loc_z_b(b_both)} схода в кювет были вовлечены оба ТС."

        pct_b_dep = (len(b_deps) / max(1, len(sub_b_runs))) * 100.0
        pct_b_crash = (len(b_deps) / max(1, analysis_b['total_crashes'])) * 100.0
        asym_lines.append(
            f"* В Подклассе B сход с проезжей части в кювет произошел в **{pct_b_dep:.1f}%** прогонов "
            f"({len(b_deps)}/{len(sub_b_runs)} от N_total, {pct_b_crash:.1f}% от выборки ДТП). "
            f"При этом Target сносится с полосы в {b_tgt_inv} {loc_sl_b(b_tgt_inv)} ({b_tgt_inv/max(1, len(sub_b_runs))*100:.1f}%), "
            f"а Ego — в {b_ego_inv} {loc_sl_b(b_ego_inv)} ({b_ego_inv/max(1, len(sub_b_runs))*100:.1f}%).{single_str}"
        )

    asym_lines.append("* **Вывод:** Данные подтверждают требование стандарта ISO 26262-3 о двустороннем анализе тяжести последствий как для экипажа Ego, так и для сторонних участников движения (Target).")
    asymmetry_content = "\n".join(asym_lines)

    return barrier_content, ditch_content, asymmetry_content


def generate_markdown_report(records: list[dict], analysis_a: dict, analysis_b: dict,
                             analysis_odd: dict, s_worst: str, output_dir: str,
                             template_path: str = None, ctrl_a: dict = None,
                             ctrl_b: dict = None, ctrl_all: dict = None) -> str:
    """
    Формирование строгого научно-инженерного отчета по функциональной безопасности.
    Архитектурно отделяет бизнес-логику расчетов от представления через Markdown-шаблон.
    """
    resolved_template_path = resolve_template_path(template_path)
    with open(resolved_template_path, mode='r', encoding='utf-8') as f:
        template_str = f.read()

    total_runs = len(records)
    crash_records = [r for r in records if is_crash_run(r)]
    total_crashes = len(crash_records)
    records_b = [r for r in records if r['subclass'] == 'SUBCLASS_B_DITCH']
    ditch_departures = [r for r in records if is_valid_ditch_departure(r)]
    ditch_departures_b = sum(1 for r in ditch_departures if r['subclass'] == 'SUBCLASS_B_DITCH')
    ditch_departures_a_cross = sum(1 for r in ditch_departures if r['subclass'] == 'SUBCLASS_A_BARRIER')
    ditch_departures_total = len(ditch_departures)
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})

    # Расчетные таблицы и динамические блоки
    table_severity = _build_severity_table(analysis_a, analysis_b, analysis_odd, s_worst)
    protocol_steps = _build_iso_steps_content(analysis_a, analysis_b, analysis_odd)
    table_kinematics = _build_kinematics_table(records)
    odd_paradox = _build_odd_paradox_content(analysis_a, analysis_b, analysis_odd, s_worst)
    conflict_funnel = _build_conflict_funnel_content(analysis_a, analysis_b)
    s3_analysis = _build_s3_analysis_content(analysis_b, records_b)
    controllability_section = _build_controllability_content(ctrl_a, ctrl_b, ctrl_all)
    asil_matrix = _build_asil_matrix_content(
        s_worst, analysis_odd['assigned_class'], analysis_a, analysis_b, analysis_odd,
        ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all
    )
    barrier_content, ditch_content, asymmetry_content = _build_mechanisms_and_asymmetry_content(
        records, analysis_a, analysis_b
    )

    # Словарь контекста для подстановки в шаблон
    context = {
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'runs_a': analysis_a['total_runs'],
        'crashes_a': analysis_a['total_crashes'],
        'runs_b': analysis_b['total_runs'],
        'crashes_b': analysis_b['total_crashes'],
        'runs_c': analysis_b['total_runs'],
        'crashes_c': analysis_b['total_crashes'],
        'ditch_departures_b': ditch_departures_b,
        'ditch_departures_a_cross': ditch_departures_a_cross,
        'ditch_departures_total': ditch_departures_total,
        'weight_a': f"{weights.get('A', 0.20):.2f}",
        'weight_b': f"{weights.get('B', 0.80):.2f}",
        'assigned_class_a': analysis_a['assigned_class'],
        'decision_reason_a': analysis_a['decision_reason'],
        'assigned_class_b': analysis_b['assigned_class'],
        'decision_reason_b': analysis_b['decision_reason'],
        'assigned_class_c': analysis_b['assigned_class'],
        'decision_reason_c': analysis_b['decision_reason'],
        's_worst': s_worst,
        's_worst_bold': f"\\mathbf{{{s_worst}}}",
        'assigned_class_odd': analysis_odd['assigned_class'],
        'decision_reason_odd': analysis_odd['decision_reason'],
        'odd_paradox_content': odd_paradox,
        'conflict_funnel_content': conflict_funnel,
        's3_analysis_content': s3_analysis,
        'controllability_section': controllability_section,
        'asil_matrix_content': asil_matrix,
        'table_severity_distribution': table_severity,
        'table_2_content': table_severity,
        'protocol_steps_content': protocol_steps,
        'section_3_content': protocol_steps,
        'table_kinematics_stats': table_kinematics,
        'stats_table_content': table_kinematics,
        'barrier_mechanisms_content': barrier_content,
        'ditch_mechanisms_content': ditch_content,
        'actor_asymmetry_content': asymmetry_content,
    }

    # Рендеринг шаблона
    report_content = render_template(template_str, context)

    os.makedirs(output_dir, exist_ok=True)
    report_path = os.path.join(output_dir, "summary_report.md")
    with open(report_path, mode='w', encoding='utf-8') as f:
        f.write(report_content)

    print(f"  [OK] Полный аналитический отчет сохранен в: {report_path} (шаблон: {resolved_template_path})")
    return report_path


# ------------------------------------------------------------------------------
# Консольный вывод сводки результатов
# ------------------------------------------------------------------------------
def print_console_summary(analysis_a: dict, analysis_b: dict, analysis_odd: dict, s_worst: str,
                          ctrl_a: dict = None, ctrl_b: dict = None, ctrl_all: dict = None):
    """Форматированный вывод сводной таблицы и многоуровневого вердикта в консоль."""
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    print("\n" + "=" * 108)
    print(" СВОДНАЯ ОЦЕНКА ТЯЖЕСТИ HARA ПО СТАНДАРТАМ ISO 26262-3 / SAE J2980:2023 (>10% RULE ПО ВЫБОРКЕ ДТП)")
    print("=" * 108)
    header = f"{'Профиль ODD':<24} | {'Вес':<4} | {'N_tot (Конфл)':<14} | {'N_crash':<7} | {'S0 (%)':<8} | {'S1 (%)':<8} | {'S2 (%)':<8} | {'S3 (%)':<8} | {'P(≥S2)':<7} | {'Вердикт':<7}"
    print(header)
    print("-" * 108)

    def print_row(name, weight, n_tot_str, n_crash, pcts, cum_pcts, assigned):
        s0_str = f"{pcts.get('S0',0):.1f}%"
        s1_str = f"{pcts.get('S1',0):.1f}%"
        s2_str = f"{pcts.get('S2',0):.1f}%"
        s3_str = f"{pcts.get('S3',0):.1f}%"
        p_ge_s2 = format_iso_pct(cum_pcts.get('>=S2', 0))
        print(f"{name:<24} | {weight:<4.2f} | {n_tot_str:<14} | {n_crash:<7} | {s0_str:<8} | {s1_str:<8} | {s2_str:<8} | {s3_str:<8} | {p_ge_s2:<7} | {assigned:<7}")

    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    print_row("Подкласс А (Ограждение)", w_a, n_tot_a_str, analysis_a['total_crashes'], analysis_a['percentages'], analysis_a['cum_percentages'], f"S{SEVERITY_RANK[analysis_a['assigned_class']]}")

    n_tot_b_str = f"{analysis_b['total_runs']} ({analysis_b['conflict_rate']:.1f}%)"
    print_row("Подкласс B (Кювет)", w_b, n_tot_b_str, analysis_b['total_crashes'], analysis_b['percentages'], analysis_b['cum_percentages'], f"S{SEVERITY_RANK[analysis_b['assigned_class']]}")

    print("-" * 108)
    tot_runs_odd = max(1, analysis_odd['total_runs'])
    n_tot_odd_str = f"{analysis_odd['total_runs']} ({(analysis_odd['total_crashes']/tot_runs_odd)*100:.1f}%)"
    print_row("ВЗВЕШЕННЫЙ ODD (Пул)", 1.00, n_tot_odd_str, analysis_odd['total_crashes'],
              analysis_odd['percentages'], analysis_odd['cum_percentages'], f"{analysis_odd['assigned_class']}")
    print("-" * 108)
    print(f"{'WORST-CASE SUBCLASS':<24} | {'-':<4} | {n_tot_odd_str:<14} | {analysis_odd['total_crashes']:<7} | {'-':<8} | {'-':<8} | {'-':<8} | {'-':<8} | {'-':<7} | >>> {s_worst} <<<")
    print("=" * 108)
    print("МНОГОУРОВНЕВЫЙ ВЕРДИКТ HARA (ПО ВЫБОРКЕ СОСТОЯВШИХСЯ ДТП):")
    print(f"  * Подкласс А (Ограждение слева): {analysis_a['assigned_class']} ({analysis_a['decision_step']})")
    print(f"  * Подкласс B (Кювет справа):     {analysis_b['assigned_class']} ({analysis_b['decision_step']})")
    print(f"  * Worst-Case Subclass:           >>> {s_worst} <<< (консервативно для раздельных строк HARA)")
    print(f"  * Взвешенный ODD:                >>> {analysis_odd['assigned_class']} <<< (при сведении в единый пул ODD)")

    if ctrl_all is not None:
        print("-" * 108)
        print("ОЦЕНКА УПРАВЛЯЕМОСТИ CONTROLLABILITY (C) ПО КАМПАНИИ --driver-post-shock active:")
        for lbl, ct in [("Подкласс А", ctrl_a), ("Подкласс B", ctrl_b), ("Итого Active", ctrl_all)]:
            if ct is None:
                continue
            p_av = ct['p_avoid_crashes'] if ct['n_crashes'] > 0 else ct['p_avoid_valid']
            n_av = ct['n_crashes'] if ct['n_crashes'] > 0 else ct['n_valid']
            k_av = ct['k_avoid_crashes'] if ct['n_crashes'] > 0 else ct['k_avoid_valid']
            print(f"  * {lbl:<14}: P_avoid(<S2) = {k_av}/{n_av} ({p_av:.1f}%), {ct['ci_note']} -> {ct['assigned_c']}")
    print("=" * 108)


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Statistical Analysis of HARA Simulation Results (ISO 26262-3 / SAE J2980:2023)")
    parser.add_argument('--input', type=str, default='results',
                        help="Path to passive CSV file or directory containing results (default: results)")
    parser.add_argument('--active-input', type=str, default=None,
                        help="Optional path to active driver campaign CSV for Controllability (C) analysis")
    parser.add_argument('--output-dir', type=str, default='results',
                        help="Directory to save summary report and plots (default: results)")
    parser.add_argument('--weight-a', type=float, default=0.20,
                        help="Statistical weight of Subclass A (Barrier) in ODD (default: 0.20)")
    parser.add_argument('--weight-b', type=float, default=0.80,
                        help="Statistical weight of Subclass B (Ditch) in ODD (default: 0.80)")
    parser.add_argument('--weight-c', type=float, default=None,
                        help=argparse.SUPPRESS)
    parser.add_argument('--template', type=str, default=None,
                        help="Path to Markdown report template (default: templates/report_template.md or report_template.md)")
    args = parser.parse_args()

    weight_b = args.weight_b
    if args.weight_c is not None:
        print("  [WARN] Флаг --weight-c устарел; используйте --weight-b (Подкласс B — Кювет).")
        weight_b = args.weight_c

    print("=" * 80)
    print(" МОДУЛЬ СТАТИСТИЧЕСКОЙ АНАЛИТИКИ HARA (ISO 26262-3 / SAE J2980:2023)")
    print(f" Входные данные (passive): '{args.input}'")
    if args.active_input:
        print(f" Входные данные (active):  '{args.active_input}'")
    print(f" Папка отчетов и графиков: '{args.output_dir}'")
    print(f" Веса в ODD: Подкласс А = {args.weight_a:.2f}, Подкласс B = {weight_b:.2f}")
    print("=" * 80)

    # 1. Загрузка данных
    records = load_simulation_results(args.input)
    print(f"\n[1/4] Загружено {len(records)} записей прогонов.")

    # 2. Выделение состоявшихся ДТП и безаварийных разъездов (Воронка сценария)
    records_a = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    records_b = [r for r in records if r['subclass'] == 'SUBCLASS_B_DITCH']

    crashes_a = [r for r in records_a if is_crash_run(r)]
    crashes_b = [r for r in records_b if is_crash_run(r)]

    counts_a = defaultdict(int)
    for r in crashes_a:
        counts_a[r['severity']] += 1

    counts_b = defaultdict(int)
    for r in crashes_b:
        counts_b[r['severity']] += 1

    # 3. Расчет по правилу >10% строго по выборке ДТП
    print("\n[2/4] Расчет кумулятивных вероятностей по правилу >10% (выборка состоявшихся ДТП)...")
    spawn_err_a = sum(1 for r in records_a if is_spawn_error_run(r))
    spawn_err_b = sum(1 for r in records_b if is_spawn_error_run(r))
    analysis_a = evaluate_iso_severity(counts_a, total_crashes=len(crashes_a), total_runs=len(records_a), spawn_errors=spawn_err_a)
    analysis_b = evaluate_iso_severity(counts_b, total_crashes=len(crashes_b), total_runs=len(records_b), spawn_errors=spawn_err_b)
    analysis_odd = evaluate_weighted_odd(analysis_a, analysis_b, weight_a=args.weight_a, weight_b=weight_b)
    s_worst = compute_worst_case_verdict(analysis_a['assigned_class'], analysis_b['assigned_class'])

    # Опциональная оценка Controllability (C) по active CSV
    ctrl_a = ctrl_b = ctrl_all = None
    if args.active_input:
        active_records = load_simulation_results(args.active_input)
        active_a = [r for r in active_records if r['subclass'] == 'SUBCLASS_A_BARRIER']
        active_b = [r for r in active_records if r['subclass'] == 'SUBCLASS_B_DITCH']
        ctrl_a = evaluate_controllability(active_a, harm_threshold='S2')
        ctrl_b = evaluate_controllability(active_b, harm_threshold='S2')
        ctrl_all = evaluate_controllability(active_records, harm_threshold='S2')

    # 4. Вывод в консоль
    print_console_summary(analysis_a, analysis_b, analysis_odd, s_worst,
                          ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all)

    # 5. Построение обновленных графиков
    print("\n[3/4] Построение 5 обновленных инженерных графиков в results/plots/...")
    generate_plots(records, analysis_a, analysis_b, analysis_odd, s_worst, args.output_dir)

    # 6. Генерация аналитического отчета
    print("\n[4/4] Генерация итогового отчета summary_report.md...")
    generate_markdown_report(records, analysis_a, analysis_b, analysis_odd, s_worst, args.output_dir,
                             template_path=args.template, ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all)

    print("\n" + "=" * 80)
    print(" АНАЛИЗ УСПЕШНО ЗАВЕРШЕН!")
    print(f" Отчет: {os.path.join(args.output_dir, 'summary_report.md')}")
    print(f" Графики: {os.path.join(args.output_dir, 'plots')}")
    print("=" * 80)


if __name__ == '__main__':
    main()
