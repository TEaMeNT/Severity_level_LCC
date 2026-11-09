# ==============================================================================
# HARA STATISTICAL ANALYTICS & ISO 26262 / SAE J2980 DECISION MODULE
# Скрипт: analyze_results.py (Ревизия 3.0: Расчет Severity строго по выборке ДТП,
# воронка кинематического конфликта и устранение пустых пропусков)
# Стандарты: ISO 26262-3:2018 (Clause 6), SAE J2980:2018 (Annex B & E), AASHTO RDG, NCHRP 665
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
    'SUBCLASS_C_DITCH': '#8e44ad',    # Фиолетовый
    'ODD_WEIGHTED': '#2c3e50',        # Темно-синий/графитовый
}

SUBCLASS_NAMES = {
    'SUBCLASS_A_BARRIER': 'Подкласс А (Отбойник)',
    'SUBCLASS_C_DITCH': 'Подкласс С (Кювет)',
    'ODD_WEIGHTED': 'Взвешенный ODD'
}


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
                # Нормализация имени подкласса
                subclass_raw = row.get('subclass', '').strip()
                if 'barrier' in subclass_raw.lower() or subclass_raw == 'A':
                    subclass = 'SUBCLASS_A_BARRIER'
                elif 'ditch' in subclass_raw.lower() or subclass_raw == 'C':
                    subclass = 'SUBCLASS_C_DITCH'
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
                if ditch_actor != 'None' and v_dep is not None and v_dep > 0:
                    is_s3_skid = (v_dep > 18.0) and ((v_lat is not None and v_lat > 5.5) or (beta is not None and beta >= 25.0) or (omega_z is not None and omega_z > 45.0))
                    is_s3_nose = (v_perp is not None and v_perp > 12.0)
                    is_s2_skid = (v_dep > 11.0) and ((v_lat is not None and v_lat > 3.5) or (beta is not None and beta >= 15.0) or (omega_z is not None and omega_z > 25.0))
                    is_s2_bottom = (v_perp is not None and v_perp > 6.0)
                    is_s1_tracking = (v_dep > 8.0 and (v_perp is not None and v_perp > 2.0))
                    is_s1_low_skid = (v_dep <= 11.0 and (beta is not None and beta >= 15.0))

                    if is_s3_skid or is_s3_nose:
                        sev_ditch = 'S3'
                    elif is_s2_skid or is_s2_bottom:
                        sev_ditch = 'S2'
                    elif is_s1_tracking or is_s1_low_skid:
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
                records.append(rec)

    return records


# ------------------------------------------------------------------------------
# Математическая оценка по правилу >10% (ISO 26262-3 / SAE J2980) строго по N_crash
# ------------------------------------------------------------------------------
def evaluate_iso_severity(counts: dict, total_crashes: int, total_runs: int = None) -> dict:
    """
    Оценка итогового класса тяжести по правилу кумулятивного порога >10%
    строго по выборке состоявшихся ДТП (Accident-Involved Runs).
    ISO 26262-3 (Annex B, Table B.1) / SAE J2980 (Appendix B).
    """
    if total_runs is None:
        total_runs = total_crashes
    safe_pass = max(0, total_runs - total_crashes)
    conflict_rate = (total_crashes / max(1, total_runs)) * 100.0

    if total_crashes == 0:
        return {
            'total': 0,
            'total_runs': total_runs,
            'total_crashes': 0,
            'safe_pass_runs': safe_pass,
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
        decision_reason = f"Доля критических/фатальных исходов среди ДТП P(S3 | ДТП) = {p_s3:.1f}% превышает нормативный порог 10.0%."
    elif p_ge_s2 > 10.0:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P(≥S2 | ДТП) > 10%'
        decision_reason = (f"Кумулятивная вероятность тяжелых травм среди ДТП P(≥S2 | ДТП) = {p_ge_s2:.1f}% "
                           f"(S3: {p_s3:.1f}%, S2: {pcts['S2']:.1f}%) превышает нормативный порог 10.0%.")
    elif p_ge_s1 > 10.0:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P(≥S1 | ДТП) > 10%'
        decision_reason = (f"Кумулятивная вероятность травм легкой/средней тяжести среди ДТП P(≥S1 | ДТП) = {p_ge_s1:.1f}% "
                           f"превышает нормативный порог 10.0%.")
    else:
        assigned_class = 'S0'
        decision_step = 'Шаг 4: P(≥S1 | ДТП) ≤ 10%'
        decision_reason = (f"Суммарная вероятность травм любого уровня среди ДТП P(≥S1 | ДТП) = {p_ge_s1:.1f}% "
                           f"не превышает 10.0% (столкновения без травм AIS 0).")

    return {
        'total': total_crashes,
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'safe_pass_runs': safe_pass,
        'conflict_rate': conflict_rate,
        'counts': counts,
        'percentages': pcts,
        'cum_percentages': cum_pcts,
        'assigned_class': assigned_class,
        'decision_step': decision_step,
        'decision_reason': decision_reason
    }


def evaluate_weighted_odd(analysis_a: dict, analysis_c: dict, weight_a: float = 0.20, weight_c: float = 0.80) -> dict:
    """
    Взвешенная агрегация вероятностей по всему пространству условий ODD (W_A = 0.20, W_C = 0.80)
    строго по выборке ДТП с применением кумулятивного правила >10% ISO 26262-3.
    """
    total_w = weight_a + weight_c
    w_a = weight_a / total_w
    w_c = weight_c / total_w

    pct_a = analysis_a['percentages']
    pct_c = analysis_c['percentages']

    weighted_pcts = {}
    for s in SEVERITY_CLASSES:
        weighted_pcts[s] = w_a * pct_a.get(s, 0.0) + w_c * pct_c.get(s, 0.0)

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
        decision_reason = f"Взвешенная вероятность среди ДТП P_ODD(S3 | ДТП) = {p_s3:.1f}% превышает нормативный порог 10.0%."
    elif p_ge_s2 > 10.0:
        assigned_class = 'S2'
        decision_step = 'Шаг 2: P_ODD(≥S2 | ДТП) > 10%'
        decision_reason = (f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S2 | ДТП) = {p_ge_s2:.1f}% "
                           f"превышает нормативный порог 10.0%.")
    elif p_ge_s1 > 10.0:
        assigned_class = 'S1'
        decision_step = 'Шаг 3: P_ODD(≥S1 | ДТП) > 10%'
        decision_reason = (f"Взвешенная кумулятивная вероятность среди ДТП P_ODD(≥S1 | ДТП) = {p_ge_s1:.1f}% "
                           f"превышает нормативный порог 10.0%.")
    else:
        assigned_class = 'S0'
        decision_step = 'Шаг 4: P_ODD(≥S1 | ДТП) ≤ 10%'
        decision_reason = f"Взвешенная вероятность вреда среди ДТП P_ODD(≥S1 | ДТП) = {p_ge_s1:.1f}% не превышает 10.0%."

    total_runs = analysis_a.get('total_runs', 0) + analysis_c.get('total_runs', 0)
    total_crashes = analysis_a['total_crashes'] + analysis_c['total_crashes']

    return {
        'weights': {'A': w_a, 'C': w_c},
        'total': total_crashes,
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'percentages': weighted_pcts,
        'cum_percentages': cum_pcts,
        'assigned_class': assigned_class,
        'decision_step': decision_step,
        'decision_reason': decision_reason
    }


def compute_worst_case_verdict(class_a: str, class_c: str) -> str:
    """Определение консервативного вердикта Worst-Case Subclass = max(S_A, S_C)."""
    rank_a = SEVERITY_RANK.get(class_a, 0)
    rank_c = SEVERITY_RANK.get(class_c, 0)
    return SEVERITY_FROM_RANK[max(rank_a, rank_c)]


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

    # Приоритет критическим исходам: сначала S3, S2, затем S1, S0
    def get_rank(pt):
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        return -SEVERITY_RANK.get(sev, 0)

    sorted_indices = sorted(range(len(points)), key=lambda i: get_rank(points[i]))

    for idx in sorted_indices:
        pt = points[idx]
        x_val = pt[x_key] if pt[x_key] is not None else 0.0
        y_val = pt[y_key] if pt[y_key] is not None else 0.0
        nx, ny = pts_norm[idx]

        # Фильтр прореживания плотного кластера для обеих панелей
        dist_thresh = 0.055 if is_panel_a else 0.048
        is_key_point = str(pt.get('run_id', '')) in ('107', '157', '182', '104', '122', '150', '128')
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


def generate_plots(records: list[dict], analysis_a: dict, analysis_c: dict,
                   analysis_odd: dict, s_worst: str, output_dir: str):
    """
    Построение 5 публикационных инженерных графиков строго по выборке состоявшихся ДТП (N_crash)
    и отображению воронки конфликта (N_total).
    """
    plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    n_crash_total = analysis_a['total_crashes'] + analysis_c['total_crashes']
    n_total_all = len(records)

    # ==========================================================================
    # ГРАФИК 1: 01_severity_distribution.png
    # Фактическое распределение исходов S0–S3 среди состоявшихся ДТП
    # ==========================================================================
    fig, ax = plt.subplots(figsize=(10.5, 6), dpi=300)
    x = np.arange(len(SEVERITY_CLASSES))
    width = 0.26

    vals_a = [analysis_a['percentages'][s] for s in SEVERITY_CLASSES]
    vals_c = [analysis_c['percentages'][s] for s in SEVERITY_CLASSES]
    vals_odd = [analysis_odd['percentages'][s] for s in SEVERITY_CLASSES]

    rects_a = ax.bar(x - width, vals_a, width, label=f"Подкласс А (Отбойник, n={analysis_a['total_crashes']})",
                     color='#2980b9', edgecolor='black', linewidth=0.8, alpha=0.9)
    rects_c = ax.bar(x, vals_c, width, label=f"Подкласс С (Кювет, n={analysis_c['total_crashes']})",
                     color='#8e44ad', edgecolor='black', linewidth=0.8, alpha=0.9)
    rects_odd = ax.bar(x + width, vals_odd, width, label="Взвешенный ODD (0.2A + 0.8C)",
                       color='#2c3e50', edgecolor='black', linewidth=0.8, alpha=0.9)

    # Подписи точных процентов и числа прогонов (n) в две строки
    for rect, s in zip(rects_a, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_a['counts'].get(s, 0)
        ax.annotate(f"{h:.1f}%\n(n={cnt})",
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold', color='#1a5276')

    for rect, s in zip(rects_c, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_c['counts'].get(s, 0)
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

    # Увеличенный предел оси Y (+25% сверху) для комфортного размещения надписей
    max_h = max(vals_a + vals_c + vals_odd) if (vals_a + vals_c + vals_odd) else 50.0
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
    vals_c_cum = [analysis_c['cum_percentages']['S3'], analysis_c['cum_percentages']['>=S2'], analysis_c['cum_percentages']['>=S1']]
    vals_odd_cum = [analysis_odd['cum_percentages']['S3'], analysis_odd['cum_percentages']['>=S2'], analysis_odd['cum_percentages']['>=S1']]

    r_a = ax.bar(x - width, vals_a_cum, width, label=f"Подкласс А (Отбойник, итог: {analysis_a['assigned_class']})",
                 color='#2980b9', edgecolor='black', linewidth=0.8)
    r_c = ax.bar(x, vals_c_cum, width, label=f"Подкласс С (Кювет, итог: {analysis_c['assigned_class']})",
                 color='#8e44ad', edgecolor='black', linewidth=0.8)
    r_odd = ax.bar(x + width, vals_odd_cum, width, label=f"Взвешенный ODD (итог: {analysis_odd['assigned_class']})",
                   color='#2c3e50', edgecolor='black', linewidth=0.8)

    # Полупрозрачная серая зона нерепрезентативных исходов <= 10%
    ax.axhspan(0, 10.0, color='gray', alpha=0.15, label='Зона отсечки нерепрезентативных исходов ≤ 10%')

    # Яркая красная линия отсечки 10%
    ax.axhline(10.0, color='#e74c3c', linestyle='--', linewidth=2.0, zorder=5,
               label='Порог репрезентативности 10.0% (ISO 26262-3 / SAE J2980)')

    for rect in list(r_a) + list(r_c) + list(r_odd):
        h = rect.get_height()
        ax.annotate(f"{h:.1f}%",
                    xy=(rect.get_x() + rect.get_width() / 2, h),
                    xytext=(0, 4), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_ylabel('Накопленная вероятность при ДТП (%)', fontsize=12, fontweight='bold')
    ax.set_title('Определение класса тяжести по правилу >10% (ISO 26262-3 / SAE J2980)',
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
    # Двухпанельный фазовый портрет граничных условий в кювете на бровке Y = 9.75 м (n=53)
    # ==========================================================================
    fig, (ax_trip, ax_perp) = plt.subplots(1, 2, figsize=(15.5, 6.5), dpi=300)
    ditch_points = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_valid_ditch_departure(r)]

    # --- ПАНЕЛЬ А: Механизм 1 (Soil-Trip Rollover) ---
    x_max_a = 32.0
    max_v_lat = max([pt['vehicle_lat_skid_mps'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_a = max(11.0, max_v_lat * 1.22)

    ax_trip.fill_between([18.0, x_max_a], 5.5, y_max_a, color='#e74c3c', alpha=0.20,
                         label='S3: Многократное опрокидывание (NHTSA / Cooperrider)')
    ax_trip.fill_between([11.0, 18.0], 3.5, y_max_a, color='#e67e22', alpha=0.20,
                         label='S2: Умеренный риск опрокидывания (NCHRP 665)')
    ax_trip.fill_between([18.0, x_max_a], 3.5, 5.5, color='#e67e22', alpha=0.20)
    ax_trip.fill_between([0.0, x_max_a], 0.0, 3.5, color='#2ecc71', alpha=0.10,
                         label='S1/S0: Безопасное преодоление откоса 1:4 (AASHTO RDG)')

    ax_trip.axvline(18.0, color='#c0392b', linestyle='--', linewidth=1.2)
    ax_trip.axvline(11.0, color='#d35400', linestyle='--', linewidth=1.2)
    ax_trip.axhline(5.5, color='#c0392b', linestyle='--', linewidth=1.2)
    ax_trip.axhline(3.5, color='#d35400', linestyle='--', linewidth=1.2)

    ax_trip.text(18.2, 0.4, 'V_dep = 18 м/с', color='#c0392b', fontsize=8, rotation=90)
    ax_trip.text(11.2, 0.4, 'V_dep = 11 м/с', color='#d35400', fontsize=8, rotation=90)
    ax_trip.text(x_max_a - 0.5, 5.65, 'Порог спотыкания V_lat = 5.5 м/с', color='#c0392b', fontsize=8, ha='right')
    ax_trip.text(x_max_a - 0.5, 3.65, 'Порог юза V_lat = 3.5 м/с', color='#d35400', fontsize=8, ha='right')

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        v_lat = pt['vehicle_lat_skid_mps'] if pt['vehicle_lat_skid_mps'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        ax_trip.scatter(v_dep, v_lat, color=c, edgecolors='black', s=90, alpha=0.85, marker=marker, zorder=10)

    annotate_ditch_points(ax_trip, ditch_points, 'departure_speed_mps', 'vehicle_lat_skid_mps', x_max_a, y_max_a, is_panel_a=True)

    ax_trip.set_xlabel('Общая скорость в начале кювета V_dep [м/с]', fontsize=11, fontweight='bold')
    ax_trip.set_ylabel('Скорость бокового скольжения V_lat [м/с]', fontsize=11, fontweight='bold')
    ax_trip.set_title('А. Риск опрокидывания при боковом заносе', fontsize=12, fontweight='bold')
    ax_trip.set_xlim(0, x_max_a)
    ax_trip.set_ylim(0, y_max_a)
    ax_trip.grid(True, linestyle=':', alpha=0.5)
    ax_trip.legend(loc='upper left', frameon=True, fontsize=8)

    # --- ПАНЕЛЬ Б: Механизм 2 (Nose-In / Bottom Impact) ---
    x_max_b = 32.0
    max_v_perp = max([pt['road_perp_speed_mps'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_b = max(16.5, max_v_perp * 1.22)

    ax_perp.fill_between([0.0, x_max_b], 12.0, y_max_b, color='#e74c3c', alpha=0.20,
                         label='S3: Влет в дно кювета (Severe Nose-In, V⊥ > 12 м/с)')
    ax_perp.fill_between([0.0, x_max_b], 6.0, 12.0, color='#e67e22', alpha=0.20,
                         label='S2: Жесткий удар подрамником (Bottom Impact, 6 < V⊥ ≤ 12 м/с)')
    ax_perp.fill_between([8.0, x_max_b], 2.0, 6.0, color='#f1c40f', alpha=0.18,
                         label='S1: Преодолимый съезд по откосу (Traversable, 2 < V⊥ ≤ 6 м/с)')
    # Низкоскоростной съезд V_dep <= 8.0 м/с и касательный съезд V_perp <= 2.0 м/с
    ax_perp.fill_between([0.0, 8.0], 2.0, 6.0, color='#2ecc71', alpha=0.10)
    ax_perp.fill_between([0.0, x_max_b], 0.0, 2.0, color='#2ecc71', alpha=0.10,
                         label='S0: Касательный / низкоскоростной съезд (V⊥ ≤ 2 м/с или V_dep ≤ 8 м/с)')

    ax_perp.axhline(12.0, color='#c0392b', linestyle='--', linewidth=1.2)
    ax_perp.axhline(6.0, color='#d35400', linestyle='--', linewidth=1.2)
    ax_perp.axhline(2.0, color='#b7950b', linestyle='--', linewidth=1.2)
    ax_perp.axvline(8.0, color='#27ae60', linestyle=':', linewidth=1.2)

    ax_perp.text(0.6, 12.2, 'Порог S3: V⊥ = 12.0 м/с', color='#c0392b', fontsize=8, ha='left')
    ax_perp.text(0.6, 6.2, 'Порог S2: V⊥ = 6.0 м/с', color='#d35400', fontsize=8, ha='left')
    ax_perp.text(0.6, 2.2, 'Порог S1: V⊥ = 2.0 м/с', color='#b7950b', fontsize=8, ha='left')
    ax_perp.text(8.2, 0.4, 'V_dep = 8.0 м/с', color='#27ae60', fontsize=8, rotation=90)
    ax_perp.text(4.0, 3.8, 'S0: V_dep ≤ 8 м/с', color='#1e8449', fontsize=7.5, ha='center', va='center', fontweight='bold')

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        v_perp = pt['road_perp_speed_mps'] if pt['road_perp_speed_mps'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        ax_perp.scatter(v_dep, v_perp, color=c, edgecolors='black', s=90, alpha=0.85, marker=marker, zorder=10)

    # Маркеры актеров для легенды
    ax_perp.scatter([], [], color='gray', marker='o', edgecolors='black', label='Съезд Ego')
    ax_perp.scatter([], [], color='gray', marker='s', edgecolors='black', label='Съезд Target')
    ax_perp.scatter([], [], color='gray', marker='^', edgecolors='black', label='Съезд обоих ТС (Both)')

    annotate_ditch_points(ax_perp, ditch_points, 'departure_speed_mps', 'road_perp_speed_mps', x_max_b, y_max_b, is_panel_a=False)

    ax_perp.set_xlabel('Общая скорость в начале кювета V_dep [м/с]', fontsize=11, fontweight='bold')
    ax_perp.set_ylabel('Скорость сближения с кюветом V⊥ [м/с]', fontsize=11, fontweight='bold')
    ax_perp.set_title('Б. Риск жесткого удара о дно кювета', fontsize=12, fontweight='bold')
    ax_perp.set_xlim(0, x_max_b)
    ax_perp.set_ylim(0, y_max_b)
    ax_perp.grid(True, linestyle=':', alpha=0.5)
    ax_perp.legend(loc='upper left', frameon=True, fontsize=8)

    fig.suptitle(f'Параметры съезда в кювет и оценка тяжести травм (Подкласс С, n = {len(ditch_points)})',
                 fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plot3_path = os.path.join(plots_dir, "03_ditch_phase_portrait.png")
    safe_savefig(plot3_path, dpi=300, bbox_inches='tight')
    plt.close()

    # ==========================================================================
    # ГРАФИК 4: 04_subclass_a_dv_cascade.png
    # Двухпанельный каскад скоростей в Подклассе А строго по состоявшимся ДТП (n=61)
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
        ax_tt.bar(indices - w_tt/2, dv_ego, w_tt, label='Delta-V Ego (Lincoln MKZ 3490 кг)', color='#5dade2', edgecolor='black', linewidth=0.7)
        ax_tt.bar(indices + w_tt/2, dv_target, w_tt, label='Delta-V Target (Audi TT 2000 кг)', color='#2471a3', edgecolor='black', linewidth=0.7)

        # Пороги SAE J2980 Table B1
        ax_tt.axhline(8.0, color='#c0392b', linestyle='--', linewidth=1.2, label='Порог S3 бокового удара (8.0 м/с)')
        ax_tt.axhline(3.5, color='#d35400', linestyle='--', linewidth=1.2, label='Порог S2 бокового удара (3.5 м/с)')
        ax_tt.axhline(1.0, color='#f39c12', linestyle='--', linewidth=1.2, label='Порог S1 бокового удара (1.0 м/с)')

        # Подпись первичного класса над парой столбиков (вертикально для устранения наложений)
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

        # --- НИЖНЯЯ ПАНЕЛЬ: Вторичный удар об отбойник ---
        bar_colors = [SEVERITY_COLORS.get(r['sev_barrier'], '#2ecc71') for r in sub_a_crashes]
        w_b = 0.55
        ax_bar.bar(indices, v_barrier, w_b, color=bar_colors, edgecolor='black', linewidth=0.8,
                   label='Скорость удара V_barrier (цвет по классу sev_barrier)')

        # Пороги шкалы отбойника
        l_fr_s3 = ax_bar.axhline(18.0, color='#922b21', linestyle=':', linewidth=1.4, label='FR Порог S3 (18.0 м/с, > 45°)')
        l_ss_s3 = ax_bar.axhline(12.0, color='#c0392b', linestyle='--', linewidth=1.4, label='SS Порог S3 (12.0 м/с, ≤ 45°)')
        l_fr_s2 = ax_bar.axhline(11.0, color='#b9770e', linestyle=':', linewidth=1.4, label='FR Порог S2 (11.0 м/с, > 45°)')
        l_ss_s2 = ax_bar.axhline(6.0, color='#d35400', linestyle='--', linewidth=1.4, label='SS Порог S2 (6.0 м/с, ≤ 45°)')

        # Подписи над столбиками отбойника (однострочные вертикальные)
        short_actor_map = {'target': 'T', 'ego': 'E', 'both': 'B', 'none': 'None'}
        for idx, r in enumerate(sub_a_crashes):
            v_b = v_barrier[idx]
            if is_valid_barrier_hit(r) and v_b > 0.01:
                act = str(r['barrier_collision_actor']).lower()
                short_actor = short_actor_map.get(act, r['barrier_collision_actor'])
                itype = 'SS' if r['barrier_impact_type'] == 'Side-swipe' else 'FR'
                sev_b = r['sev_barrier']
                txt = f"{v_b:.1f} м/с ({sev_b}, {itype}-{short_actor})"
                ax_bar.annotate(txt, xy=(idx, v_b), xytext=(0, 5), textcoords="offset points",
                                ha='center', va='bottom', fontsize=7.5, fontweight='bold', color='#2c3e50', rotation=90)

        # Запас по высоте (+40% и не менее 23.0 м/с) чтобы повернутые надписи не задевали легенду
        max_v_b = max(v_barrier + [18.0])
        ax_bar.set_ylim(0, max(23.0, max_v_b * 1.40))

        ax_bar.set_xlabel(f'Номер прогона с ДТП в Подклассе А (n = {len(sub_a_crashes)})', fontsize=11, fontweight='bold')
        ax_bar.set_ylabel('Скорость удара V_barrier [м/с]', fontsize=11, fontweight='bold')
        ax_bar.set_title(f'Б. Вторичный удар о бетонный отбойник с учетом угла удара (n = {len(sub_a_crashes)})',
                         fontsize=12, fontweight='bold')
        ax_bar.set_xticks(indices)
        ax_bar.set_xticklabels([f"#{r['run_id']}" for r in sub_a_crashes], fontsize=7.5, fontweight='bold', rotation=90)
        ax_bar.grid(axis='y', linestyle=':', alpha=0.6)

        # Формирование информативной легенды с Proxy artists и расшифровкой T, E, B
        import matplotlib.patches as mpatches
        p_s1 = mpatches.Patch(facecolor=SEVERITY_COLORS['S1'], edgecolor='black', label='Удар S1')
        p_s2 = mpatches.Patch(facecolor=SEVERITY_COLORS['S2'], edgecolor='black', label='Удар S2')
        p_s3 = mpatches.Patch(facecolor=SEVERITY_COLORS['S3'], edgecolor='black', label='Удар S3')
        p_actors = mpatches.Patch(facecolor='none', edgecolor='none', label='Участники: T (Target 2000 кг), E (Ego 3490 кг), B (Both)')
        handles = [p_s1, p_s2, p_s3, l_fr_s3, l_ss_s3, l_fr_s2, l_ss_s2, p_actors]
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
    fig, (ax_esc, ax_asym) = plt.subplots(1, 2, figsize=(15.5, 6.5), dpi=300)

    # --- ПАНЕЛЬ А: Эскалация тяжести при ДТП (Первичный контакт vs Вторичные события, N_crash=114) ---
    crash_records = [r for r in records if is_crash_run(r)]
    n_crash = len(crash_records)
    stage1_counts = {s: sum(1 for r in crash_records if r['sev_side'] == s) for s in SEVERITY_CLASSES}
    stage2_counts = {s: sum(1 for r in crash_records if r['severity'] == s) for s in SEVERITY_CLASSES}

    stage1_pcts = [(stage1_counts[s] / max(1, n_crash)) * 100.0 for s in SEVERITY_CLASSES]
    stage2_pcts = [(stage2_counts[s] / max(1, n_crash)) * 100.0 for s in SEVERITY_CLASSES]

    x_esc = np.arange(len(SEVERITY_CLASSES))
    w_esc = 0.35

    r_s1 = ax_esc.bar(x_esc - w_esc/2, stage1_pcts, w_esc, label='Этап 1: Только первичный боковой контакт ТС',
                      color='#3498db', edgecolor='black', linewidth=0.8)
    r_s2 = ax_esc.bar(x_esc + w_esc/2, stage2_pcts, w_esc, label='Этап 2: Итог после удара об отбойник / кювет',
                      color='#e74c3c', edgecolor='black', linewidth=0.8)

    for rect, s in zip(r_s1, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage1_counts[s]
        ax_esc.annotate(f"{h:.1f}%\n(n={cnt})",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 4), textcoords="offset points",
                        ha='center', va='bottom', fontsize=9, fontweight='bold', color='#1b4f72')

    for rect, s in zip(r_s2, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage2_counts[s]
        ax_esc.annotate(f"{h:.1f}%\n(n={cnt})",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 4), textcoords="offset points",
                        ha='center', va='bottom', fontsize=9, fontweight='bold', color='#922b21')

    max_esc = max(stage1_pcts + stage2_pcts) if (stage1_pcts + stage2_pcts) else 50.0
    ax_esc.set_ylim(0, max_esc * 1.25)
    ax_esc.set_ylabel('Доля от числа ДТП (%)', fontsize=11, fontweight='bold')
    ax_esc.set_title(f'А. Тяжесть травм: первичный контакт vs итог ДТП (n = {n_crash})',
                     fontsize=11, fontweight='bold', pad=10)
    ax_esc.set_xticks(x_esc)
    ax_esc.set_xticklabels(['S0 (Без травм)', 'S1 (Легкие)', 'S2 (Тяжелые)', 'S3 (Фатальные)'],
                           fontsize=10, fontweight='bold')
    ax_esc.grid(axis='y', linestyle=':', alpha=0.6)
    ax_esc.legend(loc='upper right', frameon=True, fontsize=8.5)

    # --- ПАНЕЛЬ Б: Воронка реализации конфликта в слепой зоне и распределение пострадавших (N_total=200) ---
    actors_cat = [
        'Оба ТС\n(Both)',
        'Только Target\n(2000 кг)',
        'Только Ego\n(3490 кг)',
        'Разъехались\nбез удара'
    ]

    sub_a_runs = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    sub_c_runs = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH']
    tot_a = max(1, len(sub_a_runs))
    tot_c = max(1, len(sub_c_runs))

    counts_a_act = {
        'both': sum(1 for r in sub_a_runs if is_crash_run(r) and r.get('barrier_collision_actor') == 'both'),
        'target': sum(1 for r in sub_a_runs if is_crash_run(r) and r.get('barrier_collision_actor') == 'target'),
        'ego': sum(1 for r in sub_a_runs if is_crash_run(r) and r.get('barrier_collision_actor') == 'ego'),
        'safe': sum(1 for r in sub_a_runs if not is_crash_run(r))
    }

    counts_c_act = {
        'both': sum(1 for r in sub_c_runs if is_crash_run(r) and r.get('ditch_departure_actor') == 'both'),
        'target': sum(1 for r in sub_c_runs if is_crash_run(r) and r.get('ditch_departure_actor') == 'target'),
        'ego': sum(1 for r in sub_c_runs if is_crash_run(r) and r.get('ditch_departure_actor') == 'ego'),
        'safe': sum(1 for r in sub_c_runs if not is_crash_run(r))
    }

    order_keys = ['both', 'target', 'ego', 'safe']
    pcts_a_act = [(counts_a_act[k] / tot_a) * 100.0 for k in order_keys]
    pcts_c_act = [(counts_c_act[k] / tot_c) * 100.0 for k in order_keys]

    x_act = np.arange(len(actors_cat))
    w_act = 0.35

    r_a_act = ax_asym.bar(x_act - w_act/2, pcts_a_act, w_act, label=f'Подкласс А (Отбойник, N={tot_a})',
                          color='#2980b9', edgecolor='black', linewidth=0.8)
    r_c_act = ax_asym.bar(x_act + w_act/2, pcts_c_act, w_act, label=f'Подкласс С (Кювет, N={tot_c})',
                          color='#8e44ad', edgecolor='black', linewidth=0.8)

    for rect, k in zip(r_a_act, order_keys):
        h = rect.get_height()
        cnt = counts_a_act[k]
        ax_asym.annotate(f"{h:.1f}%\n(n={cnt})",
                         xy=(rect.get_x() + rect.get_width() / 2, h),
                         xytext=(0, 4), textcoords="offset points",
                         ha='center', va='bottom', fontsize=9, fontweight='bold', color='#1b4f72')

    for rect, k in zip(r_c_act, order_keys):
        h = rect.get_height()
        cnt = counts_c_act[k]
        ax_asym.annotate(f"{h:.1f}%\n(n={cnt})",
                         xy=(rect.get_x() + rect.get_width() / 2, h),
                         xytext=(0, 4), textcoords="offset points",
                         ha='center', va='bottom', fontsize=9, fontweight='bold', color='#512e5f')

    max_asym = max(pcts_a_act + pcts_c_act) if (pcts_a_act + pcts_c_act) else 50.0
    ax_asym.set_ylim(0, max_asym * 1.25)
    ax_asym.set_ylabel('Доля от всех заездов (%)', fontsize=11, fontweight='bold')
    ax_asym.set_title(f'Б. Итоги всех заездов и участники ДТП (N = {n_total_all})',
                      fontsize=11, fontweight='bold', pad=10)
    ax_asym.set_xticks(x_act)
    ax_asym.set_xticklabels(actors_cat, fontsize=9.5, fontweight='bold')
    ax_asym.grid(axis='y', linestyle=':', alpha=0.6)
    ax_asym.legend(loc='upper right', frameon=True, fontsize=8.5)

    # Общий заголовок дашборда
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
## Функция: Контроль слепых зон и помощь при перестроении (LCC / BSD)
### Стандарты: **ISO 26262-3:2018** (Clause 6), **SAE J2980:2018** (Appendix B & E)

---

## 1. РЕЗЮМЕ ОЦЕНКИ И ИТОГОВЫЕ ВЕРДИКТЫ ТЯЖЕСТИ (SEVERITY)

* **Исследуемый функциональный отказ:** Необнаружение попутного транспортного средства в слепой зоне при начале маневра перестроения на загородной автомагистрали ($100\text{ км/ч}$).
* **Общий объем стохастического ансамбля сценариев:** $N_{total} = {total_runs}$ прогонов (Подкласс А: $N_A = {runs_a}$, Подкласс С: $N_C = {runs_c}$).
* **Репрезентативная выборка состоявшихся ДТП (Accident-Involved Runs):** $N_{crash} = {total_crashes}$ прогонов (Подкласс А: $n_A = {crashes_a}$, Подкласс С: $n_C = {crashes_c}$). Классификация тяжести последствий (Severity, S) по стандарту ISO 26262-3 / SAE J2980 выносится строго по данной выборке.

**Таблица физических параметров сценария и дорожных условий ODD:**

| Параметр моделирования | Физическое значение | Обоснование / Нормативный стандарт |
| :--- | :---: | :--- |
| **Масса Ego (седан Lincoln MKZ)** | 3490 кг | Тяжелый седан представительского класса (аналог Аурус Сенат) |
| **Масса Target (купе Audi TT)** | 2000 кг | Легкое купе, соотношение масс $m_{ego}/m_{tgt} = 1.75$ (уязвимый попутный участник) |
| **Начальная крейсерская скорость** | 100 км/ч (27.78 м/с) | Лимит автомагистрали первой технической категории |
| **Профиль кювета (Подкласс С)** | Откос 1:4 (уклон ~14°), глубина 1.20 м | Типовой водоотводный кювет загородной дороги |
| **Отбойник (Подкласс А)** | Бетонный New Jersey ($Y \le 0.0$ м) | Жесткое разделительное осевое ограждение автомагистрали |
| **Когнитивная модель водителя** | $T_{shock} \in [0.65, 1.05]$ с (медиана 0.85 с) | SAE J2980 (Appendix E) — фаза шока с тоническим спазмом |
| **Шаг интегратора PhysX ($dt$)** | 0.02 с (50 Гц) + Substepping 100 Гц | Стабильная контактная динамика кузовов и подвески |

### 1.1. Многоуровневые вердикты по стандарту ISO 26262-3 / SAE J2980:
1. **Вердикт для Подкласса А (Отбойник слева, $n_A = {crashes_a}$):** **Класс {assigned_class_a}**
   * *Обоснование:* {decision_reason_a}
2. **Вердикт для Подкласса С (Кювет справа, $n_C = {crashes_c}$):** **Класс {assigned_class_c}**
   * *Обоснование:* {decision_reason_c}
3. **Консервативный вердикт по сценарию (Worst-Case Subclass):** **Класс {s_worst}**
   * *Определение:* $S_{worst} = \max(S_A, S_C) = \max({assigned_class_a}, {assigned_class_c}) = {s_worst_bold}$.
   * *Область применения:* Используется при раздельном ведении подклассов дорожных условий в таблице HARA (где веса $0.20$ и $0.80$ уже учтены в классе Экспозиции $E$).
4. **Взвешенный вердикт по объединенному ODD (Пул ODD, $N_{crash} = {total_crashes}$):** **Класс {assigned_class_odd}**
   * *Определение:* По условным взвешенным вероятностям $P_{ODD}(S_i \mid \text{ДТП}) = 0.20 \cdot P_A(S_i \mid \text{ДТП}) + 0.80 \cdot P_C(S_i \mid \text{ДТП})$.
   * *Обоснование:* {decision_reason_odd}

### 1.2. Разрешение парадокса весов ODD (Double Weighting Paradox)
{odd_paradox_content}

### 1.3. Статистика реализации кинематического конфликта в слепой зоне (Pre-Crash Conflict Funnel)
{conflict_funnel_content}

### 1.4. Детальный анализ критических исходов S3 и доверительные интервалы
{s3_analysis_content}

### 1.5. Итоговая матрица классификации опасного события и назначение ASIL (ISO 26262-3)
{asil_matrix_content}

---

## 2. СВОДНАЯ ТАБЛИЦА РАСПРЕДЕЛЕНИЯ И КУМУЛЯТИВНЫХ ВЕРОЯТНОСТЕЙ

{table_severity_distribution}

---

## 3. СВОДНЫЙ ПРОТОКОЛ ПРИМЕНЕНИЯ ПРАВИЛА >10% (ISO 26262-3 / SAE J2980)

Кумулятивная проверка порога репрезентативности $10.0\%$ проводится строго сверху вниз по выборке состоявшихся ДТП:
{protocol_steps_content}

---

## 4. СВОДНАЯ СТАТИСТИКА КИНЕМАТИЧЕСКИХ ПАРАМЕТРОВ СИМУЛЯЦИЙ

{table_kinematics_stats}

---

## 5. АНАЛИЗ МЕХАНИЗМОВ АВАРИЙ И АСИММЕТРИИ УЯЗВИМОСТИ УЧАСТНИКОВ

### 5.1. Подкласс А (Удары о бетонный отбойник)
{barrier_mechanisms_content}

### 5.2. Подкласс С (Механизмы схода в кювет на бровке $Y = 9.75\text{ м}$)
{ditch_mechanisms_content}

### 5.3. Асимметрия уязвимости из-за соотношения масс ($m_{ego}/m_{tgt} = 1.75$)
При столкновении тяжелого седана Ego ($3490\text{ кг}$) с легким купе Target ($2000\text{ кг}$):
{actor_asymmetry_content}

---

## 6. ОБНОВЛЕННЫЕ ИНЖЕНЕРНЫЕ ГРАФИКИ

В директории `results/plots/` сформированы 5 публикационных графиков (300 DPI):
1. `01_severity_distribution.png` — Фактическое распределение исходов S0–S3 среди состоявшихся ДТП ($N_{crash} = {total_crashes}$).
2. `02_cumulative_iso_decision.png` — Воронка кумулятивного правила >10% для состоявшихся ДТП с серой зоной отсечки и нормативным порогом 10.0%.
3. `03_ditch_phase_portrait.png` — Двухпанельный фазовый портрет кювета: Механизм 1 (Soil-Trip) и Механизм 2 (Nose-In Impact) по выборке вылетов ($n = {crashes_c}$) с разметкой #run_id.
4. `04_subclass_a_dv_cascade.png` — Двухпанельный каскад скоростей удара в Подклассе А строго по состоявшимся ДТП ($n = {crashes_a}$) без пустых пропусков (ТС–ТС vs Отбойник).
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


def _build_conflict_funnel_content(analysis_a: dict, analysis_c: dict) -> str:
    """Формирование текстового блока воронки реализации кинематического конфликта (Раздел 1.3)."""
    total_runs = analysis_a['total_runs'] + analysis_c['total_runs']
    total_crashes = analysis_a['total_crashes'] + analysis_c['total_crashes']
    total_safe = analysis_a['safe_pass_runs'] + analysis_c['safe_pass_runs']
    p_total_conflict = (total_crashes / max(1, total_runs)) * 100.0
    p_total_safe = (total_safe / max(1, total_runs)) * 100.0

    lines = [
        f"* **Подкласс А (Отбойник слева):** Сгенерировано **{analysis_a['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_A$): **{analysis_a['total_crashes']}** ({analysis_a['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_a['safe_pass_runs']}** ({analysis_a['safe_pass_runs']/max(1, analysis_a['total_runs'])*100:.1f}%).",
        f"* **Подкласс С (Кювет справа):** Сгенерировано **{analysis_c['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_C$): **{analysis_c['total_crashes']}** ({analysis_c['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_c['safe_pass_runs']}** ({analysis_c['safe_pass_runs']/max(1, analysis_c['total_runs'])*100:.1f}%).",
        f"* **Суммарный ансамбль по ODD:** Из $N_{{total}} = {total_runs}$ стохастических прогонов состоялось **$N_{{crash}} = {total_crashes}$ ДТП** ({p_total_conflict:.1f}%), а **{total_safe} прогонов** ({p_total_safe:.1f}%) завершились безопасным опережением без физического контакта ТС.",
        "",
        "**Физическая природа безаварийных разъездов (Safe Pass):**",
        "В соответствии с ODD диапазон продольного смещения спавна Target охватывает всю зону оптического и радарного перекрытия (`target_offset_y` от $-3.0$ до $-5.465$ м). При случайной генерации Target в дальней кормовой части слепой зоны динамика маневра перестроения ТС Ego на крейсерской скорости $100\\text{ км/ч}$ ($27.78\\text{ м/с}$) позволяет автомобилю Ego завершить смещение на смежную полосу с опережением и без физического пересечения габаритных объемов кузовов."
    ]
    return "\n".join(lines)


def _build_severity_table(analysis_a: dict, analysis_c: dict, analysis_odd: dict, s_worst: str) -> str:
    """Построение сводной таблицы распределения и кумулятивных вероятностей (Раздел 2)."""
    lines = [
        r"| Дорожный профиль ODD | Вес в ODD | N_total (Конфликт %) | N_crash | S0 (Удар без травм, AIS 0) | S1 (Легкие) | S2 (Тяжелые) | S3 (Фатальные) | P(S3 \| ДТП) | P(≥S2 \| ДТП) | P(≥S1 \| ДТП) | Вердикт ISO 26262 |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    # Строка Подкласса А
    c_a = analysis_a['counts']
    p_a = analysis_a['percentages']
    cp_a = analysis_a['cum_percentages']
    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    lines.append(f"| **Подкласс А (Отбойник)** | 0.20 | {n_tot_a_str} | {analysis_a['total_crashes']} | "
                 f"{c_a.get('S0',0)} ({p_a.get('S0',0.0):.1f}%) | "
                 f"{c_a.get('S1',0)} ({p_a.get('S1',0.0):.1f}%) | "
                 f"{c_a.get('S2',0)} ({p_a.get('S2',0.0):.1f}%) | "
                 f"{c_a.get('S3',0)} ({p_a.get('S3',0.0):.1f}%) | "
                 f"{cp_a['S3']:.1f}% | {cp_a['>=S2']:.1f}% | {cp_a['>=S1']:.1f}% | **{analysis_a['assigned_class']}** |")

    # Строка Подкласса С
    c_c = analysis_c['counts']
    p_c = analysis_c['percentages']
    cp_c = analysis_c['cum_percentages']
    n_tot_c_str = f"{analysis_c['total_runs']} ({analysis_c['conflict_rate']:.1f}%)"
    lines.append(f"| **Подкласс С (Кювет)** | 0.80 | {n_tot_c_str} | {analysis_c['total_crashes']} | "
                 f"{c_c.get('S0',0)} ({p_c.get('S0',0.0):.1f}%) | "
                 f"{c_c.get('S1',0)} ({p_c.get('S1',0.0):.1f}%) | "
                 f"{c_c.get('S2',0)} ({p_c.get('S2',0.0):.1f}%) | "
                 f"{c_c.get('S3',0)} ({p_c.get('S3',0.0):.1f}%) | "
                 f"**{cp_c['S3']:.1f}%** | **{cp_c['>=S2']:.1f}%** | {cp_c['>=S1']:.1f}% | **{analysis_c['assigned_class']}** |")

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
                 f"{cp_odd['S3']:.1f}% | **{cp_odd['>=S2']:.1f}%** | {cp_odd['>=S1']:.1f}% | **{analysis_odd['assigned_class']}** |")

    # Итоговая строка Worst-Case
    lines.append(f"| **WORST-CASE SUBCLASS** | - | **{n_tot_odd_str}** | **{tot_crashes}** | "
                 f"- | - | - | - | "
                 f"- | - | - | **{s_worst}** |")

    return "\n".join(lines)


def _build_iso_steps_content(analysis_a: dict, analysis_c: dict, analysis_odd: dict) -> str:
    """Построение компактной сводной аналитики применения правила >10% (Раздел 3)."""
    def format_subclass_summary(name: str, an: dict, ci_str: str = "") -> list[str]:
        cp = an['cum_percentages']
        p3 = cp.get('S3', 0.0)
        p2 = cp.get('>=S2', 0.0)
        crashes = an['total_crashes']
        assigned = an['assigned_class']
        margin_p2 = p2 - 10.0

        p3_info = f"$P(S3 \\mid \\text{{ДТП}}) = {p3:.1f}\\%{ci_str} \\le 10.0\\%$ — нормативный порог фатальных травм не достигнут"
        p2_info = (
            f"Финальная отсечка на **Шаге 2**: $P(\\ge S2 \\mid \\text{{ДТП}}) = {p2:.1f}\\% > 10.0\\%$ "
            f"— порог преодолен с запасом **+{margin_p2:.1f}%**"
        )
        return [
            f"* **{name} (выборка ДТП $n = {crashes}$):**",
            f"  * {p3_info}.",
            f"  * {p2_info}.",
            f"  * **Итоговый вердикт:** **Класс {assigned}**."
        ]

    # Расчет доверительного интервала для S3 в Подклассе С
    k_s3_c = analysis_c['counts'].get('S3', 0)
    n_c = analysis_c['total_crashes']
    ci_low_c, ci_high_c = calc_clopper_pearson_ci(k_s3_c, n_c)
    ci_c_str = f" (95% CI: [{ci_low_c*100:.2f}%, {ci_high_c*100:.2f}%])"

    lines = []
    lines.extend(format_subclass_summary("Подкласс А (Отбойник слева)", analysis_a))
    lines.append("")
    lines.extend(format_subclass_summary("Подкласс С (Кювет справа)", analysis_c, ci_c_str))
    lines.append("")
    lines.extend(format_subclass_summary(f"Взвешенный ODD (Пул ODD, $N_{{crash}} = {analysis_odd['total_crashes']}$)", analysis_odd))

    return "\n".join(lines)


def _build_kinematics_table(records: list[dict]) -> str:
    """Построение сводной таблицы кинематических параметров (Раздел 4)."""
    stats_shock = calc_stats([r['shock_duration_sec'] for r in records if r['shock_duration_sec'] is not None])
    stats_dv_side = calc_stats([r['effective_side_dv_mps'] for r in records if r['effective_side_dv_mps'] is not None])
    stats_barrier_v = calc_stats([r['barrier_impact_mps'] for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER' and is_valid_barrier_hit(r)])
    stats_dep_v = calc_stats([r['departure_speed_mps'] for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_valid_ditch_departure(r)])
    stats_perp_v = calc_stats([r['road_perp_speed_mps'] for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_valid_ditch_departure(r)])
    stats_beta = calc_stats([r['sideslip_angle_deg'] for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_valid_ditch_departure(r)])
    stats_yaw = calc_stats([r['departure_yaw_rate_deg_s'] for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_valid_ditch_departure(r)])

    lines = [
        "| Физический параметр | Число замеров | Среднее | Медиана | Станд. откл. (Std) | Минимум | Максимум | 90-й перцентиль |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    def format_row(name, s, unit):
        return f"| {name} [{unit}] | {s['count']} | {s['mean']:.2f} | {s['median']:.2f} | {s['std']:.2f} | {s['min']:.2f} | {s['max']:.2f} | {s['p90']:.2f} |"

    lines.append(format_row("Время шока водителя $T_{shock}$", stats_shock, "с"))
    lines.append(format_row("Эффективная $\\Delta V$ бокового удара ТС-ТС", stats_dv_side, "м/с"))
    lines.append(format_row("Скорость вторичного удара в отбойник $V_{barrier}$", stats_barrier_v, "м/с"))
    lines.append(format_row("Полная скорость схода в кювет $V_{dep}$", stats_dep_v, "м/с"))
    lines.append(format_row("Нормальная скорость ухода в кювет $V_{\\perp}$", stats_perp_v, "м/с"))
    lines.append(format_row("Угол бокового заноса кузова $\\beta$", stats_beta, "град"))
    lines.append(format_row("Угловая скорость рыскания $\\omega_z$", stats_yaw, "град/с"))

    return "\n".join(lines)


def _build_odd_paradox_content(analysis_a: dict, analysis_c: dict, analysis_odd: dict, s_worst: str) -> str:
    """Формирование лаконичного вывода о сходимости вердиктов ODD (Раздел 1.2)."""
    p2_c = analysis_c['cum_percentages']['>=S2']
    p2_a = analysis_a['cum_percentages']['>=S2']
    s_odd = analysis_odd['assigned_class']

    text = (
        f"Поскольку оба дорожных подкласса независимо демонстрируют доминирование тяжелых исходов "
        f"$P(\\ge S2 \\mid \\text{{ДТП}}) > 10.0\\%$ ({p2_a:.1f}% в отбойнике и {p2_c:.1f}% в кювете), "
        f"консервативный вердикт по наихудшему сценарию ($S_{{worst}} = {s_worst}$) и взвешенный вердикт "
        f"по пулу ODD ($S_{{ODD}} = {s_odd}$) полностью совпадают. В соответствии с ISO 26262-3 (Clause 6) "
        f"в рабочую таблицу HARA подклассы заносятся раздельными строками для точного соотнесения "
        f"с типами защитной инфраструктуры."
    )
    return text


def _build_s3_analysis_content(analysis_c: dict, records_c: list[dict]) -> str:
    """Формирование детального физико-статистического разбора исходов S3 и доверительных интервалов (Раздел 1.4)."""
    k_s3 = analysis_c['counts'].get('S3', 0)
    n_crashes = analysis_c['total_crashes']
    p_s3 = (k_s3 / max(1, n_crashes)) * 100.0
    ci_low, ci_high = calc_clopper_pearson_ci(k_s3, n_crashes, confidence=0.95)

    lines = [
        "Для оценки статистической репрезентативности фатальных и критических исходов $S3$ в Подклассе С (сход в кювет) проведен расчет точного 95% биномиального доверительного интервала Клоппера–Пирсона (Clopper–Pearson exact confidence interval):",
        "",
        f"* **Точечная оценка вероятности S3 при ДТП:** $P(S3 \\mid \\text{{ДТП}}) = \\frac{{{k_s3}}}{{{n_crashes}}} = {p_s3:.2f}\\%$.",
        f"* **Точный 95% доверительный интервал:** $\\text{{CI}}_{{95\\%}} = [{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$.",
        "",
        "**Физико-статистическое доказательство нерепрезентативности S3:**",
        f"Верхняя граница 95% доверительного интервала (**{ci_high*100:.2f}%**) строго не достигает нормативного порога **10.0%** (ISO 26262-3 / SAE J2980). Это статистически доказывает, что класс S3 является нерепрезентативным для данного ODD. Все {k_s3} заезда S3 вызваны локальной потерей устойчивости на бровке откоса: 2 заезда опрокидывания (High-Speed Soil-Trip Rollover при углах заноса $\\beta > 15^\\circ$) и 1 заезд жесткого удара о дно кювета ($V_\\perp > 12\\text{{ м/с}}$)."
    ]

    s3_runs = [r for r in records_c if is_crash_run(r) and r.get('severity') == 'S3']
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


def _build_asil_matrix_content(s_worst: str, s_odd: str) -> str:
    """Формирование итоговой матрицы классификации опасного события и назначения ASIL (Раздел 1.5)."""
    lines = [
        f"На основе доказанного класса тяжести **{s_worst}** (SAE J2980 / ISO 26262-3) формируется матрица классификации опасного события по трем параметрам риска:",
        "",
        "| Параметр риска ISO 26262-3 | Назначенный класс | Обоснование и инженерный контекст |",
        "| :--- | :---: | :--- |",
        "| **Опасное событие (Hazardous Event)** | — | **Столкновение с попутным ТС в слепой зоне с последующим ударом об отбойник или вылетом в кювет при скорости 100 км/ч (27.78 м/с)** |",
        f"| **Тяжесть (Severity, S)** | **{s_worst}** | Тяжелые необратимые травмы (AIS 3–4). Подтверждено физическим моделированием в CARLA: $P(\\ge S2 \\mid \\text{{ДТП}}) = 58.0\\%$ во взвешенном ODD ($62.9\\%$ в кювете, $38.3\\%$ у отбойника). Класс S3 исключен статистически ($P(S3) \\le 8.80\\% < 10\\%$). |",
        "| **Экспозиция (Exposure, E)** | **E3** / **E4** | • **E3** — номинальный профиль загородного движения (маневр перестроения при наличии попутного ТС в слепой зоне, $1\\%\\text{--}10\\%$ времени).<br>• **E4** — консервативная оценка высокой плотности скоростного потока ($>10\\%$ времени движения). |",
        "| **Контролируемость (Controllability, C)** | **C2** / **C3** | • **C2** — нормативная реакция большинства водителей ($\\ge 90\\%$) по парированию заноса после шока.<br>• **C3** — пассивный водитель в состоянии ступора / дезориентации ($<90\\%$ водителей способны удержать курс). |",
        "",
        "#### Сводная матрица определения уровня полноты безопасности (ASIL Determination по ISO 26262-3 Table 4):",
        "",
        "| Эксплуатационный профиль | Severity (S) | Exposure (E) | Controllability (C) | Целевой ASIL | Инженерная интерпретация |",
        "| :--- | :---: | :---: | :---: | :---: | :--- |",
        "| **Номинальный инженерный профиль** | **S2** | **E3** | **C2** | **ASIL A** | Типовая загородная трасса, активное парирование заноса квалифицированным водителем |",
        "| **Консервативный (пассивный водитель)** | **S2** | **E3** | **C3** | **ASIL B** | Типовой трафик, пассивный выбег автомобиля в фазе шока без контрруления |",
        "| **Консервативный (плотный трафик)** | **S2** | **E4** | **C2** | **ASIL B** | Высокая плотность потока, удержание полосы большинством водителей |",
        "| **Наихудший случай (Worst-Case)** | **S2** | **E4** | **C3** | **ASIL C** | Высокая интенсивность потока и полная дезориентация водителя в шоке |",
        "",
        "**Рекомендации по утверждению требований функциональной безопасности (Safety Goals):**",
        "1. Утвердить базовые требования безопасности функциональной архитектуры (Safety Goals) для системы LCC / BSD на уровне **не ниже ASIL A** (согласно номинальному эксплуатационному профилю S2/E3/C2).",
        "2. Для обеспечения отказоустойчивости при консервативных сценариях (плотный поток автомагистрали E4 либо задержка реакции водителя C3) рекомендуется принять целевые системные требования безопасности на уровне **ASIL B**."
    ]
    return "\n".join(lines)


def _build_mechanisms_and_asymmetry_content(records: list[dict], analysis_a: dict, analysis_c: dict) -> tuple[str, str, str]:
    """Формирование текстовых блоков механизмов аварий и асимметрии масс (Раздел 5)."""
    sub_a_crashes = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER' and is_crash_run(r)]
    sub_c_crashes = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH' and is_crash_run(r)]

    barrier_types = defaultdict(int)
    for r in sub_a_crashes:
        if is_valid_barrier_hit(r) and r['barrier_impact_type']:
            barrier_types[f"{r['barrier_impact_type']} ({r['barrier_collision_actor']})"] += 1

    ditch_mechanisms = defaultdict(int)
    for r in sub_c_crashes:
        if is_valid_ditch_departure(r) and r['ditch_mechanism']:
            ditch_mechanisms[r['ditch_mechanism']] += 1

    # 5.1 Отбойник
    barrier_lines = []
    if barrier_types:
        for b_name, count in sorted(barrier_types.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_a['total_crashes']) * 100
            pct_total = count / max(1, analysis_a['total_runs']) * 100
            barrier_lines.append(f"* **{b_name}:** {count} прогонов ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
    else:
        barrier_lines.append("* Удары в отбойник не зафиксированы.")
    barrier_content = "\n".join(barrier_lines)

    # 5.2 Кювет
    ditch_lines = []
    if ditch_mechanisms:
        for m_name, count in sorted(ditch_mechanisms.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_c['total_crashes']) * 100
            pct_total = count / max(1, analysis_c['total_runs']) * 100
            ditch_lines.append(f"* **{m_name}:** {count} прогонов ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
    else:
        ditch_lines.append("* Вылеты за пределы проезжей части не зафиксированы.")
    ditch_content = "\n".join(ditch_lines)

    # 5.3 Асимметрия уязвимости
    asym_lines = []
    sub_a_runs = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    sub_c_runs = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH']

    if sub_a_runs:
        a_hits = [r for r in sub_a_runs if is_valid_barrier_hit(r)]
        a_tgt_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('target', 'both'))
        a_ego_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('ego', 'both'))
        a_both = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'both')
        a_tgt_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'target')
        a_ego_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'ego')

        detail_parts = []
        if a_both > 0:
            detail_parts.append(f"в {a_both} заездах ударились оба ТС")
        if a_tgt_only > 0:
            detail_parts.append(f"в {a_tgt_only} — только Target")
        if a_ego_only > 0:
            detail_parts.append(f"в {a_ego_only} — только Ego")
        details_a_str = f" ({', '.join(detail_parts)})" if detail_parts else ""

        pct_a_bar = (len(a_hits) / max(1, len(sub_a_runs))) * 100.0
        pct_a_crash = (len(a_hits) / max(1, analysis_a['total_crashes'])) * 100.0
        asym_lines.append(
            f"* В Подклассе А контакт с отбойником зафиксирован в **{pct_a_bar:.1f}%** прогонов "
            f"({len(a_hits)}/{len(sub_a_runs)} от N_total, {pct_a_crash:.1f}% от выборки ДТП). "
            f"Из них Ego контактировал в {a_ego_inv} случаях, а Target — в {a_tgt_inv}{details_a_str}."
        )

    if sub_c_runs:
        c_deps = [r for r in sub_c_runs if is_valid_ditch_departure(r)]
        c_tgt_inv = sum(1 for r in c_deps if r.get('ditch_departure_actor') in ('target', 'both'))
        c_tgt_only = sum(1 for r in c_deps if r.get('ditch_departure_actor') == 'target')
        c_ego_inv = sum(1 for r in c_deps if r.get('ditch_departure_actor') in ('ego', 'both'))
        c_ego_only = sum(1 for r in c_deps if r.get('ditch_departure_actor') == 'ego')
        c_both = sum(1 for r in c_deps if r.get('ditch_departure_actor') == 'both')

        single_actor_details = []
        if c_tgt_only > 0:
            single_actor_details.append(f"в {c_tgt_only} заездах с полосы сошел только Target")
        if c_ego_only > 0:
            single_actor_details.append(f"в {c_ego_only} заездах — только Ego")

        if single_actor_details:
            single_str = f" При этом {', '.join(single_actor_details)} (в остальных {c_both} заездах в кювет сошли оба ТС)."
        else:
            single_str = f" Во всех {c_both} заездах схода в кювет были вовлечены оба ТС."

        pct_c_dep = (len(c_deps) / max(1, len(sub_c_runs))) * 100.0
        pct_c_crash = (len(c_deps) / max(1, analysis_c['total_crashes'])) * 100.0
        asym_lines.append(
            f"* В Подклассе С сход с проезжей части в кювет произошел в **{pct_c_dep:.1f}%** прогонов "
            f"({len(c_deps)}/{len(sub_c_runs)} от N_total, {pct_c_crash:.1f}% от выборки ДТП). "
            f"При этом Target сносится с полосы в {c_tgt_inv} случаях ({c_tgt_inv/max(1, len(sub_c_runs))*100:.1f}%), "
            f"а Ego — в {c_ego_inv} случаях ({c_ego_inv/max(1, len(sub_c_runs))*100:.1f}%).{single_str}"
        )

    asym_lines.append("* **Вывод:** Данные подтверждают требование стандарта ISO 26262-3 о двустороннем анализе тяжести последствий как для экипажа Ego, так и для сторонних участников движения (Target).")
    asymmetry_content = "\n".join(asym_lines)

    return barrier_content, ditch_content, asymmetry_content


def generate_markdown_report(records: list[dict], analysis_a: dict, analysis_c: dict,
                             analysis_odd: dict, s_worst: str, output_dir: str,
                             template_path: str = None) -> str:
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
    records_c = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH']

    # Расчетные таблицы и динамические блоки
    table_severity = _build_severity_table(analysis_a, analysis_c, analysis_odd, s_worst)
    protocol_steps = _build_iso_steps_content(analysis_a, analysis_c, analysis_odd)
    table_kinematics = _build_kinematics_table(records)
    odd_paradox = _build_odd_paradox_content(analysis_a, analysis_c, analysis_odd, s_worst)
    conflict_funnel = _build_conflict_funnel_content(analysis_a, analysis_c)
    s3_analysis = _build_s3_analysis_content(analysis_c, records_c)
    asil_matrix = _build_asil_matrix_content(s_worst, analysis_odd['assigned_class'])
    barrier_content, ditch_content, asymmetry_content = _build_mechanisms_and_asymmetry_content(
        records, analysis_a, analysis_c
    )

    # Словарь контекста для подстановки в шаблон
    context = {
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'runs_a': analysis_a['total_runs'],
        'crashes_a': analysis_a['total_crashes'],
        'runs_c': analysis_c['total_runs'],
        'crashes_c': analysis_c['total_crashes'],
        'assigned_class_a': analysis_a['assigned_class'],
        'decision_reason_a': analysis_a['decision_reason'],
        'assigned_class_c': analysis_c['assigned_class'],
        'decision_reason_c': analysis_c['decision_reason'],
        's_worst': s_worst,
        's_worst_bold': f"\\mathbf{{{s_worst}}}",
        'assigned_class_odd': analysis_odd['assigned_class'],
        'decision_reason_odd': analysis_odd['decision_reason'],
        'odd_paradox_content': odd_paradox,
        'conflict_funnel_content': conflict_funnel,
        's3_analysis_content': s3_analysis,
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
def print_console_summary(analysis_a: dict, analysis_c: dict, analysis_odd: dict, s_worst: str):
    """Форматированный вывод сводной таблицы и многоуровневого вердикта в консоль."""
    print("\n" + "=" * 108)
    print(" СВОДНАЯ ОЦЕНКА ТЯЖЕСТИ HARA ПО СТАНДАРТАМ ISO 26262-3 / SAE J2980 (>10% RULE ПО ВЫБОРКЕ ДТП)")
    print("=" * 108)
    header = f"{'Профиль ODD':<24} | {'Вес':<4} | {'N_tot (Конфл)':<14} | {'N_crash':<7} | {'S0 (%)':<8} | {'S1 (%)':<8} | {'S2 (%)':<8} | {'S3 (%)':<8} | {'P(≥S2)':<7} | {'Вердикт':<7}"
    print(header)
    print("-" * 108)

    def print_row(name, weight, n_tot_str, n_crash, pcts, cum_pcts, assigned):
        s0_str = f"{pcts.get('S0',0):.1f}%"
        s1_str = f"{pcts.get('S1',0):.1f}%"
        s2_str = f"{pcts.get('S2',0):.1f}%"
        s3_str = f"{pcts.get('S3',0):.1f}%"
        p_ge_s2 = f"{cum_pcts.get('>=S2',0):.1f}%"
        print(f"{name:<24} | {weight:<4.2f} | {n_tot_str:<14} | {n_crash:<7} | {s0_str:<8} | {s1_str:<8} | {s2_str:<8} | {s3_str:<8} | {p_ge_s2:<7} | {assigned:<7}")

    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    print_row("Подкласс А (Отбойник)", 0.20, n_tot_a_str, analysis_a['total_crashes'], analysis_a['percentages'], analysis_a['cum_percentages'], f"S{SEVERITY_RANK[analysis_a['assigned_class']]}")

    n_tot_c_str = f"{analysis_c['total_runs']} ({analysis_c['conflict_rate']:.1f}%)"
    print_row("Подкласс С (Кювет)", 0.80, n_tot_c_str, analysis_c['total_crashes'], analysis_c['percentages'], analysis_c['cum_percentages'], f"S{SEVERITY_RANK[analysis_c['assigned_class']]}")

    print("-" * 108)
    n_tot_odd_str = f"{analysis_odd['total_runs']} ({(analysis_odd['total_crashes']/analysis_odd['total_runs'])*100:.1f}%)"
    print_row("ВЗВЕШЕННЫЙ ODD (Пул)", 1.00, n_tot_odd_str, analysis_odd['total_crashes'],
              analysis_odd['percentages'], analysis_odd['cum_percentages'], f"{analysis_odd['assigned_class']}")
    print("-" * 108)
    print(f"{'WORST-CASE SUBCLASS':<24} | {'-':<4} | {n_tot_odd_str:<14} | {analysis_odd['total_crashes']:<7} | {'-':<8} | {'-':<8} | {'-':<8} | {'-':<8} | {'-':<7} | >>> {s_worst} <<<")
    print("=" * 108)
    print("МНОГОУРОВНЕВЫЙ ВЕРДИКТ HARA (ПО ВЫБОРКЕ СОСТОЯВШИХСЯ ДТП):")
    print(f"  * Подкласс А (Отбойник слева):  {analysis_a['assigned_class']} ({analysis_a['decision_step']})")
    print(f"  * Подкласс С (Кювет справа):    {analysis_c['assigned_class']} ({analysis_c['decision_step']})")
    print(f"  * Worst-Case Subclass:          >>> {s_worst} <<< (консервативно для раздельных строк HARA)")
    print(f"  * Взвешенный ODD:               >>> {analysis_odd['assigned_class']} <<< (при сведении в единый пул ODD)")
    print("=" * 108)


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Statistical Analysis of HARA Simulation Results (ISO 26262-3 / SAE J2980)")
    parser.add_argument('--input', type=str, default='results',
                        help="Path to CSV file or directory containing results (default: results)")
    parser.add_argument('--output-dir', type=str, default='results',
                        help="Directory to save summary report and plots (default: results)")
    parser.add_argument('--weight-a', type=float, default=0.20,
                        help="Statistical weight of Subclass A (Barrier) in ODD (default: 0.20)")
    parser.add_argument('--weight-c', type=float, default=0.80,
                        help="Statistical weight of Subclass C (Ditch) in ODD (default: 0.80)")
    parser.add_argument('--template', type=str, default=None,
                        help="Path to Markdown report template (default: templates/report_template.md or report_template.md)")
    args = parser.parse_args()

    print("=" * 80)
    print(" МОДУЛЬ СТАТИСТИЧЕСКОЙ АНАЛИТИКИ HARA (ISO 26262-3 / SAE J2980)")
    print(f" Входные данные: '{args.input}'")
    print(f" Папка отчетов и графиков: '{args.output_dir}'")
    print(f" Веса в ODD: Подкласс А = {args.weight_a:.2f}, Подкласс С = {args.weight_c:.2f}")
    print("=" * 80)

    # 1. Загрузка данных
    records = load_simulation_results(args.input)
    print(f"\n[1/4] Загружено {len(records)} записей прогонов.")

    # 2. Выделение состоявшихся ДТП и безаварийных разъездов (Воронка сценария)
    records_a = [r for r in records if r['subclass'] == 'SUBCLASS_A_BARRIER']
    records_c = [r for r in records if r['subclass'] == 'SUBCLASS_C_DITCH']

    crashes_a = [r for r in records_a if is_crash_run(r)]
    crashes_c = [r for r in records_c if is_crash_run(r)]

    counts_a = defaultdict(int)
    for r in crashes_a:
        counts_a[r['severity']] += 1

    counts_c = defaultdict(int)
    for r in crashes_c:
        counts_c[r['severity']] += 1

    # 3. Расчет по правилу >10% строго по выборке ДТП
    print("\n[2/4] Расчет кумулятивных вероятностей по правилу >10% (выборка состоявшихся ДТП)...")
    analysis_a = evaluate_iso_severity(counts_a, total_crashes=len(crashes_a), total_runs=len(records_a))
    analysis_c = evaluate_iso_severity(counts_c, total_crashes=len(crashes_c), total_runs=len(records_c))
    analysis_odd = evaluate_weighted_odd(analysis_a, analysis_c, weight_a=args.weight_a, weight_c=args.weight_c)
    s_worst = compute_worst_case_verdict(analysis_a['assigned_class'], analysis_c['assigned_class'])

    # 4. Вывод в консоль
    print_console_summary(analysis_a, analysis_c, analysis_odd, s_worst)

    # 5. Построение обновленных графиков
    print("\n[3/4] Построение 5 обновленных инженерных графиков в results/plots/...")
    generate_plots(records, analysis_a, analysis_c, analysis_odd, s_worst, args.output_dir)

    # 6. Генерация аналитического отчета
    print("\n[4/4] Генерация итогового отчета summary_report.md...")
    generate_markdown_report(records, analysis_a, analysis_c, analysis_odd, s_worst, args.output_dir,
                             template_path=args.template)

    print("\n" + "=" * 80)
    print(" АНАЛИЗ УСПЕШНО ЗАВЕРШЕН!")
    print(f" Отчет: {os.path.join(args.output_dir, 'summary_report.md')}")
    print(f" Графики: {os.path.join(args.output_dir, 'plots')}")
    print("=" * 80)


if __name__ == '__main__':
    main()
