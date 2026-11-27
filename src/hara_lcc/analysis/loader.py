"""
Загрузка, нормализация и верификация CSV-результатов симуляции HARA.
Переиспользует единые классификаторы из `hara_lcc.severity.classifiers` без дублирования порогов.
"""

from __future__ import annotations

import csv
import glob
import os

from hara_lcc.config import (
    SEVERITY_CLASSES,
    SEVERITY_FROM_RANK,
    SEVERITY_RANK,
    SUBCLASS_A,
    SUBCLASS_B,
)
from hara_lcc.severity.classifiers import (
    classify_barrier_severity,
    classify_ditch_boundary,
    classify_side_severity,
)
from hara_lcc.severity.iso26262 import is_crash_run, is_spawn_error_run


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


def to_bool(val) -> bool:
    """Безопасное преобразование строки в bool."""
    if isinstance(val, bool):
        return val
    if not val:
        return False
    val_str = str(val).strip().lower()
    return val_str in ('true', '1', 't', 'yes', 'y')


def load_simulation_results(input_path: str) -> list[dict]:
    """
    Загрузка результатов моделирования из CSV-файла или директории.
    Поддерживает как новую 25-колоночную схему, так и легаси CSV.
    """
    files_to_read: list[str] = []
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

    records: list[dict] = []
    for fpath in files_to_read:
        with open(fpath, mode='r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Нормализация имени подкласса (унификация A / B с поддержкой легаси C)
                subclass_raw = row.get('subclass', '').strip()
                if 'barrier' in subclass_raw.lower() or subclass_raw == 'A':
                    subclass = SUBCLASS_A
                elif 'ditch' in subclass_raw.lower() or subclass_raw in ('B', 'C'):
                    subclass = SUBCLASS_B
                else:
                    subclass = subclass_raw

                # Извлечение численных параметров для верификации классов через единые классификаторы
                eff_side_dv = to_float(row.get('effective_side_dv_mps'))
                if eff_side_dv is not None:
                    sev_side = SEVERITY_FROM_RANK[classify_side_severity(eff_side_dv)]
                else:
                    raw_s = row.get('sev_side', 'S0').strip().upper()
                    sev_side = raw_s if raw_s in SEVERITY_CLASSES else 'S0'

                barrier_hit = to_bool(row.get('barrier_collision'))
                v_barrier = to_float(row.get('barrier_impact_mps'))
                b_type = row.get('barrier_impact_type', '').strip()
                if barrier_hit and v_barrier is not None and v_barrier > 0:
                    sev_barrier = SEVERITY_FROM_RANK[classify_barrier_severity(v_barrier, b_type)]
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
                    ditch_sev_int, ditch_mech = classify_ditch_boundary(
                        v_dep=v_dep,
                        v_perp_road=v_perp,
                        v_lat_car=v_lat,
                        beta=beta,
                        omega_z=omega_z,
                    )
                    sev_ditch = SEVERITY_FROM_RANK[ditch_sev_int]
                else:
                    raw_d = row.get('sev_ditch', 'S0').strip().upper()
                    sev_ditch = raw_d if raw_d in SEVERITY_CLASSES else 'S0'
                    ditch_mech = row.get('ditch_mechanism', '')

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
                    'ditch_mechanism': ditch_mech,
                    'sev_ditch': sev_ditch,
                    'severity': severity,
                    'scenario_outcome': row.get('scenario_outcome', ''),
                }
                rec['is_crash'] = is_crash_run(rec)
                rec['is_spawn_error'] = is_spawn_error_run(rec)
                records.append(rec)

    return records
