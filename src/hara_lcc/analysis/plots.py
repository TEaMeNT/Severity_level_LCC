"""
Модульная генерация 5 публикационных инженерных графиков HARA (300 DPI):
1. 01_severity_distribution.png
2. 02_cumulative_iso_decision.png
3. 03_ditch_phase_portrait.png
4. 04_subclass_a_dv_cascade.png
5. 05_kinematics_vs_severity_boxplots.png
"""

from __future__ import annotations

import math
import os
import time

import matplotlib
matplotlib.use('Agg')  # Headless backend для автономной генерации графиков
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from hara_lcc.analysis.stats import format_iso_pct
from hara_lcc.config import (
    DITCH_NON_TRACKING_BETA_DEG,
    DITCH_S1_SKID_VLAT_MPS,
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
    SEVERITY_CLASSES,
    SEVERITY_COLORS,
    SEVERITY_RANK,
    SUBCLASS_A,
    SUBCLASS_B,
)
from hara_lcc.severity.iso26262 import (
    calc_clopper_pearson_ci,
    compute_worst_case_verdict,
    is_crash_run,
    is_spawn_error_run,
    is_valid_barrier_hit,
    is_valid_ditch_departure,
)

# Настройка шрифтов для корректного отображения кириллицы на Windows/Linux
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Segoe UI', 'Tahoma', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False


def safe_savefig(path: str, **kwargs):
    """Безопасное сохранение графика с повторными попытками при временных блокировках Windows."""
    for attempt in range(5):
        try:
            plt.savefig(path, **kwargs)
            return
        except OSError:
            if attempt < 4:
                time.sleep(0.3)
            else:
                raise


def annotate_ditch_points(
    ax,
    points: list[dict],
    x_key: str,
    y_key: str,
    x_span: float,
    y_span: float,
    is_panel_a: bool = False,
):
    """
    Автоматическое разведение подписей #run_id из кандидатов с тонкими выносными линиями.
    Использует 2D Bounding Box collision detection для полного устранения наложений.
    Для S0/S1 в плотном кластере на Панели А выполняет адаптивное прореживание.
    """
    candidates = [
        (6, 6), (6, -10), (-24, 6), (-24, -10),
        (0, 10), (0, -13), (10, 0), (-28, 0),
        (12, 8), (-28, -12), (12, -12), (-30, 8),
    ]
    pts_norm = []
    for pt in points:
        x_val = pt[x_key] if pt[x_key] is not None else 0.0
        y_val = pt[y_key] if pt[y_key] is not None else 0.0
        pts_norm.append((x_val / max(0.1, x_span), y_val / max(0.1, y_span)))

    placed_boxes = []
    placed_points = []

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

            boundary_penalty = 0.0
            if bx0 < 0.01:
                boundary_penalty += (0.01 - bx0) * 500.0
            if bx1 > 0.99:
                boundary_penalty += (bx1 - 0.99) * 500.0
            if by0 < 0.01:
                boundary_penalty += (0.01 - by0) * 500.0
            if by1 > 0.99:
                boundary_penalty += (by1 - 0.99) * 500.0

            overlap_label_penalty = 0.0
            for px0, py0, px1, py1 in placed_boxes:
                ox = max(0.0, min(bx1, px1) - max(bx0, px0))
                oy = max(0.0, min(by1, py1) - max(by0, py0))
                if ox > 0.0 and oy > 0.0:
                    overlap_label_penalty += (ox * oy) * 10000.0 + 10.0

            point_cover_penalty = 0.0
            for j, (pnx, pny) in enumerate(pts_norm):
                if j == idx:
                    continue
                if (bx0 - 0.008 <= pnx <= bx1 + 0.008) and (by0 - 0.008 <= pny <= by1 + 0.008):
                    point_cover_penalty += 25.0

            cx = (bx0 + bx1) / 2.0
            cy = (by0 + by1) / 2.0
            d_pts = min([math.hypot(cx - pnx, cy - pny) for j, (pnx, pny) in enumerate(pts_norm) if j != idx] or [1.0])
            d_lbls = min([math.hypot(cx - (px0 + px1) / 2, cy - (py0 + py1) / 2) for px0, py0, px1, py1 in placed_boxes] or [1.0])

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
            arrowprops=dict(arrowstyle='-', color='gray', lw=0.6, alpha=0.7),
        )


def _get_cum_ci_95(analysis: dict, key: str) -> tuple[float, float]:
    """Извлечение 95% ДИ Клоппера–Пирсона для кумулятивной доли ('S3', '>=S2', '>=S1')."""
    cum_ci = analysis.get('cum_ci_95')
    if isinstance(cum_ci, dict) and key in cum_ci:
        return cum_ci[key]
    n = int(analysis.get('total_crashes', 0))
    counts = analysis.get('counts', {})
    if key == 'S3':
        k = int(counts.get('S3', 0))
    elif key == '>=S2':
        k = int(counts.get('S3', 0) + counts.get('S2', 0))
    else:
        k = int(counts.get('S3', 0) + counts.get('S2', 0) + counts.get('S1', 0))
    return calc_clopper_pearson_ci(k, n, confidence=0.95)


def plot_severity_distribution(analysis_a: dict, analysis_b: dict, analysis_odd: dict, plots_dir: str) -> str:
    """График 1: 01_severity_distribution.png — фактическое распределение S0–S3 среди ДТП."""
    n_crash_total = analysis_a['total_crashes'] + analysis_b['total_crashes']
    w_a = analysis_odd['weights']['A']
    w_b = analysis_odd['weights']['B']

    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')

    fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=300)
    x = np.arange(len(SEVERITY_CLASSES))
    width = 0.26

    vals_a = [analysis_a['percentages'][s] for s in SEVERITY_CLASSES]
    vals_b = [analysis_b['percentages'][s] for s in SEVERITY_CLASSES]
    vals_odd = [analysis_odd['percentages'][s] for s in SEVERITY_CLASSES]

    rects_a = ax.bar(
        x - width, vals_a, width,
        label=(
            f"1.1.A Подкласс А (Ограждение, n={analysis_a['total_crashes']}, "
            f"P(≥S2)={format_iso_pct(analysis_a['cum_percentages']['>=S2'])} "
            f"[95% ДИ {ci2_a_lo*100:.1f}–{ci2_a_hi*100:.1f}%])"
        ),
        color='#2980b9', edgecolor='black', linewidth=0.8, alpha=0.9,
    )
    rects_b = ax.bar(
        x, vals_b, width,
        label=(
            f"1.1.B Подкласс B (Кювет, n={analysis_b['total_crashes']}, "
            f"P(≥S2)={format_iso_pct(analysis_b['cum_percentages']['>=S2'])} "
            f"[95% ДИ {ci2_b_lo*100:.1f}–{ci2_b_hi*100:.1f}%])"
        ),
        color='#8e44ad', edgecolor='black', linewidth=0.8, alpha=0.9,
    )
    rects_odd = ax.bar(
        x + width, vals_odd, width,
        label=f"Взвешенный ODD ({w_a:.2f}A + {w_b:.2f}B, справочно)",
        color='#7f8c8d', edgecolor='black', linewidth=0.8, alpha=0.65,
    )

    for rect, s in zip(rects_a, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_a['counts'].get(s, 0)
        ax.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=9, fontweight='bold', color='#1a5276',
        )

    for rect, s in zip(rects_b, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = analysis_b['counts'].get(s, 0)
        ax.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=9, fontweight='bold', color='#5b2c6f',
        )

    for rect, s in zip(rects_odd, SEVERITY_CLASSES):
        h = rect.get_height()
        ax.annotate(
            f"{h:.1f}%",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=8.5, fontweight='bold', color='#566573',
        )

    max_h = max(vals_a + vals_b + vals_odd + [10.0])
    ax.set_ylim(0, max_h * 1.28)

    ax.set_ylabel('Доля от числа ДТП (%)', fontsize=12, fontweight='bold')
    ax.set_title(
        f'Распределение тяжести последствий (S0–S3) в состоявшихся ДТП (n = {n_crash_total})',
        fontsize=13, fontweight='bold', pad=15,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(
        ['S0\n(Удар без травм)', 'S1\n(Легкие травмы)', 'S2\n(Тяжелые травмы)', 'S3\n(Опасные для жизни)'],
        fontsize=10, fontweight='bold',
    )
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, fontsize=8.8)
    plt.tight_layout()
    plot1_path = os.path.join(plots_dir, "01_severity_distribution.png")
    safe_savefig(plot1_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot1_path


def plot_cumulative_iso_decision(analysis_a: dict, analysis_b: dict, analysis_odd: dict, plots_dir: str) -> str:
    """
    График 2: 02_cumulative_iso_decision.png — воронка кумулятивного правила >10% ISO 26262-3
    для раздельных строк HARA 1.1.A и 1.1.B с 95% ДИ Клоппера–Пирсона (SAE J2980 §5.2.1.5(g), §B.1).
    """
    fig, ax = plt.subplots(figsize=(11.2, 6.6), dpi=300)
    steps = [
        'Шаг 1: P(S3 | ДТП)\n[Критические травмы]',
        'Шаг 2: P(≥S2 | ДТП)\n[Тяжелые и критические]',
        'Шаг 3: P(≥S1 | ДТП)\n[Любые травмы]',
    ]
    step_keys = ['S3', '>=S2', '>=S1']
    x = np.arange(len(steps))
    width = 0.34

    vals_a_cum = [analysis_a['cum_percentages'][k] for k in step_keys]
    vals_b_cum = [analysis_b['cum_percentages'][k] for k in step_keys]

    cis_a = [_get_cum_ci_95(analysis_a, k) for k in step_keys]
    cis_b = [_get_cum_ci_95(analysis_b, k) for k in step_keys]

    yerr_a = np.array([
        [max(0.0, v - lo * 100.0) for v, (lo, _) in zip(vals_a_cum, cis_a)],
        [max(0.0, hi * 100.0 - v) for v, (_, hi) in zip(vals_a_cum, cis_a)],
    ])
    yerr_b = np.array([
        [max(0.0, v - lo * 100.0) for v, (lo, _) in zip(vals_b_cum, cis_b)],
        [max(0.0, hi * 100.0 - v) for v, (_, hi) in zip(vals_b_cum, cis_b)],
    ])

    s_worst = compute_worst_case_verdict(analysis_a['assigned_class'], analysis_b['assigned_class'])

    r_a = ax.bar(
        x - width / 2, vals_a_cum, width,
        yerr=yerr_a, capsize=4.5, error_kw=dict(ecolor='#1b4f72', elinewidth=1.3, capthick=1.2, zorder=6),
        label=f"Строка HARA 1.1.A — Подкласс А (Ограждение, n={analysis_a['total_crashes']}, итог: {analysis_a['assigned_class']})",
        color='#2980b9', edgecolor='black', linewidth=0.8,
    )
    r_b = ax.bar(
        x + width / 2, vals_b_cum, width,
        yerr=yerr_b, capsize=4.5, error_kw=dict(ecolor='#4a235a', elinewidth=1.3, capthick=1.2, zorder=6),
        label=f"Строка HARA 1.1.B — Подкласс B (Кювет, n={analysis_b['total_crashes']}, итог: {analysis_b['assigned_class']})",
        color='#8e44ad', edgecolor='black', linewidth=0.8,
    )

    ax.axhspan(0, 10.0, color='gray', alpha=0.15, label='Зона отсечки нерепрезентативных исходов ≤ 10%')
    ax.axhline(
        10.0, color='#e74c3c', linestyle='--', linewidth=2.0, zorder=5,
        label='Порог репрезентативности 10.0% (точечная оценка, ISO 26262-3 / SAE J2980)',
    )
    ax.errorbar(
        [], [], yerr=1.0, fmt='none', ecolor='#2c3e50', elinewidth=1.3, capsize=4.5,
        label='Усы: 95% ДИ Клоппера–Пирсона (информац. неопределённость выборки, SAE J2980 §B.1)',
    )

    for rects, cis, color_txt in [(r_a, cis_a, '#1b4f72'), (r_b, cis_b, '#4a235a')]:
        for idx_s, (rect, (lo, hi)) in enumerate(zip(rects, cis)):
            h = rect.get_height()
            top_y = max(h, hi * 100.0)
            if idx_s in (0, 1):  # S3 и >=S2: явная запись точечной оценки и 95% ДИ
                txt = f"{format_iso_pct(h)}\n[{lo*100:.1f}–{hi*100:.1f}%]"
            else:
                txt = format_iso_pct(h)
            ax.annotate(
                txt,
                xy=(rect.get_x() + rect.get_width() / 2, top_y),
                xytext=(0, 5), textcoords="offset points",
                ha='center', va='bottom', fontsize=8.6, fontweight='bold', color=color_txt,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.86),
                zorder=8,
            )

    ax.set_ylabel('Накопленная вероятность при ДТП (%)', fontsize=12, fontweight='bold')
    ax.set_title(
        f'Определение класса тяжести по правилу >10% — раздельные строки HARA 1.1.A и 1.1.B\n'
        f'(Главный консервативный итог Worst-Case = max(S_A, S_B) = {s_worst}; SAE J2980:2023 §5.2.1.5(g), §5.2.2.2 Note 1)',
        fontsize=11.8, fontweight='bold', pad=14,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(steps, fontsize=10, fontweight='bold')
    ax.set_ylim(0, 122)
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.legend(loc='upper left', frameon=True, fontsize=8.8)
    plt.tight_layout()
    plot2_path = os.path.join(plots_dir, "02_cumulative_iso_decision.png")
    safe_savefig(plot2_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot2_path


def plot_ditch_phase_portrait(records: list[dict], plots_dir: str) -> str:
    """График 3: 03_ditch_phase_portrait.png — двухпанельный фазовый портрет граничных условий Y = 9.75 м."""
    fig, (ax_trip, ax_perp) = plt.subplots(1, 2, figsize=(16.0, 6.8), dpi=300)
    ditch_points = [r for r in records if is_valid_ditch_departure(r)]
    n_ditch_b = sum(1 for pt in ditch_points if pt['subclass'] == SUBCLASS_B)
    n_ditch_a_cross = sum(1 for pt in ditch_points if pt['subclass'] == SUBCLASS_A)

    # --- ПАНЕЛЬ А: Механизм 1 (Soil-Trip Rollover: чистый Cooperrider без полки 15°, V_dep vs beta) ---
    max_v_dep_a = max([pt['departure_speed_mps'] or 0.0 for pt in ditch_points] + [30.0])
    x_max_a = max(32.0, max_v_dep_a * 1.05)
    max_beta = max([pt['sideslip_angle_deg'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_a = max(42.0, max_beta * 1.12)

    xs_grid = np.linspace(0.0, x_max_a, 500)

    def _s1_lower_beta(v_arr):
        beta_s1_lat = np.degrees(
            np.arcsin(np.minimum(1.0, DITCH_S1_SKID_VLAT_MPS / np.maximum(v_arr, DITCH_S1_SKID_VLAT_MPS)))
        )
        return np.where(
            v_arr <= DITCH_VDEP_TRACKING_MIN_MPS,
            DITCH_NON_TRACKING_BETA_DEG,
            np.minimum(DITCH_NON_TRACKING_BETA_DEG, beta_s1_lat),
        )

    def _s2_lower_beta(v_arr):
        beta_trip = np.degrees(
            np.arcsin(np.minimum(1.0, DITCH_S2_TRIP_VLAT_MPS / np.maximum(v_arr, DITCH_S2_TRIP_VLAT_MPS)))
        )
        return np.where(
            v_arr <= DITCH_S2_SKID_VDEP_MPS, y_max_a,
            np.where(
                v_arr <= DITCH_S2_TRIP_VDEP_MPS, DITCH_S2_SKID_BETA_DEG,
                np.minimum(DITCH_S2_SKID_BETA_DEG, beta_trip),
            ),
        )

    ys_s1_lo = _s1_lower_beta(xs_grid)
    ys_s2_lo = _s2_lower_beta(xs_grid)
    ys_s3_lo = np.where(xs_grid > DITCH_S3_SKID_VDEP_MPS, DITCH_S3_SKID_BETA_DEG, y_max_a)

    ax_trip.fill_between(
        xs_grid, 0.0, ys_s1_lo, color='#2ecc71', alpha=0.09,
        label=f'S0/S1 (без срыва: β < {DITCH_NON_TRACKING_BETA_DEG:.0f}° и V_lat ≤ {DITCH_S1_SKID_VLAT_MPS:.1f} м/с)',
    )
    s1_mask = (ys_s2_lo > ys_s1_lo)
    ax_trip.fill_between(
        xs_grid, ys_s1_lo, np.minimum(ys_s2_lo, y_max_a), where=s1_mask,
        color='#f1c40f', alpha=0.18,
        label=f'S1: Занос / скольжение без опрокидывания (β ≥ {DITCH_NON_TRACKING_BETA_DEG:.0f}° или V_lat > {DITCH_S1_SKID_VLAT_MPS:.1f})',
    )
    s2_mask = (xs_grid > DITCH_S2_SKID_VDEP_MPS) & (ys_s3_lo > ys_s2_lo)
    ax_trip.fill_between(
        xs_grid, ys_s2_lo, ys_s3_lo, where=s2_mask,
        color='#e67e22', alpha=0.22,
        label=(
            f'S2: Cooperrider (V_dep>{DITCH_S2_TRIP_VDEP_MPS:.0f}, V_lat>{DITCH_S2_TRIP_VLAT_MPS:.1f}) '
            f'и срыв β≥{DITCH_S2_SKID_BETA_DEG:.0f}° (V_dep>{DITCH_S2_SKID_VDEP_MPS:.0f})'
        ),
    )
    ax_trip.fill_between(
        xs_grid, DITCH_S3_SKID_BETA_DEG, y_max_a, where=(xs_grid > DITCH_S3_SKID_VDEP_MPS),
        color='#e74c3c', alpha=0.22,
        label=(
            f'S3: Тяжёлый срыв (V_dep>{DITCH_S3_SKID_VDEP_MPS:.0f}, β≥{DITCH_S3_SKID_BETA_DEG:.0f}° '
            f'или β≥{DITCH_S3_SPIN_BETA_DEG:.0f}° & ω_z>{DITCH_S3_SPIN_OMEGA_Z_DEG_S:.0f}°/с)'
        ),
    )

    ax_trip.plot(xs_grid, ys_s1_lo, color='#b7950b', linestyle=':', linewidth=1.2)
    ax_trip.hlines(DITCH_S2_SKID_BETA_DEG, DITCH_S2_SKID_VDEP_MPS, x_max_a, color='#d35400', linestyle='--', linewidth=1.1, alpha=0.75)
    ax_trip.hlines(DITCH_S3_SKID_BETA_DEG, DITCH_S3_SKID_VDEP_MPS, x_max_a, color='#c0392b', linestyle='-', linewidth=1.5)
    ax_trip.vlines(DITCH_S2_SKID_VDEP_MPS, DITCH_S2_SKID_BETA_DEG, y_max_a, color='#d35400', linestyle='--', linewidth=1.1)
    ax_trip.vlines(DITCH_S2_TRIP_VDEP_MPS, float(_s2_lower_beta(np.array([DITCH_S2_TRIP_VDEP_MPS]))[0]), y_max_a, color='#d35400', linestyle=':', linewidth=1.0, alpha=0.7)
    ax_trip.vlines(DITCH_S3_SKID_VDEP_MPS, DITCH_S3_SKID_BETA_DEG, y_max_a, color='#c0392b', linestyle='--', linewidth=1.2)

    xs_coop = np.linspace(DITCH_S2_TRIP_VDEP_MPS, x_max_a, 240)
    ys_coop = np.degrees(np.arcsin(np.minimum(1.0, DITCH_S2_TRIP_VLAT_MPS / xs_coop)))
    ax_trip.plot(
        xs_coop, ys_coop, color='#a04000', linestyle='-', linewidth=2.1, zorder=6,
        label=f'Кривая Cooperrider: β = arcsin({DITCH_S2_TRIP_VLAT_MPS:.1f} / V_dep) (без полки 15°)',
    )

    beta_at_100 = math.degrees(math.asin(DITCH_S2_TRIP_VLAT_MPS / (100.0 / 3.6)))
    ax_trip.text(0.6, DITCH_NON_TRACKING_BETA_DEG + 0.6, 'β = 15° (S1)', color='#7d6608', fontsize=7.5, ha='left')
    ax_trip.text(x_max_a - 0.5, DITCH_S2_SKID_BETA_DEG + 0.6, 'β = 20° (S2: V_dep > 11 м/с)', color='#a04000', fontsize=7.5, ha='right')
    ax_trip.text(x_max_a - 0.5, DITCH_S3_SKID_BETA_DEG + 0.6, 'β = 25° (S3: V_dep > 18 м/с)', color='#922b21', fontsize=7.5, ha='right', fontweight='bold')
    ax_trip.text(DITCH_S2_SKID_VDEP_MPS + 0.2, 39.0, '11 м/с', color='#a04000', fontsize=7.5, rotation=90, va='bottom')
    ax_trip.text(DITCH_S3_SKID_VDEP_MPS + 0.2, 39.0, '18 м/с', color='#c0392b', fontsize=7.5, rotation=90, va='bottom')
    ax_trip.annotate(
        f'Cooperrider: V_lat = {DITCH_S2_TRIP_VLAT_MPS:.1f} м/с\n({beta_at_100:.1f}° при 100 км/ч)',
        xy=(100.0 / 3.6, beta_at_100),
        xytext=(16.5, 8.2),
        ha='center', va='center', fontsize=7.2, color='#a04000', fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='#a04000', lw=0.9),
    )

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        beta_val = pt['sideslip_angle_deg'] if pt['sideslip_angle_deg'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        is_cross_a = (pt['subclass'] == SUBCLASS_A)
        ax_trip.scatter(
            v_dep, beta_val, color=c,
            edgecolors='#4d5656' if is_cross_a else 'black',
            linewidths=1.8 if is_cross_a else 0.9,
            linestyles='--' if is_cross_a else '-',
            s=100 if is_cross_a else 85,
            alpha=0.88, marker=marker, zorder=10,
        )

    ax_trip.scatter(
        [], [], color='white', marker='o', edgecolors='black', linewidths=0.9, s=75,
        label=f'Целевой съезд SubB (n={n_ditch_b})',
    )
    ax_trip.scatter(
        [], [], color='white', marker='o', edgecolors='#4d5656', linewidths=1.8, linestyles='--', s=85,
        label=f'Перекрёстный съезд SubA (пунктир рамки, n={n_ditch_a_cross})',
    )

    annotate_ditch_points(ax_trip, ditch_points, 'departure_speed_mps', 'sideslip_angle_deg', x_max_a, y_max_a, is_panel_a=True)

    ax_trip.set_xlabel('Общая скорость в начале кювета V_dep [м/с]', fontsize=11, fontweight='bold')
    ax_trip.set_ylabel('Угол бокового заноса β [град]', fontsize=11, fontweight='bold')
    ax_trip.set_title('А. Риск опрокидывания при боковом заносе (V_dep, β)', fontsize=12, fontweight='bold')
    ax_trip.set_xlim(0, x_max_a)
    ax_trip.set_ylim(0, y_max_a)
    ax_trip.grid(True, linestyle=':', alpha=0.5)
    ax_trip.legend(loc='upper right', frameon=True, fontsize=7.1)

    # --- ПАНЕЛЬ Б: Механизм 2 (Nose-In / Bottom Impact: пороги V_perp = 2 / 12 / 18 м/с) ---
    x_max_b = x_max_a
    max_v_perp = max([pt['road_perp_speed_mps'] or 0.0 for pt in ditch_points] + [0.0])
    y_max_b = max(DITCH_S3_NOSE_VPERP_MPS + 4.5, max_v_perp * 1.25)

    ax_perp.fill_between(
        [DITCH_VDEP_TRACKING_MIN_MPS, x_max_b], DITCH_S3_NOSE_VPERP_MPS, y_max_b, color='#e74c3c', alpha=0.20,
        label=f'S3: Крутой влёт в дно (Severe Nose-In, V⊥ > {DITCH_S3_NOSE_VPERP_MPS:.0f} м/с)',
    )
    ax_perp.fill_between(
        [DITCH_VDEP_TRACKING_MIN_MPS, x_max_b], DITCH_S2_BOTTOM_VPERP_MPS, DITCH_S3_NOSE_VPERP_MPS, color='#e67e22', alpha=0.20,
        label=f'S2: Выход за предел AASHTO RDG (Bottom Impact, {DITCH_S2_BOTTOM_VPERP_MPS:.0f} < V⊥ ≤ {DITCH_S3_NOSE_VPERP_MPS:.0f} м/с)',
    )
    ax_perp.fill_between(
        [DITCH_VDEP_TRACKING_MIN_MPS, x_max_b], DITCH_S1_TRACKING_VPERP_MPS, DITCH_S2_BOTTOM_VPERP_MPS, color='#f1c40f', alpha=0.18,
        label=f'S1: Проходимый съезд (Traversable, {DITCH_S1_TRACKING_VPERP_MPS:.0f} < V⊥ ≤ {DITCH_S2_BOTTOM_VPERP_MPS:.0f} м/с; RDG ≤ 11.34)',
    )
    ax_perp.fill_between([0.0, DITCH_VDEP_TRACKING_MIN_MPS], DITCH_S1_TRACKING_VPERP_MPS, y_max_b, color='#2ecc71', alpha=0.10)
    ax_perp.fill_between(
        [0.0, x_max_b], 0.0, DITCH_S1_TRACKING_VPERP_MPS, color='#2ecc71', alpha=0.10,
        label=f'S0: Касательный / низкоскоростной съезд (V⊥ ≤ {DITCH_S1_TRACKING_VPERP_MPS:.0f} м/с или V_dep ≤ {DITCH_VDEP_TRACKING_MIN_MPS:.0f} м/с)',
    )

    ax_perp.axhline(DITCH_S3_NOSE_VPERP_MPS, color='#c0392b', linestyle='--', linewidth=1.3)
    ax_perp.axhline(DITCH_S2_BOTTOM_VPERP_MPS, color='#d35400', linestyle='--', linewidth=1.3)
    ax_perp.axhline(DITCH_S1_TRACKING_VPERP_MPS, color='#b7950b', linestyle='--', linewidth=1.2)
    ax_perp.axvline(DITCH_VDEP_TRACKING_MIN_MPS, color='#27ae60', linestyle=':', linewidth=1.2)

    ax_perp.text(x_max_b - 0.5, DITCH_S3_NOSE_VPERP_MPS + 0.35, f'Порог S3: V⊥ = {DITCH_S3_NOSE_VPERP_MPS:.1f} м/с', color='#c0392b', fontsize=8, ha='right', fontweight='bold')
    ax_perp.text(x_max_b - 0.5, DITCH_S2_BOTTOM_VPERP_MPS + 0.35, f'Порог S2: V⊥ = {DITCH_S2_BOTTOM_VPERP_MPS:.1f} м/с', color='#d35400', fontsize=8, ha='right', fontweight='bold')
    ax_perp.text(0.6, DITCH_S1_TRACKING_VPERP_MPS + 0.30, f'Порог S1: V⊥ = {DITCH_S1_TRACKING_VPERP_MPS:.1f} м/с', color='#b7950b', fontsize=8, ha='left')
    ax_perp.text(DITCH_VDEP_TRACKING_MIN_MPS + 0.2, 0.4, f'V_dep = {DITCH_VDEP_TRACKING_MIN_MPS:.1f} м/с', color='#27ae60', fontsize=8, rotation=90)
    ax_perp.text(4.0, 9.0, f'S0: V_dep ≤ {DITCH_VDEP_TRACKING_MIN_MPS:.0f} м/с', color='#1e8449', fontsize=8, ha='center', va='center', fontweight='bold')

    for pt in ditch_points:
        v_dep = pt['departure_speed_mps']
        v_perp = pt['road_perp_speed_mps'] if pt['road_perp_speed_mps'] is not None else 0.0
        sev = pt['sev_ditch'] if pt['sev_ditch'] in SEVERITY_CLASSES else pt['severity']
        c = SEVERITY_COLORS.get(sev, '#34495e')
        marker = 'o' if pt['ditch_departure_actor'] == 'ego' else ('s' if pt['ditch_departure_actor'] == 'target' else '^')
        is_cross_a = (pt['subclass'] == SUBCLASS_A)
        ax_perp.scatter(
            v_dep, v_perp, color=c,
            edgecolors='#4d5656' if is_cross_a else 'black',
            linewidths=1.8 if is_cross_a else 0.9,
            linestyles='--' if is_cross_a else '-',
            s=100 if is_cross_a else 85,
            alpha=0.88, marker=marker, zorder=10,
        )

    ax_perp.scatter([], [], color='gray', marker='o', edgecolors='black', label='Съезд Ego')
    ax_perp.scatter([], [], color='gray', marker='s', edgecolors='black', label='Съезд Target')
    ax_perp.scatter([], [], color='gray', marker='^', edgecolors='black', label='Съезд обоих ТС (Both)')
    ax_perp.scatter(
        [], [], color='white', marker='o', edgecolors='black', linewidths=0.9,
        label=f'Целевой съезд SubB (n={n_ditch_b})',
    )
    ax_perp.scatter(
        [], [], color='white', marker='o', edgecolors='#4d5656', linewidths=1.8, linestyles='--',
        label=f'Перекрёстный съезд SubA (пунктир рамки, n={n_ditch_a_cross})',
    )
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
        fontsize=14, fontweight='bold',
    )
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plot3_path = os.path.join(plots_dir, "03_ditch_phase_portrait.png")
    safe_savefig(plot3_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot3_path


def plot_subclass_a_dv_cascade(records: list[dict], plots_dir: str) -> str:
    """
    График 4: 04_subclass_a_dv_cascade.png — каскад скоростей в Подклассе А строго по ДТП.
    Адаптивный режим подписей и тиков оси X для малых (n<=45), средних (n~100) и больших (n>=200) выборок.
    """
    sub_a_crashes = [r for r in records if r['subclass'] == SUBCLASS_A and is_crash_run(r)]
    n_crashes = len(sub_a_crashes)
    fig, (ax_tt, ax_bar) = plt.subplots(2, 1, figsize=(16.0 if n_crashes > 110 else 15.0, 8.8), dpi=300, sharex=True)

    if sub_a_crashes:
        indices = np.arange(n_crashes)
        dv_ego = [r['delta_v_ego_mps'] if r['delta_v_ego_mps'] is not None else 0.0 for r in sub_a_crashes]
        dv_target = [r['delta_v_target_mps'] if r['delta_v_target_mps'] is not None else 0.0 for r in sub_a_crashes]
        v_barrier = [r['barrier_impact_mps'] if r['barrier_impact_mps'] is not None else 0.0 for r in sub_a_crashes]
        dv_max_per_run = [max(e, t) for e, t in zip(dv_ego, dv_target)]

        dense_mode = n_crashes > 45
        very_dense = n_crashes > 110
        edge_lw_tt = 0.25 if very_dense else (0.45 if dense_mode else 0.7)
        edge_lw_bar = 0.28 if very_dense else (0.50 if dense_mode else 0.8)

        # --- ВЕРХНЯЯ ПАНЕЛЬ: Первичный удар ТС-ТС ---
        w_tt = 0.42 if very_dense else 0.35
        ax_tt.bar(
            indices - w_tt / 2, dv_ego, w_tt,
            label='Delta-V Ego (Chevrolet Impala, 3490 кг)',
            color='#5dade2', edgecolor='#1b4f72' if very_dense else 'black', linewidth=edge_lw_tt,
        )
        ax_tt.bar(
            indices + w_tt / 2, dv_target, w_tt,
            label='Delta-V Target (Seat Leon, var. mass)',
            color='#2471a3', edgecolor='#154360' if very_dense else 'black', linewidth=edge_lw_tt,
        )

        ax_tt.axhline(8.0, color='#c0392b', linestyle='--', linewidth=1.2, label='Порог S3 бокового удара (8.0 м/с)')
        ax_tt.axhline(3.5, color='#d35400', linestyle='--', linewidth=1.2, label='Порог S2 бокового удара (3.5 м/с)')
        ax_tt.axhline(1.0, color='#f39c12', linestyle='--', linewidth=1.2, label='Порог S1 бокового удара (1.0 м/с)')

        max_dv = max(dv_ego + dv_target + [8.0])
        ax_tt.set_ylim(0, max(10.5, max_dv * 1.32))

        if not dense_mode:
            for idx, r in enumerate(sub_a_crashes):
                top_val = dv_max_per_run[idx]
                s_primary = r['sev_side']
                if top_val > 0:
                    ax_tt.annotate(
                        f"{top_val:.1f} ({s_primary})",
                        xy=(idx, top_val), xytext=(0, 4), textcoords="offset points",
                        ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#1b4f72', rotation=90,
                    )
        else:
            # Адаптивная разметка: выделяем все S2/S3 и глобальный максимум Delta-V без наложения
            max_dv_idx = int(np.argmax(dv_max_per_run))
            notable_tt = [
                i for i, r in enumerate(sub_a_crashes)
                if r.get('sev_side') in ('S2', 'S3') or i == max_dv_idx
            ]
            last_x_tt = -999
            min_gap_tt = max(4, n_crashes // 28)
            for idx in notable_tt:
                r = sub_a_crashes[idx]
                top_val = dv_max_per_run[idx]
                if top_val <= 0:
                    continue
                y_off = 22 if (idx - last_x_tt) < min_gap_tt else 12
                last_x_tt = idx
                ax_tt.annotate(
                    f"#{r['run_id']}: {top_val:.2f} м/с ({r['sev_side']})",
                    xy=(idx, top_val), xytext=(0, y_off), textcoords="offset points",
                    ha='center', va='bottom', fontsize=8.0, fontweight='bold', color='#1b4f72',
                    bbox=dict(boxstyle='round,pad=0.18', facecolor='white', edgecolor='#2471a3', alpha=0.88, lw=0.7),
                    arrowprops=dict(arrowstyle='->', color='#2471a3', lw=0.8),
                )
            nz_dv = [v for v in dv_max_per_run if v > 0]
            if nz_dv:
                s_cnt_tt = {s: sum(1 for r in sub_a_crashes if r.get('sev_side') == s) for s in SEVERITY_CLASSES}
                badge_tt = (
                    f"Сводка ΔV (n={n_crashes}): мед. {np.median(nz_dv):.2f}, P90 {np.percentile(nz_dv, 90):.2f}, "
                    f"макс. {max(nz_dv):.2f} м/с (#{sub_a_crashes[max_dv_idx]['run_id']})\n"
                    f"Классы S_side: S0={s_cnt_tt['S0']}, S1={s_cnt_tt['S1']}, S2={s_cnt_tt['S2']}, S3={s_cnt_tt['S3']}"
                )
                ax_tt.text(
                    0.012, 0.94, badge_tt, transform=ax_tt.transAxes,
                    ha='left', va='top', fontsize=8.3, color='#1b4f72',
                    bbox=dict(boxstyle='round,pad=0.35', facecolor='#f8f9f9', edgecolor='#85c1e9', alpha=0.92),
                )

        ax_tt.set_ylabel('Delta-V ТС-ТС [м/с]', fontsize=11, fontweight='bold')
        ax_tt.set_title(
            f'А. Первичный боковой удар между автомобилями на полосе (n = {n_crashes})',
            fontsize=12, fontweight='bold',
        )
        ax_tt.grid(axis='y', linestyle=':', alpha=0.6)
        ax_tt.legend(loc='upper right', frameon=True, fontsize=8.5)

        # --- НИЖНЯЯ ПАНЕЛЬ: Вторичный удар об ограждение ---
        bar_colors = [SEVERITY_COLORS.get(r['sev_barrier'], '#2ecc71') for r in sub_a_crashes]
        is_ditch_s2_flags = [(r.get('severity') in ('S2', 'S3') and r.get('sev_barrier') not in ('S2', 'S3')) for r in sub_a_crashes]
        bar_edges = ['#d35400' if flag else ('#566573' if very_dense else 'black') for flag in is_ditch_s2_flags]
        bar_lws = [1.8 if flag else edge_lw_bar for flag in is_ditch_s2_flags]
        w_b = 0.72 if very_dense else 0.55
        ax_bar.bar(
            indices, v_barrier, w_b, color=bar_colors, edgecolor=bar_edges, linewidth=bar_lws,
            label='Скорость удара V_barrier (цвет по классу sev_barrier)',
        )

        l_fr_s3 = ax_bar.axhline(18.0, color='#922b21', linestyle=':', linewidth=1.4, label='FR Порог S3 (18.0 м/с, > 45°)')
        l_ss_s3 = ax_bar.axhline(12.0, color='#c0392b', linestyle='--', linewidth=1.4, label='SS Порог S3 (12.0 м/с, ≤ 45°)')
        l_fr_s2 = ax_bar.axhline(11.0, color='#b9770e', linestyle=':', linewidth=1.4, label='FR Порог S2 (11.0 м/с, > 45°)')
        l_ss_s2 = ax_bar.axhline(6.0, color='#d35400', linestyle='--', linewidth=1.4, label='SS Порог S2 (6.0 м/с, ≤ 45°)')

        short_actor_map = {'target': 'T', 'ego': 'E', 'both': 'B', 'none': 'None'}
        has_ditch_s2_override = any(is_ditch_s2_flags)

        if not dense_mode:
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
                        txt += f" [{r['severity']}: кювет]"
                    ax_bar.annotate(
                        txt, xy=(idx, v_b), xytext=(0, 5), textcoords="offset points",
                        ha='center', va='bottom', fontsize=7.5, fontweight='bold',
                        color='#d35400' if is_ditch_override else '#2c3e50', rotation=90,
                    )
                elif is_ditch_override:
                    ax_bar.annotate(
                        f"0.0 м/с [{r['severity']}: кювет]", xy=(idx, 0.0), xytext=(0, 6), textcoords="offset points",
                        ha='center', va='bottom', fontsize=7.5, fontweight='bold',
                        color='#d35400', rotation=90,
                    )
        else:
            # Адаптивная разметка: подписываем все S2/S3, перекрёстные кювет-оверрайды и Top-3 пика V_barrier
            valid_bar_indices = [i for i, r in enumerate(sub_a_crashes) if is_valid_barrier_hit(r) and v_barrier[i] > 0.01]
            top_peak_indices = sorted(valid_bar_indices, key=lambda i: v_barrier[i], reverse=True)[:3]
            notable_set = set(top_peak_indices)
            for i, r in enumerate(sub_a_crashes):
                if r.get('sev_barrier') in ('S2', 'S3') or is_ditch_s2_flags[i]:
                    notable_set.add(i)
            notable_bar = sorted(notable_set)

            # Разведение близких выносок по вертикали и горизонтали
            min_gap_b = max(16, n_crashes // 7)
            placed_bar = []
            for idx in notable_bar:
                r = sub_a_crashes[idx]
                v_b = v_barrier[idx]
                is_ditch_override = is_ditch_s2_flags[idx]
                if is_valid_barrier_hit(r) and v_b > 0.01:
                    act = str(r['barrier_collision_actor']).lower()
                    short_actor = short_actor_map.get(act, r['barrier_collision_actor'])
                    itype = 'SS' if r['barrier_impact_type'] == 'Side-swipe' else 'FR'
                    txt = f"#{r['run_id']}: {v_b:.2f} м/с ({r['sev_barrier']}, {itype}-{short_actor})"
                    if is_ditch_override:
                        txt += f" [{r['severity']}: кювет]"
                else:
                    txt = f"#{r['run_id']}: 0.0 м/с [{r['severity']}: кювет]"

                level = 0
                for prev_idx, prev_lvl in placed_bar:
                    if abs(idx - prev_idx) < min_gap_b and prev_lvl == level:
                        level += 1
                placed_bar.append((idx, level))
                y_off = 14 + level * 22
                x_off = 0
                if idx < n_crashes * 0.08:
                    ha = 'left'
                    x_off = 4
                elif idx > n_crashes * 0.92:
                    ha = 'right'
                    x_off = -4
                else:
                    ha = 'center'

                is_high = r.get('sev_barrier') in ('S2', 'S3') or is_ditch_override
                ax_bar.annotate(
                    txt,
                    xy=(idx, v_b), xytext=(x_off, y_off), textcoords="offset points",
                    ha=ha, va='bottom', fontsize=7.8, fontweight='bold',
                    color='#c0392b' if is_high else '#2c3e50',
                    bbox=dict(
                        boxstyle='round,pad=0.18', facecolor='white',
                        edgecolor='#d35400' if is_high else '#7f8c8d', alpha=0.90, lw=0.8,
                    ),
                    arrowprops=dict(arrowstyle='->', color='#d35400' if is_high else '#566573', lw=0.8),
                )

            if valid_bar_indices:
                vb_vals = [v_barrier[i] for i in valid_bar_indices]
                max_vb_idx = max(valid_bar_indices, key=lambda i: v_barrier[i])
                s_cnt_b = {s: sum(1 for r in sub_a_crashes if r.get('sev_barrier') == s) for s in SEVERITY_CLASSES}
                badge_bar = (
                    f"Удары о барьер: {len(valid_bar_indices)}/{n_crashes} | мед. {np.median(vb_vals):.2f}, "
                    f"P90 {np.percentile(vb_vals, 90):.2f}, макс. {v_barrier[max_vb_idx]:.2f} м/с (#{sub_a_crashes[max_vb_idx]['run_id']})\n"
                    f"Классы S_barrier: S0={s_cnt_b['S0']}, S1={s_cnt_b['S1']}, S2={s_cnt_b['S2']}, S3={s_cnt_b['S3']}"
                )
                ax_bar.text(
                    0.012, 0.94, badge_bar, transform=ax_bar.transAxes,
                    ha='left', va='top', fontsize=8.3, color='#2c3e50',
                    bbox=dict(boxstyle='round,pad=0.35', facecolor='#f8f9f9', edgecolor='#aeb6bf', alpha=0.92),
                )

        max_v_b = max(v_barrier + [18.0])
        ax_bar.set_ylim(0, max(24.0, max_v_b * 1.45))

        ax_bar.set_xlabel(f'Номер прогона с ДТП в Подклассе А (n = {n_crashes})', fontsize=11, fontweight='bold')
        ax_bar.set_ylabel('Скорость удара V_barrier [м/с]', fontsize=11, fontweight='bold')
        ax_bar.set_title(
            f'Б. Вторичный удар о жёсткое ограждение с учетом угла удара (n = {n_crashes})',
            fontsize=12, fontweight='bold',
        )

        # Адаптивные метки оси X без наслоения
        if not dense_mode:
            tick_positions = indices
            tick_labels = [f"#{r['run_id']}" for r in sub_a_crashes]
            tick_fs = 7.5
            tick_rot = 90
        else:
            step = 5 if not very_dense else max(5, int(math.ceil(n_crashes / 25.0 / 5.0)) * 5)
            tick_positions = list(range(0, n_crashes, step))
            if (n_crashes - 1) - tick_positions[-1] >= max(2, step // 2):
                tick_positions.append(n_crashes - 1)
            tick_labels = [f"#{sub_a_crashes[i]['run_id']}" for i in tick_positions]
            tick_fs = 8.2
            tick_rot = 45

        ax_bar.set_xticks(tick_positions)
        ax_bar.set_xticklabels(tick_labels, fontsize=tick_fs, fontweight='bold', rotation=tick_rot, ha='right' if tick_rot == 45 else 'center')
        ax_bar.grid(axis='y', linestyle=':', alpha=0.6)

        p_s0 = mpatches.Patch(facecolor=SEVERITY_COLORS['S0'], edgecolor='black', label='Удар S0')
        p_s1 = mpatches.Patch(facecolor=SEVERITY_COLORS['S1'], edgecolor='black', label='Удар S1')
        p_s2 = mpatches.Patch(facecolor=SEVERITY_COLORS['S2'], edgecolor='black', label='Удар S2')
        p_s3 = mpatches.Patch(facecolor=SEVERITY_COLORS['S3'], edgecolor='black', label='Удар S3')
        p_ditch_s2 = mpatches.Patch(
            facecolor='white', edgecolor='#d35400', linewidth=2.0,
            label='S2/S3: кювет (перекрёстный уход при sev_barrier<S2)',
        )
        p_actors = mpatches.Patch(facecolor='none', edgecolor='none', label='Участники: T (Target, var. mass), E (Ego 3490 кг), B (Both)')
        has_s0_bar = any(r.get('sev_barrier') == 'S0' for r in sub_a_crashes)
        handles = (
            ([p_s0] if has_s0_bar else [])
            + [p_s1, p_s2, p_s3]
            + ([p_ditch_s2] if has_ditch_s2_override else [])
            + [l_fr_s3, l_ss_s3, l_fr_s2, l_ss_s2, p_actors]
        )
        ax_bar.legend(handles=handles, loc='upper right', frameon=True, fontsize=8, ncol=2)
    else:
        ax_tt.text(0.5, 0.5, 'Нет прогонов с ДТП для Подкласса А', ha='center', va='center')
        ax_bar.text(0.5, 0.5, 'Нет прогонов с ДТП для Подкласса А', ha='center', va='center')

    fig.suptitle(
        f'Сравнение скоростей первичного и вторичного ударов (Подкласс А, n = {n_crashes})',
        fontsize=14, fontweight='bold',
    )
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plot4_path = os.path.join(plots_dir, "04_subclass_a_dv_cascade.png")
    safe_savefig(plot4_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot4_path


def plot_kinematics_vs_severity(records: list[dict], analysis_odd: dict, plots_dir: str) -> str:
    """График 5: 05_kinematics_vs_severity_boxplots.png — эскалация тяжести и воронка конфликта."""
    n_total_all = len(records)
    fig, (ax_esc, ax_asym) = plt.subplots(1, 2, figsize=(16.5, 6.8), dpi=300)

    # --- ПАНЕЛЬ А: Эскалация тяжести при ДТП (Взвешенный ODD: 0.20·A + 0.80·B) ---
    crash_records = [r for r in records if is_crash_run(r)]
    n_crash = len(crash_records)
    crash_a = [r for r in crash_records if r['subclass'] == SUBCLASS_A]
    crash_b = [r for r in crash_records if r['subclass'] == SUBCLASS_B]
    n_ca = max(1, len(crash_a))
    n_cb = max(1, len(crash_b))
    weights_odd = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights_odd.get('A', 0.20)
    w_b = weights_odd.get('B', 0.80)

    stage1_counts = {s: sum(1 for r in crash_records if r['sev_side'] == s) for s in SEVERITY_CLASSES}
    stage2_counts = {s: sum(1 for r in crash_records if r['severity'] == s) for s in SEVERITY_CLASSES}

    stage1_pcts = [
        w_a * (sum(1 for r in crash_a if r['sev_side'] == s) / n_ca * 100.0)
        + w_b * (sum(1 for r in crash_b if r['sev_side'] == s) / n_cb * 100.0)
        for s in SEVERITY_CLASSES
    ]
    stage2_pcts = [analysis_odd['percentages'].get(s, 0.0) for s in SEVERITY_CLASSES]

    x_esc = np.arange(len(SEVERITY_CLASSES))
    w_esc = 0.35

    r_s1 = ax_esc.bar(
        x_esc - w_esc / 2, stage1_pcts, w_esc,
        label=f'Этап 1: Первичный боковой контакт ({w_a:.2f}·A + {w_b:.2f}·B, справ.)',
        color='#3498db', edgecolor='black', linewidth=0.8,
    )
    r_s2 = ax_esc.bar(
        x_esc + w_esc / 2, stage2_pcts, w_esc,
        label=f'Этап 2: Итог после ограждения / кювета ({w_a:.2f}·A + {w_b:.2f}·B, справ.)',
        color='#e74c3c', edgecolor='black', linewidth=0.8,
    )

    for rect, s in zip(r_s1, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage1_counts[s]
        ax_esc.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=8.8, fontweight='bold', color='#1b4f72',
        )

    for rect, s in zip(r_s2, SEVERITY_CLASSES):
        h = rect.get_height()
        cnt = stage2_counts[s]
        ax_esc.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=8.8, fontweight='bold', color='#922b21',
        )

    max_esc = max(stage1_pcts + stage2_pcts + [10.0])
    ax_esc.set_ylim(0, max_esc * 1.28)
    ax_esc.set_ylabel(f'Справочная взвешенная доля ODD ({w_a:.2f}·A + {w_b:.2f}·B), %', fontsize=10.2, fontweight='bold')
    ax_esc.set_title(
        f'А. Эскалация тяжести: первичный контакт vs итог ДТП\n(Справочный профиль ODD: {w_a:.2f}·A + {w_b:.2f}·B, n = {n_crash}; вердикты HARA — 1.1.A / 1.1.B)',
        fontsize=10.4, fontweight='bold', pad=10,
    )
    ax_esc.set_xticks(x_esc)
    ax_esc.set_xticklabels(['S0 (Без травм)', 'S1 (Легкие)', 'S2 (Тяжелые)', 'S3 (Фатальные)'], fontsize=9.5, fontweight='bold')
    ax_esc.grid(axis='y', linestyle=':', alpha=0.6)
    ax_esc.legend(loc='upper right', frameon=True, fontsize=8.2)

    # --- ПАНЕЛЬ Б: Воронка реализации конфликта в слепой зоне и распределение пострадавших ---
    actors_cat = [
        'Оба ТС\n(Both)',
        'Только Target\n(Seat Leon)',
        'Только Ego\n(Impala 3490 кг)',
        'Только контакт\nТС–ТС (без\nбарьера/кювета)',
        'Разъехались\nбез удара',
    ]

    sub_a_runs = [r for r in records if r['subclass'] == SUBCLASS_A]
    sub_b_runs = [r for r in records if r['subclass'] == SUBCLASS_B]
    tot_a = max(1, len(sub_a_runs))
    tot_b = max(1, len(sub_b_runs))

    def _norm_actor(val):
        return str(val or '').strip().lower()

    counts_a_act = {
        'both': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'both'),
        'target': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'target'),
        'ego': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) == 'ego'),
        'car_only': sum(1 for r in sub_a_runs if is_crash_run(r) and _norm_actor(r.get('barrier_collision_actor')) not in ('both', 'target', 'ego')),
        'safe': sum(1 for r in sub_a_runs if not is_crash_run(r) and not is_spawn_error_run(r)),
    }

    counts_b_act = {
        'both': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'both'),
        'target': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'target'),
        'ego': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) == 'ego'),
        'car_only': sum(1 for r in sub_b_runs if is_crash_run(r) and _norm_actor(r.get('ditch_departure_actor')) not in ('both', 'target', 'ego')),
        'safe': sum(1 for r in sub_b_runs if not is_crash_run(r) and not is_spawn_error_run(r)),
    }

    order_keys = ['both', 'target', 'ego', 'car_only', 'safe']
    pcts_a_act = [(counts_a_act[k] / tot_a) * 100.0 for k in order_keys]
    pcts_b_act = [(counts_b_act[k] / tot_b) * 100.0 for k in order_keys]

    x_act = np.arange(len(actors_cat))
    w_act = 0.36

    r_a_act = ax_asym.bar(
        x_act - w_act / 2, pcts_a_act, w_act,
        label=f'Подкласс А (Ограждение, N={len(sub_a_runs)})',
        color='#2980b9', edgecolor='black', linewidth=0.8,
    )
    r_b_act = ax_asym.bar(
        x_act + w_act / 2, pcts_b_act, w_act,
        label=f'Подкласс B (Кювет, N={len(sub_b_runs)})',
        color='#8e44ad', edgecolor='black', linewidth=0.8,
    )

    for rect, k in zip(r_a_act, order_keys):
        h = rect.get_height()
        cnt = counts_a_act[k]
        ax_asym.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=8.2, fontweight='bold', color='#1b4f72',
        )

    for rect, k in zip(r_b_act, order_keys):
        h = rect.get_height()
        cnt = counts_b_act[k]
        ax_asym.annotate(
            f"{h:.1f}%\n(n={cnt})",
            xy=(rect.get_x() + rect.get_width() / 2, h),
            xytext=(0, 4), textcoords="offset points",
            ha='center', va='bottom', fontsize=8.2, fontweight='bold', color='#512e5f',
        )

    max_asym = max(pcts_a_act + pcts_b_act + [10.0])
    ax_asym.set_ylim(0, max_asym * 1.25)
    ax_asym.set_ylabel('Доля от всех заездов (%)', fontsize=10.5, fontweight='bold')
    ax_asym.set_title(
        f'Б. Итоги всех заездов и участники ДТП (N = {n_total_all}, сумма = 100%)',
        fontsize=11, fontweight='bold', pad=10,
    )
    ax_asym.set_xticks(x_act)
    ax_asym.set_xticklabels(actors_cat, fontsize=8.2, fontweight='bold')
    ax_asym.grid(axis='y', linestyle=':', alpha=0.6)
    ax_asym.legend(loc='upper right', frameon=True, fontsize=8.2)

    fig.suptitle(
        'Рост тяжести травм при вторичных ударах и общая статистика заездов',
        fontsize=14, fontweight='bold', y=0.98,
    )
    plt.tight_layout(rect=[0, 0.02, 1, 0.93])
    plt.subplots_adjust(wspace=0.22)
    plot5_path = os.path.join(plots_dir, "05_kinematics_vs_severity_boxplots.png")
    safe_savefig(plot5_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot5_path


def generate_plots(
    records: list[dict],
    analysis_a: dict,
    analysis_b: dict,
    analysis_odd: dict,
    s_worst: str,
    output_dir: str,
) -> list[str]:
    """
    Построение 5 публикационных инженерных графиков строго по выборке состоявшихся ДТП (N_crash)
    и отображению воронки конфликта (N_total).
    """
    plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    paths = [
        plot_severity_distribution(analysis_a, analysis_b, analysis_odd, plots_dir),
        plot_cumulative_iso_decision(analysis_a, analysis_b, analysis_odd, plots_dir),
        plot_ditch_phase_portrait(records, plots_dir),
        plot_subclass_a_dv_cascade(records, plots_dir),
        plot_kinematics_vs_severity(records, analysis_odd, plots_dir),
    ]

    print(f"  [OK] Все 5 обновленных графиков успешно сохранены в: {plots_dir}")
    return paths
