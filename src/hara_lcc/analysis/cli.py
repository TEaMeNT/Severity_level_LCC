"""
CLI-оркестратор модуля статистической аналитики HARA (`analyze_results.py`).
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import os
import sys

from hara_lcc.analysis.loader import load_simulation_results
from hara_lcc.analysis.plots import generate_plots
from hara_lcc.analysis.report import generate_markdown_report, print_console_summary
from hara_lcc.config import DEFAULT_WEIGHT_A, DEFAULT_WEIGHT_B, SUBCLASS_A, SUBCLASS_B
from hara_lcc.severity.iso26262 import (
    compute_worst_case_verdict,
    evaluate_controllability,
    evaluate_iso_severity,
    evaluate_weighted_odd,
    is_crash_run,
    is_spawn_error_run,
)


def _configure_windows_utf8():
    if sys.platform == 'win32':
        try:
            sys.stdout.reconfigure(encoding='utf-8')
            sys.stderr.reconfigure(encoding='utf-8')
        except Exception:
            pass


def build_arg_parser() -> argparse.ArgumentParser:
    """Построение CLI-парсера analyze_results.py со 100% сохранением флагов."""
    parser = argparse.ArgumentParser(description="Statistical Analysis of HARA Simulation Results (ISO 26262-3 / SAE J2980:2023)")
    parser.add_argument(
        '--input', type=str, default='results',
        help="Path to passive CSV file or directory containing results (default: results)",
    )
    parser.add_argument(
        '--active-input', type=str, default=None,
        help="Optional path to active driver campaign CSV for Controllability (C) analysis",
    )
    parser.add_argument(
        '--output-dir', type=str, default='results',
        help="Directory to save summary report and plots (default: results)",
    )
    parser.add_argument(
        '--weight-a', type=float, default=DEFAULT_WEIGHT_A,
        help="Statistical weight of Subclass A (Barrier) in ODD (default: 0.20)",
    )
    parser.add_argument(
        '--weight-b', type=float, default=DEFAULT_WEIGHT_B,
        help="Statistical weight of Subclass B (Ditch) in ODD (default: 0.80)",
    )
    parser.add_argument(
        '--weight-c', type=float, default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        '--template', type=str, default=None,
        help="Path to Markdown report template (default: templates/report_template.md or report_template.md)",
    )
    return parser


def run_analysis(args: argparse.Namespace) -> dict:
    """Выполнение полного цикла аналитики HARA: загрузка, расчёт, консоль, графики, отчёт."""
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
    print(f" Вердикт HARA: раздельно по 1.1.A / 1.1.B и Worst-Case (справочные веса ODD: A={args.weight_a:.2f}, B={weight_b:.2f})")
    print("=" * 80)

    # 1. Загрузка данных
    records = load_simulation_results(args.input)
    print(f"\n[1/4] Загружено {len(records)} записей прогонов.")

    # 2. Выделение состоявшихся ДТП и безаварийных разъездов (Воронка сценария)
    records_a = [r for r in records if r['subclass'] == SUBCLASS_A]
    records_b = [r for r in records if r['subclass'] == SUBCLASS_B]

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
        active_a = [r for r in active_records if r['subclass'] == SUBCLASS_A]
        active_b = [r for r in active_records if r['subclass'] == SUBCLASS_B]
        ctrl_a = evaluate_controllability(active_a, harm_threshold='S2')
        ctrl_b = evaluate_controllability(active_b, harm_threshold='S2')
        ctrl_all = evaluate_controllability(active_records, harm_threshold='S2')

    # 4. Вывод в консоль
    print_console_summary(
        analysis_a, analysis_b, analysis_odd, s_worst,
        ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all,
    )

    # 5. Построение обновленных графиков
    print("\n[3/4] Построение 5 обновленных инженерных графиков в results/plots/...")
    plot_paths = generate_plots(records, analysis_a, analysis_b, analysis_odd, s_worst, args.output_dir)

    # 6. Генерация аналитического отчета
    print("\n[4/4] Генерация итогового отчета summary_report.md...")
    report_path = generate_markdown_report(
        records, analysis_a, analysis_b, analysis_odd, s_worst, args.output_dir,
        template_path=args.template, ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all,
    )

    print("\n" + "=" * 80)
    print(" АНАЛИЗ УСПЕШНО ЗАВЕРШЕН!")
    print(f" Отчет: {os.path.join(args.output_dir, 'summary_report.md')}")
    print(f" Графики: {os.path.join(args.output_dir, 'plots')}")
    print("=" * 80)

    return {
        'records': records,
        'analysis_a': analysis_a,
        'analysis_b': analysis_b,
        'analysis_odd': analysis_odd,
        's_worst': s_worst,
        'ctrl_a': ctrl_a,
        'ctrl_b': ctrl_b,
        'ctrl_all': ctrl_all,
        'plot_paths': plot_paths,
        'report_path': report_path,
    }


def main(argv: list[str] | None = None):
    _configure_windows_utf8()
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    run_analysis(args)
