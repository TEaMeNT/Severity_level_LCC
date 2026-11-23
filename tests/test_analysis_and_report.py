"""Тесты статистики (Уилсон/Клоппер–Пирсон), ASIL, 1-в-1 рендера отчёта и генерации 5 графиков."""

from __future__ import annotations

from pathlib import Path

import pytest

from hara_lcc.analysis.cli import build_arg_parser, run_analysis
from hara_lcc.analysis.stats import (
    calc_clopper_pearson_ci,
    calc_stats,
    calc_wilson_ci,
    format_iso_margin,
    format_iso_pct,
)
from hara_lcc.severity.iso26262 import (
    compute_worst_case_verdict,
    determine_asil,
    evaluate_controllability,
    evaluate_iso_severity,
    evaluate_weighted_odd,
)


def test_confidence_intervals_and_stats_helpers():
    # Эталонная точка из отчёта: k=4, n=100 -> 95% Clopper-Pearson [1.10%, 9.93%]
    lo_cp, hi_cp = calc_clopper_pearson_ci(4, 100, confidence=0.95)
    assert lo_cp * 100.0 == pytest.approx(1.10, abs=0.01)
    assert hi_cp * 100.0 == pytest.approx(9.93, abs=0.01)
    assert calc_clopper_pearson_ci(0, 0) == (0.0, 0.0)
    assert calc_clopper_pearson_ci(0, 50)[0] == 0.0
    assert calc_clopper_pearson_ci(50, 50)[1] == 1.0

    # Доверительный интервал Уилсона
    lo_w, hi_w = calc_wilson_ci(4, 100, confidence=0.95)
    assert 0.0 < lo_w < 0.04 < hi_w < 0.10
    assert calc_wilson_ci(0, 0) == (0.0, 0.0)
    assert calc_wilson_ci(0, 50)[0] == 0.0
    assert calc_wilson_ci(50, 50)[1] == 1.0

    st = calc_stats([1.0, 2.0, 3.0, None])
    assert st["count"] == 3
    assert st["mean"] == pytest.approx(2.0)
    assert st["median"] == pytest.approx(2.0)
    assert calc_stats([])["count"] == 0

    assert format_iso_pct(10.004) == "10.00%"
    assert format_iso_pct(12.0) == "12.0%"
    assert format_iso_margin(0.004) == "+0.004%"
    assert format_iso_margin(2.0) == "+2.0%"


def test_iso_severity_weighted_odd_controllability_and_asil():
    # Проверка кумулятивного правила >10%
    res_s3 = evaluate_iso_severity({'S0': 80, 'S1': 5, 'S2': 4, 'S3': 11}, total_crashes=100)
    assert res_s3['assigned_class'] == 'S3'
    res_s2 = evaluate_iso_severity({'S0': 85, 'S1': 3, 'S2': 8, 'S3': 4}, total_crashes=100)
    assert res_s2['assigned_class'] == 'S2'
    res_s1 = evaluate_iso_severity({'S0': 80, 'S1': 15, 'S2': 5, 'S3': 0}, total_crashes=100)
    assert res_s1['assigned_class'] == 'S1'
    res_s0 = evaluate_iso_severity({'S0': 92, 'S1': 8, 'S2': 0, 'S3': 0}, total_crashes=100)
    assert res_s0['assigned_class'] == 'S0'

    odd = evaluate_weighted_odd(res_s1, res_s2, weight_a=0.20, weight_b=0.80)
    assert odd['assigned_class'] == 'S2'
    assert compute_worst_case_verdict('S1', 'S2') == 'S2'

    # Controllability C1 / C2 / C3
    rec_c1 = [{'side_collision': True, 'severity': 'S1', 'scenario_outcome': 'ok'}] * 100
    assert evaluate_controllability(rec_c1)['assigned_c'] == 'C1'
    rec_c2 = [{'side_collision': True, 'severity': 'S1', 'scenario_outcome': 'ok'}] * 95 + [
        {'side_collision': True, 'severity': 'S2', 'scenario_outcome': 'ok'}
    ] * 5
    assert evaluate_controllability(rec_c2)['assigned_c'] == 'C2'
    rec_c3 = [{'side_collision': True, 'severity': 'S1', 'scenario_outcome': 'ok'}] * 80 + [
        {'side_collision': True, 'severity': 'S2', 'scenario_outcome': 'ok'}
    ] * 20
    assert evaluate_controllability(rec_c3)['assigned_c'] == 'C3'

    # Матрица ISO 26262-3 Table 4
    assert determine_asil('S0', 'E4', 'C3') == 'QM'
    assert determine_asil('S1', 'E3', 'C2') == 'QM'
    assert determine_asil('S2', 'E3', 'C2') == 'ASIL A'
    assert determine_asil('S2', 'E3', 'C3') == 'ASIL B'
    assert determine_asil('S2', 'E4', 'C3') == 'ASIL C'
    assert determine_asil('S3', 'E4', 'C3') == 'ASIL D'


def test_full_analysis_exact_report_match_and_5_plots(
    baseline_csv_path: Path,
    baseline_report_path: Path,
    tmp_path: Path,
):
    args = build_arg_parser().parse_args([
        "--input", str(baseline_csv_path),
        "--output-dir", str(tmp_path),
    ])
    result = run_analysis(args)

    # 1. Проверка точного посимвольного совпадения summary_report.md с эталоном
    generated_report = (tmp_path / "summary_report.md").read_text(encoding="utf-8")
    expected_report = baseline_report_path.read_text(encoding="utf-8")
    assert generated_report == expected_report

    # 2. Проверка генерации всех 5 графиков PNG
    expected_plots = [
        "01_severity_distribution.png",
        "02_cumulative_iso_decision.png",
        "03_ditch_phase_portrait.png",
        "04_subclass_a_dv_cascade.png",
        "05_kinematics_vs_severity_boxplots.png",
    ]
    assert len(result["plot_paths"]) == 5
    for fname in expected_plots:
        p = tmp_path / "plots" / fname
        assert p.is_file(), f"Не создан график: {fname}"
        data = p.read_bytes()
        assert len(data) > 5000
        assert data[:8] == b"\x89PNG\r\n\x1a\n"


def test_analysis_with_active_input_renders_controllability_section(
    baseline_csv_path: Path,
    tmp_path: Path,
):
    args = build_arg_parser().parse_args([
        "--input", str(baseline_csv_path),
        "--active-input", str(baseline_csv_path),
        "--output-dir", str(tmp_path),
    ])
    result = run_analysis(args)
    assert result["ctrl_all"] is not None
    report_text = (tmp_path / "summary_report.md").read_text(encoding="utf-8")
    assert "Измеренный (`active`" in report_text
    assert "Объединенная кампания `active`" in report_text
