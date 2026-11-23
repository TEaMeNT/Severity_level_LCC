"""Слой статистического анализа, графиков и генерации отчётов HARA."""

from hara_lcc.analysis.cli import build_arg_parser, main, run_analysis
from hara_lcc.analysis.loader import load_simulation_results, to_bool, to_float
from hara_lcc.analysis.plots import (
    annotate_ditch_points,
    generate_plots,
    plot_cumulative_iso_decision,
    plot_ditch_phase_portrait,
    plot_kinematics_vs_severity,
    plot_severity_distribution,
    plot_subclass_a_dv_cascade,
    safe_savefig,
)
from hara_lcc.analysis.report import (
    DEFAULT_REPORT_TEMPLATE,
    generate_markdown_report,
    print_console_summary,
    render_template,
    resolve_template_path,
)
from hara_lcc.analysis.stats import (
    calc_clopper_pearson_ci,
    calc_stats,
    calc_wilson_ci,
    format_iso_margin,
    format_iso_pct,
    plural_ru,
)

__all__ = [
    "DEFAULT_REPORT_TEMPLATE",
    "annotate_ditch_points",
    "build_arg_parser",
    "calc_clopper_pearson_ci",
    "calc_stats",
    "calc_wilson_ci",
    "format_iso_margin",
    "format_iso_pct",
    "generate_markdown_report",
    "generate_plots",
    "load_simulation_results",
    "main",
    "plot_cumulative_iso_decision",
    "plot_ditch_phase_portrait",
    "plot_kinematics_vs_severity",
    "plot_severity_distribution",
    "plot_subclass_a_dv_cascade",
    "plural_ru",
    "print_console_summary",
    "render_template",
    "resolve_template_path",
    "run_analysis",
    "safe_savefig",
    "to_bool",
    "to_float",
]
