#!/usr/bin/env python3
"""
Тонкая точка входа для статистического анализа HARA (обратная совместимость CLI и импортов).
Реализация вынесена в модульный пакет `src/hara_lcc/`.
"""

from __future__ import annotations

import os
import sys

_SRC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from hara_lcc.analysis import (  # noqa: E402, F401
    DEFAULT_REPORT_TEMPLATE,
    annotate_ditch_points,
    build_arg_parser,
    calc_clopper_pearson_ci,
    calc_stats,
    calc_wilson_ci,
    format_iso_margin,
    format_iso_pct,
    generate_markdown_report,
    generate_plots,
    load_simulation_results,
    main,
    plural_ru,
    print_console_summary,
    render_template,
    resolve_template_path,
    run_analysis,
    safe_savefig,
    to_bool,
    to_float,
)
from hara_lcc.config import (  # noqa: E402, F401
    ASIL_MATRIX,
    SEVERITY_CLASSES,
    SEVERITY_COLORS,
    SEVERITY_FROM_RANK,
    SEVERITY_RANK,
    SUBCLASS_COLORS,
    SUBCLASS_NAMES,
)
from hara_lcc.severity import (  # noqa: E402, F401
    compute_worst_case_verdict,
    determine_asil,
    evaluate_controllability,
    evaluate_iso_severity,
    evaluate_weighted_odd,
    is_crash_run,
    is_spawn_error_run,
    is_valid_barrier_hit,
    is_valid_ditch_departure,
)


if __name__ == '__main__':
    main()
