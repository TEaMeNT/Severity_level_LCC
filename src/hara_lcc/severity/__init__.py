"""Классификаторы тяжести S0..S3 и логика HARA (ISO 26262-3 / SAE J2980:2023)."""

from hara_lcc.severity.classifiers import (
    BarrierImpactMetrics,
    SideCollisionMetrics,
    classify_barrier_severity,
    classify_ditch_boundary,
    classify_side_severity,
    compute_barrier_impact,
    compute_side_collision_metrics,
    select_worst_ditch_metrics,
)
from hara_lcc.severity.iso26262 import (
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

__all__ = [
    "BarrierImpactMetrics",
    "SideCollisionMetrics",
    "classify_barrier_severity",
    "classify_ditch_boundary",
    "classify_side_severity",
    "compute_barrier_impact",
    "compute_side_collision_metrics",
    "compute_worst_case_verdict",
    "determine_asil",
    "evaluate_controllability",
    "evaluate_iso_severity",
    "evaluate_weighted_odd",
    "is_crash_run",
    "is_spawn_error_run",
    "is_valid_barrier_hit",
    "is_valid_ditch_departure",
    "select_worst_ditch_metrics",
]
