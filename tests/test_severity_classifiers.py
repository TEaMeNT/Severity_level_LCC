"""Тесты классификаторов S0..S3: боковой Delta-V, барьер, кювет Variant B и эталонные кейсы v1.0."""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from hara_lcc.config import SEVERITY_FROM_RANK, SUBCLASS_A, SUBCLASS_B
from hara_lcc.severity.classifiers import (
    classify_barrier_severity,
    classify_ditch_boundary,
    classify_side_severity,
    compute_barrier_impact,
    compute_side_collision_metrics,
    select_worst_ditch_metrics,
)
from hara_lcc.simulation.campaign import format_scenario_outcome


def test_side_collision_momentum_and_thresholds():
    # m_ego = 3490, m_target = 1480, e = 0.15
    m = compute_side_collision_metrics(v_closing_lat=2.0, m_ego=3490.0, m_target=1480.0, restitution=0.15)
    expected_ego = (1480.0 / 4970.0) * 1.15 * 2.0
    expected_tgt = (3490.0 / 4970.0) * 1.15 * 2.0
    assert m.delta_v_ego == pytest.approx(expected_ego, rel=1e-9)
    assert m.delta_v_target == pytest.approx(expected_tgt, rel=1e-9)
    assert m.effective_side_dv == pytest.approx(expected_tgt, rel=1e-9)
    assert m.sev_side == 1  # ~1.615 > 1.0 and <= 3.5

    assert classify_side_severity(1.0) == 0
    assert classify_side_severity(1.01) == 1
    assert classify_side_severity(3.5) == 1
    assert classify_side_severity(3.51) == 2
    assert classify_side_severity(8.0) == 2
    assert classify_side_severity(8.01) == 3


def test_barrier_impact_inward_projection_and_bounce_rejection():
    # Дорога идёт на Восток (+X): lane_forward=(1, 0), lane_right=(0, 1).
    # Левый барьер на Y=0 => движение внутрь стены имеет vy < 0.
    inward = compute_barrier_impact(
        v_actor_x=25.0, v_actor_y=-6.5,
        fw_actor_x=0.95, fw_actor_y=-0.31,  # |cos_alpha| = 0.95 >= 0.707 -> Side-swipe
        lane_forward_x=1.0, lane_forward_y=0.0,
        lane_right_x=0.0, lane_right_y=1.0,
    )
    assert inward.v_barrier == pytest.approx(6.5)
    assert inward.impact_type == 'Side-swipe'
    assert inward.sev_barrier == 2  # > 6.0

    # Отскок от стены (vy > 0): строго обнуляется!
    bounce = compute_barrier_impact(
        v_actor_x=25.0, v_actor_y=+9.5,
        fw_actor_x=0.95, fw_actor_y=0.31,
        lane_forward_x=1.0, lane_forward_y=0.0,
        lane_right_x=0.0, lane_right_y=1.0,
    )
    assert bounce.v_barrier == 0.0
    assert bounce.sev_barrier == 0

    # Угол атаки > 45 deg (|cos_alpha| < 0.707) -> Frontal/Rear
    frontal = compute_barrier_impact(
        v_actor_x=5.0, v_actor_y=-12.0,
        fw_actor_x=0.5, fw_actor_y=-0.866,
        lane_forward_x=1.0, lane_forward_y=0.0,
        lane_right_x=0.0, lane_right_y=1.0,
    )
    assert frontal.impact_type == 'Frontal/Rear'
    assert frontal.v_barrier == pytest.approx(12.0)
    assert frontal.sev_barrier == 2  # 11 < 12 <= 18

    assert classify_barrier_severity(6.0, 'Frontal/Rear') == 0
    assert classify_barrier_severity(6.1, 'Frontal/Rear') == 1
    assert classify_barrier_severity(11.1, 'Frontal/Rear') == 2
    assert classify_barrier_severity(18.1, 'Frontal/Rear') == 3
    assert classify_barrier_severity(2.0, 'Side-swipe') == 0
    assert classify_barrier_severity(2.1, 'Side-swipe') == 1
    assert classify_barrier_severity(6.1, 'Side-swipe') == 2
    assert classify_barrier_severity(12.1, 'Side-swipe') == 3


def test_ditch_variant_b_all_branches():
    # S0: Shallow grazing
    assert classify_ditch_boundary(v_dep=7.5, v_perp_road=1.5, v_lat_car=1.0, beta=5.0, omega_z=5.0) == (0, 'Shallow Grazing Departure')
    # S1: Tracking slope departure (V_dep > 8, V_perp > 2)
    assert classify_ditch_boundary(v_dep=15.0, v_perp_road=3.0, v_lat_car=1.5, beta=6.0, omega_z=5.0) == (1, 'Traversable Tracking Slope Departure')
    # S1: Low-speed skid (beta >= 15, V_dep <= 11)
    assert classify_ditch_boundary(v_dep=9.5, v_perp_road=1.5, v_lat_car=2.8, beta=22.0, omega_z=15.0) == (1, 'Traversable Tracking Slope Departure')
    # S2: Cooperrider soil-trip (V_dep > 14, beta >= 15, V_lat > 5.5)
    assert classify_ditch_boundary(v_dep=22.0, v_perp_road=4.0, v_lat_car=6.0, beta=16.0, omega_z=10.0) == (2, 'Moderate Rollover Risk')
    # S2: Non-tracking skid (V_dep > 11, beta >= 20)
    assert classify_ditch_boundary(v_dep=13.24, v_perp_road=5.41, v_lat_car=5.12, beta=22.8, omega_z=159.2) == (2, 'Moderate Rollover Risk')
    # S2: Hard Ditch Bottom Impact (V_dep > 8, V_perp > 8.0)
    assert classify_ditch_boundary(v_dep=25.94, v_perp_road=8.10, v_lat_car=3.32, beta=7.4, omega_z=2.4) == (2, 'Hard Ditch Bottom Impact')
    # S3: High-Speed Soil-Trip Rollover (beta >= 25)
    assert classify_ditch_boundary(v_dep=24.04, v_perp_road=6.29, v_lat_car=12.67, beta=31.8, omega_z=33.5) == (3, 'High-Speed Soil-Trip Rollover')
    # S3: High-Speed Soil-Trip Rollover (beta >= 20 and omega_z > 45)
    assert classify_ditch_boundary(v_dep=20.0, v_perp_road=5.0, v_lat_car=7.0, beta=21.0, omega_z=50.0) == (3, 'High-Speed Soil-Trip Rollover')
    # S3: Severe Ditch Nose-In Impact (V_dep > 8, V_perp > 12.0)
    assert classify_ditch_boundary(v_dep=18.0, v_perp_road=12.5, v_lat_car=2.0, beta=5.0, omega_z=5.0) == (3, 'Severe Ditch Nose-In Impact')


def test_crossover_ditch_supported_in_both_subclasses():
    outcome_a = format_scenario_outcome(
        subclass=SUBCLASS_A,
        has_side_hit=True,
        has_barrier_hit=True,
        barrier_impact_type='Side-swipe',
        barrier_actor='both',
        ditch_actor='both',
        mechanism='Moderate Rollover Risk',
        severity='S2',
    )
    assert 'Moderate Rollover Risk (both)' in outcome_a

    outcome_b = format_scenario_outcome(
        subclass=SUBCLASS_B,
        has_side_hit=True,
        has_barrier_hit=False,
        barrier_impact_type='',
        barrier_actor='None',
        ditch_actor='target',
        mechanism='Hard Ditch Bottom Impact',
        severity='S2',
    )
    assert outcome_b == 'Боковой удар с целевым ТС + Hard Ditch Bottom Impact (target) (S2)'

    worst = select_worst_ditch_metrics({
        'ego': {'v_dep': 10.0, 'v_perp_road': 3.0, 'theta_enc': 17.5, 'v_lat_car': 1.0, 'beta': 5.0, 'omega_z': 2.0, 'mechanism': 'Traversable Tracking Slope Departure', 'actor_sev': 1},
        'target': {'v_dep': 25.0, 'v_perp_road': 8.2, 'theta_enc': 19.1, 'v_lat_car': 4.7, 'beta': 10.4, 'omega_z': 4.4, 'mechanism': 'Hard Ditch Bottom Impact', 'actor_sev': 2},
    })
    assert worst[0] == 2
    assert worst[1] == 'both'
    assert worst[8] == 'Hard Ditch Bottom Impact'


@pytest.mark.parametrize(
    "run_id, expected_subclass, expected_sev_side, expected_sev_barrier, expected_sev_ditch, expected_severity, expected_mech, expected_crash",
    [
        ("43",  SUBCLASS_A, "S0", "S0", "S0", "S0", "", False),
        ("49",  SUBCLASS_A, "S1", "S2", "S1", "S2", "Traversable Tracking Slope Departure", True),
        ("24",  SUBCLASS_A, "S0", "S1", "S2", "S2", "Moderate Rollover Risk", True),
        ("164", SUBCLASS_B, "S1", "S0", "S1", "S1", "Traversable Tracking Slope Departure", True),
        ("156", SUBCLASS_B, "S1", "S0", "S2", "S2", "Hard Ditch Bottom Impact", True),
        ("200", SUBCLASS_B, "S1", "S0", "S2", "S2", "Hard Ditch Bottom Impact", True),
        ("113", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
        ("116", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
    ],
)
def test_baseline_v1_reference_cases(
    baseline_records: list[dict],
    run_id: str,
    expected_subclass: str,
    expected_sev_side: str,
    expected_sev_barrier: str,
    expected_sev_ditch: str,
    expected_severity: str,
    expected_mech: str,
    expected_crash: bool,
):
    by_id = {str(r["run_id"]): r for r in baseline_records}
    rec = by_id[run_id]

    assert rec["subclass"] == expected_subclass
    assert rec["sev_side"] == expected_sev_side
    assert rec["sev_barrier"] == expected_sev_barrier
    assert rec["sev_ditch"] == expected_sev_ditch
    assert rec["severity"] == expected_severity
    assert rec["is_crash"] is expected_crash

    if expected_mech:
        sev_int, mech = classify_ditch_boundary(
            v_dep=rec["departure_speed_mps"],
            v_perp_road=rec["road_perp_speed_mps"],
            v_lat_car=rec["vehicle_lat_skid_mps"],
            beta=rec["sideslip_angle_deg"],
            omega_z=rec["departure_yaw_rate_deg_s"],
        )
        assert SEVERITY_FROM_RANK[sev_int] == expected_sev_ditch
        assert mech == expected_mech


def test_all_200_baseline_rows_match_raw_csv_severity_columns(baseline_csv_path: Path, baseline_records: list[dict]):
    with open(baseline_csv_path, mode='r', encoding='utf-8-sig') as f:
        raw_rows = list(csv.DictReader(f))
    assert len(raw_rows) == len(baseline_records) == 200

    for raw, parsed in zip(raw_rows, baseline_records):
        assert parsed["sev_side"] == raw["sev_side"]
        assert parsed["sev_barrier"] == raw["sev_barrier"]
        assert parsed["sev_ditch"] == raw["sev_ditch"]
        assert parsed["severity"] == raw["severity"]
