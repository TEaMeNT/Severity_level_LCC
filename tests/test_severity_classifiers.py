"""Тесты классификаторов S0..S3: боковой Delta-V, барьер, кювет Variant B и эталонные кейсы v1.0."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import pytest

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
    SEVERITY_FROM_RANK,
    SEVERITY_RANK,
    SUBCLASS_A,
    SUBCLASS_B,
)
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
    # S0: Shallow grazing (V_dep <= 8 или V_perp <= 2 & V_lat <= 2 & beta < 15)
    assert classify_ditch_boundary(v_dep=7.5, v_perp_road=1.5, v_lat_car=1.0, beta=5.0, omega_z=5.0) == (0, 'Shallow Grazing Departure')
    assert classify_ditch_boundary(v_dep=20.0, v_perp_road=1.5, v_lat_car=1.8, beta=5.2, omega_z=2.0) == (0, 'Shallow Grazing Departure')
    # S1: Tracking slope departure внутри AASHTO RDG (V_dep > 8, 2 < V_perp <= 12)
    assert classify_ditch_boundary(v_dep=15.0, v_perp_road=3.0, v_lat_car=1.5, beta=6.0, omega_z=5.0) == (1, 'Traversable Tracking Slope Departure')
    # S1: Бывший порог V_perp=8.10 м/с теперь внутри безопасной зоны AASHTO RDG Fig. 3-7 (11.34 м/с) -> S1
    assert classify_ditch_boundary(v_dep=25.94, v_perp_road=8.10, v_lat_car=3.32, beta=7.4, omega_z=2.4) == (1, 'Traversable Tracking Slope Departure')
    assert classify_ditch_boundary(v_dep=25.95, v_perp_road=11.34, v_lat_car=4.0, beta=8.9, omega_z=3.0) == (1, 'Traversable Tracking Slope Departure')
    # S1: Low-speed skid (beta >= 15, V_dep <= 11) и боковое скольжение V_lat > 2.0 (строгое вложение S2 => S1)
    assert classify_ditch_boundary(v_dep=9.5, v_perp_road=1.5, v_lat_car=2.8, beta=22.0, omega_z=15.0) == (1, 'Traversable Tracking Slope Departure')
    assert classify_ditch_boundary(v_dep=25.0, v_perp_road=1.5, v_lat_car=5.4, beta=12.5, omega_z=2.0) == (1, 'Traversable Tracking Slope Departure')
    # S2: Чистый Cooperrider soil-trip без ручного гейта beta >= 15° (V_dep > 14, V_lat > 5.5)
    assert classify_ditch_boundary(v_dep=25.0, v_perp_road=5.8, v_lat_car=6.0, beta=13.9, omega_z=10.0) == (2, 'Moderate Rollover Risk')
    assert classify_ditch_boundary(v_dep=27.78, v_perp_road=1.2, v_lat_car=5.6, beta=11.6, omega_z=3.0) == (2, 'Moderate Rollover Risk')
    # S2: Non-tracking skid NCHRP 665 (V_dep > 11, beta >= 20)
    assert classify_ditch_boundary(v_dep=13.24, v_perp_road=5.41, v_lat_car=5.12, beta=22.8, omega_z=159.2) == (2, 'Moderate Rollover Risk')
    # S2: Hard Ditch Bottom Impact — выход за предел AASHTO RDG (V_dep > 8, 12.0 < V_perp <= 18.0)
    assert classify_ditch_boundary(v_dep=22.0, v_perp_road=12.5, v_lat_car=2.0, beta=5.0, omega_z=5.0) == (2, 'Hard Ditch Bottom Impact')
    # S3: High-Speed Soil-Trip Rollover (beta >= 25)
    assert classify_ditch_boundary(v_dep=24.04, v_perp_road=6.29, v_lat_car=12.67, beta=31.8, omega_z=33.5) == (3, 'High-Speed Soil-Trip Rollover')
    # S3: High-Speed Soil-Trip Rollover (beta >= 20 and omega_z > 45)
    assert classify_ditch_boundary(v_dep=20.0, v_perp_road=5.0, v_lat_car=7.0, beta=21.0, omega_z=50.0) == (3, 'High-Speed Soil-Trip Rollover')
    # S3: Severe Ditch Nose-In Impact — экстремально крутой влёт (V_dep > 8, V_perp > 18.0)
    assert classify_ditch_boundary(v_dep=27.0, v_perp_road=18.5, v_lat_car=2.0, beta=4.2, omega_z=5.0) == (3, 'Severe Ditch Nose-In Impact')


def test_ditch_strict_nesting_s3_implies_s2_implies_s1_on_physical_grid():
    """
    Проверка строгого вложения S3 => S2 => S1 и отсутствия скачков S0 -> S2
    на всей физичной 5D-сетке (V_dep, V_perp, V_lat = V_dep*sin(beta), beta, omega_z).
    """
    v_deps = [0.5, 4.0, 7.9, 8.0, 8.1, 10.9, 11.1, 13.9, 14.1, 16.0, 17.9, 18.1, 21.2, 22.0, 25.0, 27.78, 32.0]
    betas = [0.0, 3.0, 6.0, 10.0, 11.4, 11.6, 12.5, 14.0, 14.9, 15.0, 19.9, 20.0, 24.9, 25.0, 35.0, 50.0, 75.0]
    thetas = [0.0, 1.0, 3.0, 5.0, 10.0, 15.0, 25.0, 35.0, 42.0, 55.0, 75.0]
    omegas = [0.0, 20.0, 44.9, 45.1, 90.0]

    checked = 0
    for v_dep in v_deps:
        for beta in betas:
            v_lat = v_dep * math.sin(math.radians(beta))
            for theta in thetas:
                v_perp = v_dep * math.sin(math.radians(theta))
                for omega_z in omegas:
                    checked += 1
                    is_non_tracking = beta >= DITCH_NON_TRACKING_BETA_DEG
                    is_s3 = (
                        (v_dep > DITCH_S3_SKID_VDEP_MPS and (beta >= DITCH_S3_SKID_BETA_DEG or (beta >= DITCH_S3_SPIN_BETA_DEG and omega_z > DITCH_S3_SPIN_OMEGA_Z_DEG_S)))
                        or (v_dep > DITCH_VDEP_TRACKING_MIN_MPS and v_perp > DITCH_S3_NOSE_VPERP_MPS)
                    )
                    is_s2 = (
                        (v_dep > DITCH_S2_TRIP_VDEP_MPS and v_lat > DITCH_S2_TRIP_VLAT_MPS)
                        or (v_dep > DITCH_S2_SKID_VDEP_MPS and beta >= DITCH_S2_SKID_BETA_DEG)
                        or (v_dep > DITCH_VDEP_TRACKING_MIN_MPS and v_perp > DITCH_S2_BOTTOM_VPERP_MPS)
                    )
                    is_s1 = (
                        (v_dep > DITCH_VDEP_TRACKING_MIN_MPS and v_perp > DITCH_S1_TRACKING_VPERP_MPS)
                        or is_non_tracking
                        or (v_dep > DITCH_VDEP_TRACKING_MIN_MPS and v_lat > DITCH_S1_SKID_VLAT_MPS)
                    )

                    if is_s3:
                        assert is_s2, f"Нарушение S3 => S2: {(v_dep, v_perp, v_lat, beta, omega_z)}"
                    if is_s2:
                        assert is_s1, f"Нарушение S2 => S1 (скачок S0 -> S2): {(v_dep, v_perp, v_lat, beta, omega_z)}"

                    sev, _ = classify_ditch_boundary(v_dep, v_perp, v_lat, beta, omega_z)
                    expected = 3 if is_s3 else (2 if is_s2 else (1 if is_s1 else 0))
                    assert sev == expected

    # Непрерывность перехода через порог Cooperrider V_lat = 5.5 м/с при V_perp <= 2.0 м/с и beta < 15°
    sev_below, _ = classify_ditch_boundary(v_dep=27.78, v_perp_road=1.0, v_lat_car=5.49, beta=11.4, omega_z=0.0)
    sev_above, _ = classify_ditch_boundary(v_dep=27.78, v_perp_road=1.0, v_lat_car=5.51, beta=11.45, omega_z=0.0)
    assert (sev_below, sev_above) == (1, 2)
    assert checked > 15000


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
        ("35",  SUBCLASS_A, "S1", "S1", "S2", "S2", "Moderate Rollover Risk", True),
        ("49",  SUBCLASS_A, "S1", "S2", "S1", "S2", "Traversable Tracking Slope Departure", True),
        ("141", SUBCLASS_A, "S1", "S2", "S0", "S2", "Shallow Grazing Departure", True),
        ("208", SUBCLASS_B, "S1", "S0", "S2", "S2", "Moderate Rollover Risk", True),
        ("220", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
        ("256", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
        ("289", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
        ("303", SUBCLASS_B, "S1", "S0", "S3", "S3", "High-Speed Soil-Trip Rollover", True),
        ("400", SUBCLASS_B, "S1", "S0", "S1", "S1", "Traversable Tracking Slope Departure", True),
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
        assert rec["ditch_mechanism"] == expected_mech


def test_all_400_canonical_rows_consistent_with_classifiers(baseline_csv_path: Path, baseline_records: list[dict]):
    with open(baseline_csv_path, mode='r', encoding='utf-8-sig') as f:
        raw_rows = list(csv.DictReader(f))
    assert len(raw_rows) == len(baseline_records) == 400

    for raw, parsed in zip(raw_rows, baseline_records):
        assert parsed["sev_side"] == raw["sev_side"]
        assert parsed["sev_barrier"] == raw["sev_barrier"]
        assert parsed["sev_ditch"] == raw["sev_ditch"]
        assert parsed["ditch_mechanism"] == raw["ditch_mechanism"]
        assert parsed["severity"] == raw["severity"]
        if parsed["ditch_departure_actor"] not in ("None", "") and parsed["departure_speed_mps"]:
            sev_int, mech = classify_ditch_boundary(
                v_dep=parsed["departure_speed_mps"],
                v_perp_road=parsed["road_perp_speed_mps"],
                v_lat_car=parsed["vehicle_lat_skid_mps"],
                beta=parsed["sideslip_angle_deg"],
                omega_z=parsed["departure_yaw_rate_deg_s"],
            )
            assert parsed["sev_ditch"] == SEVERITY_FROM_RANK[sev_int]
            assert parsed["ditch_mechanism"] == mech
        expected_final = SEVERITY_FROM_RANK[
            max(
                SEVERITY_RANK[parsed["sev_side"]],
                SEVERITY_RANK[parsed["sev_barrier"]],
                SEVERITY_RANK[parsed["sev_ditch"]],
            )
        ]
        assert parsed["severity"] == expected_final
