"""Тесты схемы 25 колонок CSV, констант Config F, канонических путей карт и CLI-совместимости."""

from __future__ import annotations

import csv
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import importlib

import pytest

from hara_lcc import config
from hara_lcc.simulation.assets import resolve_map_assets
from hara_lcc.simulation.physics import configure_ego_physics, configure_target_physics


def test_csv_schema_has_exact_25_columns_and_matches_baseline(baseline_csv_path: Path):
    expected = [
        'run_id',
        'subclass',
        'driver_post_shock_mode',
        'shock_duration_sec',
        'side_collision',
        'delta_v_ego_mps',
        'delta_v_target_mps',
        'effective_side_dv_mps',
        'sev_side',
        'barrier_collision',
        'barrier_collision_actor',
        'barrier_impact_type',
        'barrier_impact_mps',
        'sev_barrier',
        'ditch_departure_actor',
        'departure_speed_mps',
        'road_perp_speed_mps',
        'encroachment_angle_deg',
        'vehicle_lat_skid_mps',
        'sideslip_angle_deg',
        'departure_yaw_rate_deg_s',
        'ditch_mechanism',
        'sev_ditch',
        'severity',
        'scenario_outcome',
    ]
    assert len(config.CSV_FIELDNAMES) == 25
    assert config.CSV_FIELDNAMES == expected

    with open(baseline_csv_path, mode='r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
    assert header == config.CSV_FIELDNAMES


def test_frame_log_schema_has_61_columns_and_matches_baseline(project_root: Path):
    assert len(config.FRAME_LOG_FIELDNAMES) == 61
    assert config.FRAME_LOG_FIELDNAMES[0] == 'frame'
    assert 'ego_in_ditch' in config.FRAME_LOG_FIELDNAMES
    assert 'target_in_ditch' in config.FRAME_LOG_FIELDNAMES

    sample_logs = sorted((project_root / "results" / "frame_logs").glob("*.csv"))
    if sample_logs:
        with open(sample_logs[0], mode='r', encoding='utf-8') as f:
            header = next(csv.reader(f))
        assert header == config.FRAME_LOG_FIELDNAMES


def test_config_f_and_geometry_invariants():
    assert config.EGO_MASS == 3490.0
    assert config.TARGET_MASS_DEFAULT == 1480.0
    assert config.RESTITUTION_E == 0.15
    assert config.ASPHALT_TIRE_FRICTION_SCALE == 1.05
    assert config.GRAVEL_TIRE_FRICTION_SCALE == 0.57
    assert config.USE_SWEEP_WHEEL_COLLISION is True
    assert config.EGO_FRONT_BRAKE_TORQUE_NM == 3800.0
    assert config.EGO_REAR_BRAKE_TORQUE_NM == 2050.0
    assert config.TARGET_FRONT_BRAKE_NM_PER_KG == 1.00
    assert config.TARGET_REAR_BRAKE_NM_PER_KG == 0.54
    assert config.SHOULDER_START_Y == 7.50
    assert config.FRICTION_TRIGGER_BOX_LEFT_Y == 8.50
    assert config.FRICTION_TRIGGER_BOX_RIGHT_Y == 32.00
    assert config.FRICTION_TRIGGER_CENTER_Y == 20.25
    assert config.FRICTION_TRIGGER_EXTENT_Y_CM == 1175.0
    assert config.DITCH_BOUNDARY_Y == 9.75
    assert config.BARRIER_FILTER_MAX_Y == 3.5


def test_canonical_map_assets_and_no_root_duplicates(project_root: Path):
    for removed in [
        "map.xodr",
        "map.snet",
        "map_ditch.xodr",
        "map_ditch.snet",
        "map_ditch.fbx",
        "maps/assets/map_example.xodr",
    ]:
        assert not (project_root / removed).exists(), f"Устаревший/дублирующий файл должен быть удалён: {removed}"

    assert (project_root / "maps" / "assets" / "map_ditch.Package.json").is_file()

    assets_dir, xodr_path = resolve_map_assets()
    assert Path(assets_dir).resolve() == (project_root / "maps" / "assets").resolve()
    assert Path(xodr_path).resolve() == (project_root / "maps" / "assets" / "map_ditch.xodr").resolve()


def test_resolve_map_assets_has_no_legacy_map_xodr_fallback(tmp_path: Path, monkeypatch):
    (tmp_path / "map.xodr").write_text("<OpenDRIVE/>", encoding="utf-8")
    monkeypatch.setattr("hara_lcc.simulation.assets.DEFAULT_MAPS_DIR", str(tmp_path))
    with pytest.raises(FileNotFoundError):
        resolve_map_assets(str(tmp_path))


def test_configure_vehicle_physics_config_f_mock():
    class _MockVehicle:
        def __init__(self):
            self.phys = SimpleNamespace(
                mass=0.0,
                use_sweep_wheel_collision=False,
                wheels=[SimpleNamespace(tire_friction=0.0, max_brake_torque=0.0) for _ in range(4)],
            )
            self.applied = False

        def get_physics_control(self):
            return self.phys

        def apply_physics_control(self, phys):
            self.phys = phys
            self.applied = True

    ego = _MockVehicle()
    configure_ego_physics(ego)
    assert ego.applied is True
    assert ego.phys.mass == 3490.0
    assert ego.phys.use_sweep_wheel_collision is True
    assert [w.tire_friction for w in ego.phys.wheels] == [1.05, 1.05, 1.05, 1.05]
    assert [w.max_brake_torque for w in ego.phys.wheels] == [3800.0, 3800.0, 2050.0, 2050.0]

    target = _MockVehicle()
    configure_target_physics(target, 1600.0)
    assert target.applied is True
    assert target.phys.mass == 1600.0
    assert target.phys.use_sweep_wheel_collision is True
    assert [w.tire_friction for w in target.phys.wheels] == [1.05, 1.05, 1.05, 1.05]
    assert [w.max_brake_torque for w in target.phys.wheels] == [1600.0, 1600.0, 864.0, 864.0]


def test_root_wrappers_and_utilities_backward_compatibility():
    runner_mod = importlib.import_module("runner")
    analyze_mod = importlib.import_module("analyze_results")
    install_mod = importlib.import_module("install_map_to_carla")
    friction_mod = importlib.import_module("test_shoulder_friction")

    # Проверяем CLI-флаги runner.py
    r_args = runner_mod.build_arg_parser().parse_args([
        "--subclass", "A",
        "--mode", "equal",
        "--driver-post-shock", "active",
        "--runs", "4",
        "--timestep", "0.01",
        "--seed", "42",
        "--output", "out.csv",
        "--clear",
    ])
    assert r_args.subclass == "A"
    assert r_args.driver_post_shock == "active"
    assert r_args.runs == 4
    assert r_args.clear is True

    # Проверяем CLI-флаги analyze_results.py (включая legacy --weight-c)
    a_args = analyze_mod.build_arg_parser().parse_args([
        "--input", "results/lcc_hara_results.csv",
        "--active-input", "results/active.csv",
        "--output-dir", "tmp_out",
        "--weight-a", "0.25",
        "--weight-b", "0.75",
        "--template", "templates/report_template.md",
    ])
    assert a_args.weight_a == 0.25
    assert a_args.weight_b == 0.75
    assert a_args.active_input == "results/active.csv"

    # Проверяем утилиты без запуска CARLA
    assert Path(install_mod.SOURCE_ASSETS_DIR).name == "assets"
    plateau = friction_mod.compute_steady_plateau_stats(
        [
            {"v_lon": 20.0, "s_along": 0.0, "decel_dvdt": 8.0},
            {"v_lon": 15.0, "s_along": 10.9, "decel_dvdt": 8.0},
            {"v_lon": 10.0, "s_along": 18.7, "decel_dvdt": 8.0},
        ],
        v_start=25.0,
    )
    assert plateau["n_window"] == 3
    assert plateau["a_med_g"] == pytest.approx(8.0 / 9.80665, rel=1e-4)


@pytest.mark.parametrize(
    "script_name",
    ["runner.py", "analyze_results.py", "install_map_to_carla.py", "test_shoulder_friction.py"],
)
def test_cold_start_cli_help_subprocesses(project_root: Path, script_name: str):
    proc = subprocess.run(
        [sys.executable, str(project_root / script_name), "--help"],
        cwd=str(project_root),
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, f"{script_name} --help failed:\n{proc.stderr}"

