"""Слой симуляции HARA LCC в CARLA + Scenic."""

from hara_lcc.simulation.assets import resolve_map_assets
from hara_lcc.simulation.campaign import (
    build_arg_parser,
    build_campaign_plan,
    build_csv_row,
    format_scenario_outcome,
    main,
    run_campaign,
)
from hara_lcc.simulation.manager import AutopilotManager
from hara_lcc.simulation.physics import (
    configure_ego_physics,
    configure_target_physics,
    configure_vehicle_physics,
    configure_world_physx,
    spawn_shoulder_friction_trigger,
)
from hara_lcc.simulation.telemetry import (
    enforce_frame_log_quota,
    extract_telemetry_frame,
    prepare_frame_logs_dir,
    write_frame_log,
)

__all__ = [
    "AutopilotManager",
    "build_arg_parser",
    "build_campaign_plan",
    "build_csv_row",
    "configure_ego_physics",
    "configure_target_physics",
    "configure_vehicle_physics",
    "configure_world_physx",
    "enforce_frame_log_quota",
    "extract_telemetry_frame",
    "format_scenario_outcome",
    "main",
    "prepare_frame_logs_dir",
    "resolve_map_assets",
    "run_campaign",
    "spawn_shoulder_friction_trigger",
    "write_frame_log",
]
