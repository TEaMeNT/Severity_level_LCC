#!/usr/bin/env python3
"""
Тонкая точка входа для HARA-симуляции CARLA + Scenic (обратная совместимость CLI и импортов).
Реализация вынесена в модульный пакет `src/hara_lcc/`.
"""

from __future__ import annotations

import os
import sys

_SRC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from hara_lcc.config import (  # noqa: E402, F401
    ACTIVE_MAX_STEER_RATE_PER_SEC,
    ASPHALT_TIRE_FRICTION_SCALE,
    BASE_DIR,
    BRAKE_RISE_TIME_SEC,
    CARLA_API_PATH,
    CARLA_MAP_NAME,
    CRUISE_SPEED_MPS,
    CSV_FIELDNAMES,
    CSV_PATH,
    DEFAULT_MAPS_DIR,
    EGO_FRONT_BRAKE_TORQUE_NM,
    EGO_MASS,
    EGO_REAR_BRAKE_TORQUE_NM,
    FRAME_LOGS_DIR,
    FRAME_LOG_FIELDNAMES,
    GRAVEL_TIRE_FRICTION_SCALE,
    HOLD_TIME_MAX_SEC,
    HOLD_TIME_MIN_SEC,
    MAX_FRAME_LOGS_BYTES,
    MIN_DISK_FREE_BYTES,
    REACTION_TIME_LOG_SIGMA,
    REACTION_TIME_MAX_SEC,
    REACTION_TIME_MEDIAN_SEC,
    REACTION_TIME_MIN_SEC,
    RESTITUTION_E,
    RESULTS_DIR,
    SCENIC_PATH,
    SUBCLASS_A,
    SUBCLASS_B,
    TARGET_FRONT_BRAKE_NM_PER_KG,
    TARGET_MASS_DEFAULT,
    TARGET_REAR_BRAKE_NM_PER_KG,
)
from hara_lcc.driver import (  # noqa: E402, F401
    compute_post_impact_control,
    create_driver_model,
    normalize_deg,
    sample_reaction_and_hold_times,
)
from hara_lcc.severity import (  # noqa: E402, F401
    classify_ditch_boundary,
)
from hara_lcc.simulation import (  # noqa: E402, F401
    AutopilotManager,
    build_arg_parser,
    fast_forward_skipped_run,
    main,
    parse_run_ids,
    resolve_map_assets,
)


if __name__ == '__main__':
    main()
