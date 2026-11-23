"""
Главный оркестратор кампании моделирования HARA в CARLA + Scenic и CLI-интерфейс runner.py.
"""

from __future__ import annotations

import argparse
import csv
import os
import random
import sys
import time

from hara_lcc.config import (
    CARLA_MAP_NAME,
    CSV_FIELDNAMES,
    CSV_PATH,
    DEFAULT_MAPS_DIR,
    DEFAULT_TIMESTEP_SEC,
    DEFAULT_WEIGHT_A,
    DEFAULT_WEIGHT_B,
    FRAME_LOGS_DIR,
    RESULTS_DIR,
    SCENIC_PATH,
    SIMULATION_HORIZON_SEC,
    SUBCLASS_A,
    SUBCLASS_B,
    TARGET_MASS_DEFAULT,
)
from hara_lcc.simulation.assets import resolve_map_assets
from hara_lcc.simulation.manager import AutopilotManager
from hara_lcc.simulation.physics import configure_world_physx
from hara_lcc.simulation.telemetry import (
    enforce_frame_log_quota,
    prepare_frame_logs_dir,
    write_frame_log,
)


def _configure_windows_utf8():
    if sys.platform == 'win32':
        try:
            sys.stdout.reconfigure(encoding='utf-8')
            sys.stderr.reconfigure(encoding='utf-8')
        except Exception:
            pass


def build_campaign_plan(subclass: str, mode: str, runs: int, rng=random) -> list[str]:
    """Формирование последовательности подклассов для кампании прогонов."""
    if subclass == 'A':
        return [SUBCLASS_A] * runs
    if subclass == 'B':
        return [SUBCLASS_B] * runs
    if mode == 'equal':
        runs_a = runs // 2
        runs_b = runs - runs_a
        return [SUBCLASS_A] * runs_a + [SUBCLASS_B] * runs_b
    if mode == 'sequential':
        runs_a = max(1, int(round(runs * DEFAULT_WEIGHT_A)))
        runs_b = max(1, runs - runs_a)
        return [SUBCLASS_A] * runs_a + [SUBCLASS_B] * runs_b
    return rng.choices([SUBCLASS_A, SUBCLASS_B], weights=[DEFAULT_WEIGHT_A, DEFAULT_WEIGHT_B], k=runs)


def format_scenario_outcome(
    subclass: str,
    has_side_hit: bool,
    has_barrier_hit: bool,
    barrier_impact_type: str,
    barrier_actor: str,
    ditch_actor: str,
    mechanism: str,
    severity: str,
    spawn_error_msg: str | None = None,
) -> str:
    """Классификация текстового исхода сценария на русском языке."""
    if spawn_error_msg is not None:
        return f"Ошибка спавна: {spawn_error_msg}"

    outcome = "Без столкновений"
    if subclass == SUBCLASS_A:
        b_type = barrier_impact_type
        actor_desc = f" ({barrier_actor})" if barrier_actor != 'None' else ""
        if has_side_hit and has_barrier_hit:
            outcome = f"Боковой удар с целевым ТС + {b_type} удар в ограждение{actor_desc}"
        elif has_barrier_hit:
            outcome = f"{b_type} удар в ограждение{actor_desc} (без удара целевого ТС)"
        elif has_side_hit:
            outcome = "Боковой удар с целевым ТС (Без удара в ограждение)"
        if ditch_actor != 'None':
            outcome = f"{outcome} + {mechanism} ({ditch_actor})"
    else:  # SUBCLASS_B
        prefix = "Боковой удар с целевым ТС + " if has_side_hit else ""
        if ditch_actor != 'None':
            outcome = f"{prefix}{mechanism} ({ditch_actor}) ({severity})"
        elif has_side_hit:
            outcome = "Боковой удар с целевым ТС (Без съезда в кювет)"
    return outcome


def build_csv_row(
    run_idx: int,
    subclass: str,
    manager: AutopilotManager,
    spawn_error_msg: str | None = None,
) -> tuple[dict, str, str, int, tuple]:
    """Сбор и каскадная агрегация метрик одиночного прогона в 25-колоночный словарь CSV."""
    has_side_hit = manager.side_collision_frame is not None
    has_barrier_hit = manager.barrier_collision_frame is not None
    barrier_actor = manager.get_barrier_collision_actor()

    ditch_tuple = manager.compute_ditch_metrics()
    (
        sev_ditch_val,
        ditch_actor,
        dep_speed,
        road_perp_speed,
        enc_angle,
        lat_skid,
        sideslip,
        yaw_rate,
        mechanism,
    ) = ditch_tuple

    final_sev_val = max(manager.sev_side_val, manager.sev_barrier_val, sev_ditch_val)
    severity = f"S{final_sev_val}"

    outcome = format_scenario_outcome(
        subclass=subclass,
        has_side_hit=has_side_hit,
        has_barrier_hit=has_barrier_hit,
        barrier_impact_type=manager.barrier_impact_type,
        barrier_actor=barrier_actor,
        ditch_actor=ditch_actor,
        mechanism=mechanism,
        severity=severity,
        spawn_error_msg=spawn_error_msg,
    )

    row_data = {
        'run_id': run_idx,
        'subclass': subclass,
        'driver_post_shock_mode': manager.driver_post_shock,
        'shock_duration_sec': round(manager.shock_duration_sec, 3),
        'side_collision': has_side_hit,
        'delta_v_ego_mps': round(manager.delta_v_ego, 2) if has_side_hit else '',
        'delta_v_target_mps': round(manager.delta_v_target, 2) if has_side_hit else '',
        'effective_side_dv_mps': round(manager.effective_side_delta_v, 2) if has_side_hit else '',
        'sev_side': f"S{manager.sev_side_val}",
        'barrier_collision': has_barrier_hit,
        'barrier_collision_actor': barrier_actor,
        'barrier_impact_type': manager.barrier_impact_type if has_barrier_hit else '',
        'barrier_impact_mps': round(manager.barrier_impact_speed, 2) if has_barrier_hit else '',
        'sev_barrier': f"S{manager.sev_barrier_val}",
        'ditch_departure_actor': ditch_actor,
        'departure_speed_mps': dep_speed if ditch_actor != 'None' else '',
        'road_perp_speed_mps': road_perp_speed if ditch_actor != 'None' else '',
        'encroachment_angle_deg': enc_angle if ditch_actor != 'None' else '',
        'vehicle_lat_skid_mps': lat_skid if ditch_actor != 'None' else '',
        'sideslip_angle_deg': sideslip if ditch_actor != 'None' else '',
        'departure_yaw_rate_deg_s': yaw_rate if ditch_actor != 'None' else '',
        'ditch_mechanism': mechanism if ditch_actor != 'None' else '',
        'sev_ditch': f"S{sev_ditch_val}",
        'severity': severity,
        'scenario_outcome': outcome,
    }
    return row_data, severity, outcome, sev_ditch_val, ditch_tuple


def build_arg_parser() -> argparse.ArgumentParser:
    """Построение CLI-парсера runner.py со 100% сохранением флагов."""
    parser = argparse.ArgumentParser(description="HARA LCC Blind Spot Simulation Campaign (Physical Runner)")
    parser.add_argument(
        '--subclass', type=str, default='ALL', choices=['A', 'B', 'ALL'],
        help="Road subclass: 'A' (Barrier), 'B' (Ditch), or 'ALL' (Campaign)",
    )
    parser.add_argument(
        '--driver-post-shock', type=str, default='passive',
        choices=['passive', 'active'],
        help="Driver behavior mode after shock phase: 'passive' (default, coasting for pure Severity S), or 'active' (counter-steering & braking for Controllability C)",
    )
    parser.add_argument(
        '--mode', type=str, default='equal', choices=['equal', 'sequential', 'weighted'],
        help="Campaign mode: 'equal' (50/50: N/2 of A, N/2 of B), 'sequential' (0.2 A / 0.8 B), or 'weighted'",
    )
    parser.add_argument('--runs', type=int, default=20, help="Total number of simulation runs (default: 20)")
    parser.add_argument(
        '--map-dir', type=str, default=DEFAULT_MAPS_DIR,
        help="Path to directory containing map assets (default: maps/assets)",
    )
    parser.add_argument(
        '--output', type=str, default=None,
        help="Custom output CSV file path (default: results/lcc_hara_results.csv)",
    )
    parser.add_argument('--clear', action='store_true', help="Clear/overwrite previous CSV results before running")
    parser.add_argument(
        '--log-frames', action='store_true',
        help="Записывать подробную покадровую телеметрию обоих автомобилей для каждого прогона в отдельный файл.",
    )
    parser.add_argument('--render', action='store_true', help="Enable Carla rendering window")
    parser.add_argument(
        '--timestep', type=float, default=DEFAULT_TIMESTEP_SEC,
        help="Simulation timestep dt in seconds (default: 0.01, recommended: 0.005 - 0.025)",
    )
    parser.add_argument(
        '--seed', type=int, default=42,
        help="Fixed random seed for deterministic Scenic scene generation and reproducibility (default: 42)",
    )
    return parser


def run_campaign(args: argparse.Namespace):
    """
    Главный оркестратор кампании моделирования HARA:
    - Подключается к CARLA и инициализирует мир с картой 'map_ditch'.
    - Разрешает пути к ассетам OpenDRIVE (maps/assets/map_ditch.xodr).
    - Компилирует сценарии Scenic для Подклассов А (ограждение) и B (кювет).
    - Выполняет физические прогоны с сохранением кинематики в 25-колоночный CSV.
    - НЕ содержит логики статистической агрегации кампании (>10%).
    """
    import carla
    import scenic
    from scenic.simulators.carla.simulator import CarlaSimulator

    output_csv_path = os.path.abspath(args.output) if args.output else CSV_PATH
    output_csv_dir = os.path.dirname(output_csv_path)
    if output_csv_dir:
        os.makedirs(output_csv_dir, exist_ok=True)

    # Фиксация случайного зерна для воспроизводимости стохастических сцен
    if args.seed is not None:
        random.seed(args.seed)
        print(f"  [РЕПРОДУЦИРУЕМОСТЬ] Установлен фиксированный random seed: {args.seed}")

    # Проверка диапазона шага симуляции dt
    if args.timestep < 0.005 or args.timestep > 0.025:
        print(
            f"  [ПРЕДУПРЕЖДЕНИЕ] Шаг симуляции dt={args.timestep}c находится вне рекомендованного диапазона "
            f"[0.005..0.025] с. Возможна нестабильность интегратора PhysX или избыточное время моделирования."
        )

    print("=" * 80)
    print(" HARA SIMULATION CAMPAIGN: LANE CHANGE COLLISION (LCC) BLIND SPOT FAILURE")
    print(" ISO 26262-3 / SAE J2980:2023 (Dual Collision Sensors & Ditch Boundary Conditions)")
    print(f" Post-Shock Driver Mode: '{args.driver_post_shock}' (Default: passive)")
    print(f" Output CSV: {output_csv_path}")
    print(f" Simulation Timestep dt: {args.timestep} s (Substepping: max_substep=0.01s, max_substeps=10)")
    print("=" * 80)

    # 1. Подключение к симулятору CARLA
    print("\n[1/4] Подключение к CARLA (localhost:2000)...")
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        print(f"  [OK] Успешное подключение к CARLA. Текущая карта: {world.get_map().name}")

        # Настройка PhysX substepping в CARLA для стабильной контактной динамики
        configure_world_physx(world, args.timestep)
        print(f"  [OK] Режим PhysX substepping включен (dt={args.timestep}c, max_substep_dt=0.01c, max_substeps=10).")
    except Exception as e:
        print(f"  [ОШИБКА] Не удалось подключиться к CARLA / настроить PhysX: {e}")
        sys.exit(1)

    # Разрешение путей к карте OpenDRIVE
    maps_assets_dir, map_xodr_path = resolve_map_assets(args.map_dir)
    print(f"  [КАРТА] Директория ассетов: {maps_assets_dir}")
    print(f"  [КАРТА] Файл OpenDRIVE: {map_xodr_path}")

    # 2. Инициализация CarlaSimulator Scenic
    print(f"\n[2/4] Инициализация CarlaSimulator с картой '{CARLA_MAP_NAME}'...")
    try:
        target_carla_map = None if CARLA_MAP_NAME in world.get_map().name else CARLA_MAP_NAME
        simulator = CarlaSimulator(
            carla_map=target_carla_map,
            map_path=map_xodr_path,
            timestep=args.timestep,
            render=args.render,
        )
        print("  [OK] CarlaSimulator успешно инициализирован.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой инициализации CarlaSimulator: {e}")
        sys.exit(1)

    # 3. Компиляция сценариев Scenic для обоих подклассов с актуальным путем к map и timestep
    print(f"\n[3/4] Компиляция Scenic сценария ({SCENIC_PATH})...")
    scenarios = {}
    try:
        scenarios[SUBCLASS_A] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_A, 'map': map_xodr_path, 'timestep': args.timestep},
        )
        scenarios[SUBCLASS_B] = scenic.scenarioFromFile(
            SCENIC_PATH,
            params={'subclass': SUBCLASS_B, 'map': map_xodr_path, 'timestep': args.timestep},
        )
        print("  [OK] Сценарии для Подкласса А (Ограждение) и Подкласса B (Кювет) скомпилированы.")
    except Exception as e:
        print(f"  [ОШИБКА] Сбой компиляции Scenic: {e}")
        sys.exit(1)

    # 4. Формирование плана кампании
    campaign_plan = build_campaign_plan(args.subclass, args.mode, args.runs)

    print(
        f"\n[4/4] Запуск кампании: всего {len(campaign_plan)} прогонов "
        f"({campaign_plan.count(SUBCLASS_A)} Подкласс А, {campaign_plan.count(SUBCLASS_B)} Подкласс B)"
    )

    # Подготовка выходного CSV-файла со строгой 25-колоночной схемой
    if args.clear and os.path.isfile(output_csv_path):
        try:
            os.remove(output_csv_path)
        except Exception:
            pass

    file_exists = os.path.isfile(output_csv_path)

    # Проверка совместимости колонок в существующем файле
    if file_exists and not args.clear:
        try:
            with open(output_csv_path, 'r', encoding='utf-8-sig') as f_check:
                header_line = f_check.readline()
                if 'ditch_mechanism' not in header_line or 'sev_side' not in header_line:
                    backup_csv = output_csv_path + ".legacy_backup.csv"
                    os.replace(output_csv_path, backup_csv)
                    print(f"  [ИНФО] Старый CSV сохранен в резервную копию: {backup_csv}")
                    file_exists = False
        except Exception:
            pass

    # Подготовка папки для покадровых логов и проверка дискового пространства
    log_frames_enabled = bool(args.log_frames)
    if log_frames_enabled:
        log_frames_enabled = prepare_frame_logs_dir(FRAME_LOGS_DIR, RESULTS_DIR, clear=args.clear)

    with open(output_csv_path, mode='w' if args.clear or not file_exists else 'a', newline='', encoding='utf-8-sig') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        if not file_exists or args.clear:
            writer.writeheader()

        # ----------------------------------------------------------------------
        # Цикл прогонов HARA кампании
        # ----------------------------------------------------------------------
        for run_idx, subclass in enumerate(campaign_plan, start=1):
            scenario = scenarios[subclass]
            scene, _ = scenario.generate()
            target_mass = float(scene.params.get('target_mass_kg', TARGET_MASS_DEFAULT))

            subclass_label = "Подкласс А (Ограждение)" if subclass == SUBCLASS_A else "Подкласс B (Кювет)"
            print("\n" + "=" * 80)
            print(f" ПРОГОН [{run_idx}/{len(campaign_plan)}] -> {subclass_label}")
            print(f" Режим водителя после реакции: {args.driver_post_shock} | Масса Target (Scenic): {target_mass:.1f} кг")
            print("=" * 80)

            manager = AutopilotManager(
                client,
                world,
                subclass=subclass,
                driver_post_shock=args.driver_post_shock,
                dt=args.timestep,
                log_frames=log_frames_enabled,
                target_mass=target_mass,
            )
            manager.start()

            spawn_error_msg = None
            try:
                max_steps = int(round(SIMULATION_HORIZON_SEC / args.timestep))
                simulator.simulate(scene, maxSteps=max_steps)
            except Exception as e:
                err_str = str(e).strip()
                exc_name = type(e).__name__
                if "Unable to spawn" in err_str or "SimulationCreationError" in exc_name or not manager.configured:
                    spawn_error_msg = err_str if "SimulationCreationError" in err_str else f"{exc_name}: {err_str}"
                print(f"[ПРЕДУПРЕЖДЕНИЕ] Симуляция завершена или прервана: {e}")
            finally:
                manager.stop()
                time.sleep(0.3)

                row_data, severity, outcome, sev_ditch_val, ditch_tuple = build_csv_row(
                    run_idx=run_idx,
                    subclass=subclass,
                    manager=manager,
                    spawn_error_msg=spawn_error_msg,
                )
                has_side_hit = bool(row_data['side_collision'])
                has_barrier_hit = bool(row_data['barrier_collision'])
                barrier_actor = row_data['barrier_collision_actor']
                (_, ditch_actor, dep_speed, _, _, _, sideslip, _, mechanism) = ditch_tuple

                writer.writerow(row_data)
                csv_file.flush()

                if log_frames_enabled and manager.frame_buffer:
                    log_filename = os.path.join(FRAME_LOGS_DIR, f"run_{run_idx:04d}_{subclass}.csv")
                    write_frame_log(manager.frame_buffer, log_filename)
                    log_frames_enabled = enforce_frame_log_quota(FRAME_LOGS_DIR, RESULTS_DIR, log_filename)

                print("\n" + "-" * 80)
                print(f"ИТОГ ПРОГОНА [{run_idx}]:")
                print(f"  * Подкласс: {subclass}")
                print(f"  * Масса Target (Scenic): {target_mass:.1f} кг")
                print(f"  * Режим водителя после реакции: {manager.driver_post_shock}")
                print(f"  * Время реакции водителя: {manager.shock_duration_sec:.3f} с ({manager.shock_duration_ticks} тиков)")
                print(
                    f"  * Удар ТС-ТС: {has_side_hit} "
                    + (
                        f"(Delta-V Ego: {manager.delta_v_ego:.2f} м/с, Target: {manager.delta_v_target:.2f} м/с, S_side: S{manager.sev_side_val})"
                        if has_side_hit
                        else ""
                    )
                )
                if subclass == SUBCLASS_A:
                    print(
                        f"  * Удар в ограждение: {has_barrier_hit} "
                        + (
                            f"({manager.barrier_impact_type}, {manager.barrier_impact_speed:.2f} м/с, актер: {barrier_actor}, S_barrier: S{manager.sev_barrier_val})"
                            if has_barrier_hit
                            else ""
                        )
                    )
                    if ditch_actor != 'None':
                        print(
                            f"  * Перекрёстный вылет в кювет: {ditch_actor} "
                            f"(Скорость: {dep_speed:.2f} м/с, Угол beta: {sideslip:.1f}°, Механизм: '{mechanism}', S_ditch: S{sev_ditch_val})"
                        )
                else:
                    print(
                        f"  * Вылет в кювет: {ditch_actor} "
                        + (
                            f"(Скорость: {dep_speed:.2f} м/с, Угол beta: {sideslip:.1f}°, Механизм: '{mechanism}', S_ditch: S{sev_ditch_val})"
                            if ditch_actor != 'None'
                            else ""
                        )
                    )
                print(f"  * Итоговая тяжесть прогона: >>> {severity} <<<")
                print(f"  * Исход: {outcome}")
                print("-" * 80)

                time.sleep(0.3)

    print("\n" + "=" * 80)
    print("КАМПАНИЯ СИМУЛЯЦИЙ ЗАВЕРШЕНА!")
    print(f"Результаты записаны в: {output_csv_path}")
    print("Для статистического анализа по правилу >10% ISO 26262-3 запустите:")
    print(f"  python analyze_results.py --input {output_csv_path}")
    print("=" * 80)


def main(argv: list[str] | None = None):
    _configure_windows_utf8()
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    run_campaign(args)
