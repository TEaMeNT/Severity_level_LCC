"""
Покадровое логирование телеметрии Ego и Target (61 колонка) и контроль дисковой квоты frame_logs.
"""

from __future__ import annotations

import csv
import math
import os
import shutil

from hara_lcc.config import (
    DITCH_BOUNDARY_Y,
    FRAME_LOG_FIELDNAMES,
    MAX_FRAME_LOGS_BYTES,
    MIN_DISK_FREE_BYTES,
)
from hara_lcc.driver.post_impact import normalize_deg


def extract_telemetry_frame(
    snapshot,
    tick_count: int,
    dt: float,
    lane_change_triggered: bool,
    lost_control: bool,
    collision_tick: int | None,
    side_collision_frame: int | None,
    barrier_collision_frame: int | None,
    ego_snap,
    target_snap,
    ego_vehicle,
    target_vehicle,
    lane_forward,
    lane_right,
) -> dict:
    """Извлечение полной покадровой телеметрии Ego и Target в словарь (61 колонка)."""
    sim_time = tick_count * dt
    phase = '2_maneuver' if lane_change_triggered else '1_sync'
    time_since_impact = (
        (tick_count - collision_tick) * dt
        if (lost_control and collision_tick is not None)
        else 0.0
    )

    # 1. Кинематика Ego
    e_tf = ego_snap.get_transform()
    e_loc = e_tf.location
    e_rot = e_tf.rotation
    e_vel = ego_snap.get_velocity()
    e_spd = math.hypot(e_vel.x, e_vel.y)
    e_fw = e_tf.get_forward_vector()
    e_rt = e_tf.get_right_vector()
    e_v_lon = e_vel.x * e_fw.x + e_vel.y * e_fw.y
    e_v_lat = e_vel.x * e_rt.x + e_vel.y * e_rt.y
    e_v_perp = e_vel.x * lane_right.x + e_vel.y * lane_right.y
    e_beta = math.degrees(math.atan2(abs(e_v_lat), max(abs(e_v_lon), 0.01)))
    e_v_lon_road = e_vel.x * lane_forward.x + e_vel.y * lane_forward.y
    e_theta = math.degrees(math.atan2(abs(e_v_perp), max(abs(e_v_lon_road), 0.01)))
    e_ang_vel = ego_snap.get_angular_velocity()
    e_acc = ego_snap.get_acceleration()

    try:
        e_ctrl = ego_vehicle.get_control()
        e_steer, e_throttle, e_brake = e_ctrl.steer, e_ctrl.throttle, e_ctrl.brake
    except Exception:
        e_steer, e_throttle, e_brake = 0.0, 0.0, 0.0

    # 2. Кинематика Target
    t_tf = target_snap.get_transform()
    t_loc = t_tf.location
    t_rot = t_tf.rotation
    t_vel = target_snap.get_velocity()
    t_spd = math.hypot(t_vel.x, t_vel.y)
    t_fw = t_tf.get_forward_vector()
    t_rt = t_tf.get_right_vector()
    t_v_lon = t_vel.x * t_fw.x + t_vel.y * t_fw.y
    t_v_lat = t_vel.x * t_rt.x + t_vel.y * t_rt.y
    t_v_perp = t_vel.x * lane_right.x + t_vel.y * lane_right.y
    t_beta = math.degrees(math.atan2(abs(t_v_lat), max(abs(t_v_lon), 0.01)))
    t_v_lon_road = t_vel.x * lane_forward.x + t_vel.y * lane_forward.y
    t_theta = math.degrees(math.atan2(abs(t_v_perp), max(abs(t_v_lon_road), 0.01)))
    t_ang_vel = target_snap.get_angular_velocity()
    t_acc = target_snap.get_acceleration()

    try:
        t_ctrl = target_vehicle.get_control()
        t_steer, t_throttle, t_brake = t_ctrl.steer, t_ctrl.throttle, t_ctrl.brake
    except Exception:
        t_steer, t_throttle, t_brake = 0.0, 0.0, 0.0

    return {
        'frame': snapshot.frame,
        'tick_count': tick_count,
        'sim_time_sec': round(sim_time, 4),
        'phase': phase,
        'lane_change_triggered': lane_change_triggered,
        'lost_control': lost_control,
        'time_since_impact_sec': round(time_since_impact, 4),
        'side_collision_occurred': side_collision_frame is not None,
        'barrier_collision_occurred': barrier_collision_frame is not None,

        'ego_x': round(e_loc.x, 3),
        'ego_y': round(e_loc.y, 3),
        'ego_z': round(e_loc.z, 3),
        'ego_roll_deg': round(normalize_deg(e_rot.roll), 2),
        'ego_pitch_deg': round(normalize_deg(e_rot.pitch), 2),
        'ego_yaw_deg': round(normalize_deg(e_rot.yaw), 2),
        'ego_vx': round(e_vel.x, 3),
        'ego_vy': round(e_vel.y, 3),
        'ego_vz': round(e_vel.z, 3),
        'ego_speed_mps': round(e_spd, 3),
        'ego_speed_kmh': round(e_spd * 3.6, 2),
        'ego_v_lon_car_mps': round(e_v_lon, 3),
        'ego_v_lat_car_mps': round(e_v_lat, 3),
        'ego_v_perp_road_mps': round(e_v_perp, 3),
        'ego_sideslip_beta_deg': round(e_beta, 2),
        'ego_road_angle_theta_deg': round(e_theta, 2),
        'ego_omega_x_deg_s': round(e_ang_vel.x, 2),
        'ego_omega_y_deg_s': round(e_ang_vel.y, 2),
        'ego_omega_z_deg_s': round(e_ang_vel.z, 2),
        'ego_ax': round(e_acc.x, 3),
        'ego_ay': round(e_acc.y, 3),
        'ego_az': round(e_acc.z, 3),
        'ego_steer': round(e_steer, 4),
        'ego_throttle': round(e_throttle, 4),
        'ego_brake': round(e_brake, 4),
        'ego_in_ditch': bool(e_loc.y >= DITCH_BOUNDARY_Y),

        'target_x': round(t_loc.x, 3),
        'target_y': round(t_loc.y, 3),
        'target_z': round(t_loc.z, 3),
        'target_roll_deg': round(normalize_deg(t_rot.roll), 2),
        'target_pitch_deg': round(normalize_deg(t_rot.pitch), 2),
        'target_yaw_deg': round(normalize_deg(t_rot.yaw), 2),
        'target_vx': round(t_vel.x, 3),
        'target_vy': round(t_vel.y, 3),
        'target_vz': round(t_vel.z, 3),
        'target_speed_mps': round(t_spd, 3),
        'target_speed_kmh': round(t_spd * 3.6, 2),
        'target_v_lon_car_mps': round(t_v_lon, 3),
        'target_v_lat_car_mps': round(t_v_lat, 3),
        'target_v_perp_road_mps': round(t_v_perp, 3),
        'target_sideslip_beta_deg': round(t_beta, 2),
        'target_road_angle_theta_deg': round(t_theta, 2),
        'target_omega_x_deg_s': round(t_ang_vel.x, 2),
        'target_omega_y_deg_s': round(t_ang_vel.y, 2),
        'target_omega_z_deg_s': round(t_ang_vel.z, 2),
        'target_ax': round(t_acc.x, 3),
        'target_ay': round(t_acc.y, 3),
        'target_az': round(t_acc.z, 3),
        'target_steer': round(t_steer, 4),
        'target_throttle': round(t_throttle, 4),
        'target_brake': round(t_brake, 4),
        'target_in_ditch': bool(t_loc.y >= DITCH_BOUNDARY_Y),
    }


def prepare_frame_logs_dir(frame_logs_dir: str, results_dir: str, clear: bool = False) -> bool:
    """Проверка дискового резерва и подготовка директории покадровых логов."""
    try:
        os.makedirs(results_dir, exist_ok=True)
        free_bytes = shutil.disk_usage(results_dir).free
        if free_bytes < MIN_DISK_FREE_BYTES:
            print(
                f"  [ПРЕДУПРЕЖДЕНИЕ] Свободное место на диске ({free_bytes / (1024**3):.2f} ГБ) "
                f"меньше лимита 1.5 ГБ! Покадровое логирование отключено."
            )
            return False

        os.makedirs(frame_logs_dir, exist_ok=True)
        if clear:
            for fname in os.listdir(frame_logs_dir):
                f_path = os.path.join(frame_logs_dir, fname)
                if os.path.isfile(f_path):
                    try:
                        os.remove(f_path)
                    except Exception:
                        pass
            print(f"  [ИНФО] Папка покадровых логов очищена: {frame_logs_dir}")
        return True
    except Exception as e:
        print(f"  [ПРЕДУПРЕЖДЕНИЕ] Ошибка подготовки папки логов: {e}")
        return False


def write_frame_log(frame_buffer: list[dict], log_filename: str) -> bool:
    """Запись буфера покадровой телеметрии в CSV-файл."""
    try:
        with open(log_filename, mode='w', newline='', encoding='utf-8') as f_log:
            log_writer = csv.DictWriter(f_log, fieldnames=FRAME_LOG_FIELDNAMES)
            log_writer.writeheader()
            log_writer.writerows(frame_buffer)
        return True
    except Exception as err:
        print(f"  [ПРЕДУПРЕЖДЕНИЕ] Ошибка записи покадрового лога {log_filename}: {err}")
        return False


def enforce_frame_log_quota(frame_logs_dir: str, results_dir: str, last_log_filename: str) -> bool:
    """
    Динамический контроль дискового пространства (лимит 1 ГБ и резерв 1.5 ГБ).
    Возвращает True, если логирование можно продолжать, и False при превышении лимита.
    """
    try:
        current_logs_size = sum(
            os.path.getsize(os.path.join(frame_logs_dir, f))
            for f in os.listdir(frame_logs_dir)
            if os.path.isfile(os.path.join(frame_logs_dir, f))
        )
        current_free = shutil.disk_usage(results_dir).free

        if current_logs_size > MAX_FRAME_LOGS_BYTES or current_free < MIN_DISK_FREE_BYTES:
            print(
                f"\n  [ПРЕДУПРЕЖДЕНИЕ] Превышен лимит размера покадровых логов "
                f"({current_logs_size / (1024**2):.1f} МБ > 1024 МБ) или порог диска "
                f"({current_free / (1024**3):.2f} ГБ < 1.5 ГБ)!"
            )
            print("  [ПРЕДУПРЕЖДЕНИЕ] Удаление избыточного файла и отключение покадрового логирования для оставшихся прогонов.")
            if os.path.isfile(last_log_filename):
                try:
                    os.remove(last_log_filename)
                except Exception:
                    pass
            return False
    except Exception:
        pass
    return True
