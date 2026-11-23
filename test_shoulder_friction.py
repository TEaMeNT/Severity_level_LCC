#!/usr/bin/env python3
"""
ТЕСТОВЫЙ КОМПЛЕКС КАЛИБРОВКИ И ВЕРИФИКАЦИИ ТРИГГЕРА ТРЕНИЯ ОБОЧИНЫ (CARLA PhysX, Config F)
===========================================================================================
Формула сцепления в CARLA / UE4 PhysX 4:
    tireFriction_i = mu_PhysMat * FrictionScale
где FrictionScale — безразмерный параметр CommonTireConfig / WheelPhysicsControl.tire_friction.

Конфигурация Config F:
- ASPHALT_TIRE_FRICTION_SCALE = 1.05 (~0.81-0.86 g на сухом асфальте)
- GRAVEL_TIRE_FRICTION_SCALE  = 0.57 (~0.46-0.48 g на укрепленной гравийной обочине)
- Пропорциональный EBD max_brake_torque (~65% перед / ~35% зад):
  * vehicle.chevrolet.impala (3490 кг): 3800 Н*м (перед) / 2050 Н*м (зад)
  * vehicle.seat.leon (1480 кг):      1.00 * m (перед) / 0.54 * m (зад)

Методология проверки (без воздушного сброса с z = 0.5 м):
1. Фаза 1: Граничный аудит координат срабатывания триггера static.trigger.friction (Y in [7.50, 9.75] м).
2. Фаза 2: Калибровочный бенчмарк влияния FrictionScale на плато замедления v in [0.25*v0, 0.85*v0].
3. Фаза 3: Наземное сравнение торможения (90 км/ч): Асфальт (Y=5.625) vs Обочина (Y=8.625).
4. Фаза 4: Непрерывное пересечение границы Y = 7.50 м на скорости 72 км/ч (проверка подвески).
"""

import argparse
import math
import statistics
import sys

# Настройка UTF-8 для корректного вывода в Windows PowerShell
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

import carla

G_ACCEL = 9.80665
DEFAULT_BLUEPRINTS = ['vehicle.chevrolet.impala', 'vehicle.seat.leon']

# Рабочие массы и EBD тормозной баланс Config F
EGO_MASS_KG = 3490.0
EGO_FRONT_BRAKE_NM = 3800.0
EGO_REAR_BRAKE_NM = 2050.0

TARGET_NOMINAL_MASS_KG = 1480.0
TARGET_FRONT_BRAKE_PER_KG = 1.00
TARGET_REAR_BRAKE_PER_KG = 0.54

SETTLE_MIN_SEC = 0.80
SETTLE_VZ_LIMIT = 0.02
SETTLE_AZ_LIMIT = 0.20


def compute_steady_plateau_stats(rows: list[dict], v_start: float) -> dict:
    """Расчёт установившегося плато замедления в окне v_lon in [0.25*v_start, 0.85*v_start]."""
    v_lo = 0.25 * v_start
    v_hi = 0.85 * v_start
    win = [r for r in rows if v_lo <= r['v_lon'] <= v_hi]
    if len(win) < 3:
        return {
            'n_window': len(win),
            'a_med_g': 0.0,
            'a_mean_g': 0.0,
            'a_energy_g': 0.0,
            'a_max_g': 0.0,
        }
    decels = [r['decel_dvdt'] for r in win]
    a_med = statistics.median(decels)
    a_mean = statistics.mean(decels)
    a_max = max(decels)
    v1 = win[0]['v_lon']
    v2 = win[-1]['v_lon']
    ds_win = max(1e-6, win[-1]['s_along'] - win[0]['s_along'])
    a_energy = (v1 * v1 - v2 * v2) / (2.0 * ds_win)
    return {
        'n_window': len(win),
        'a_med_g': a_med / G_ACCEL,
        'a_mean_g': a_mean / G_ACCEL,
        'a_energy_g': a_energy / G_ACCEL,
        'a_max_g': a_max / G_ACCEL,
    }


class FrictionTriggerValidator:
    def __init__(
        self,
        host: str = '127.0.0.1',
        port: int = 2000,
        dt: float = 0.01,
        asphalt_scale: float = 1.05,
        gravel_scale: float = 0.57,
    ):
        self.host = host
        self.port = port
        self.dt = float(dt)
        self.asphalt_scale = float(asphalt_scale)
        self.gravel_scale = float(gravel_scale)
        self.client = None
        self.world = None
        self.orig_settings = None
        self.bp_lib = None

    def connect(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(15.0)
        self.world = self.client.get_world()

        if 'map_ditch' not in self.world.get_map().name.lower():
            print("[SETUP] Загрузка карты 'map_ditch'...")
            self.world = self.client.load_world('map_ditch')

        self.orig_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        self.world.apply_settings(settings)
        self.bp_lib = self.world.get_blueprint_library()
        freq = 1.0 / self.dt
        print(f"[SETUP] Подключение успешно: {self.world.get_map().name}, dt={self.dt:.3f} с ({freq:.0f} Гц)")

    def cleanup(self):
        if self.world and self.orig_settings:
            try:
                self.world.apply_settings(self.orig_settings)
                print("\n[CLEANUP] Исходные настройки симулятора CARLA восстановлены.")
            except Exception as e:
                print(f"[CLEANUP ERROR] {e}")

    def _configure_vehicle_config_f(self, veh, blueprint_id: str, scale: float):
        """Назначение рабочей массы (Impala 3490 / Leon 1480), FrictionScale, EBD и sweep-коллайдеров."""
        pc = veh.get_physics_control()
        is_ego = ('impala' in blueprint_id.lower())
        mass_kg = EGO_MASS_KG if is_ego else TARGET_NOMINAL_MASS_KG
        pc.mass = float(mass_kg)
        pc.use_sweep_wheel_collision = True
        wheels = pc.wheels
        for idx, w in enumerate(wheels):
            w.tire_friction = float(scale)
            if is_ego:
                w.max_brake_torque = EGO_FRONT_BRAKE_NM if idx < 2 else EGO_REAR_BRAKE_NM
            else:
                w.max_brake_torque = float(
                    (TARGET_FRONT_BRAKE_PER_KG if idx < 2 else TARGET_REAR_BRAKE_PER_KG) * mass_kg
                )
        pc.wheels = wheels
        veh.apply_physics_control(pc)

    def _settle_and_launch_on_ground(
        self,
        veh,
        v0: float,
        settle_sec: float = SETTLE_MIN_SEC,
        enter_trigger_at_x: float = None,
    ) -> tuple[float, float]:
        """
        Свободное осаживание подвески на полотне (>= 0.5 с) с контролем |vz| < 0.02, |az| < 0.2,
        затем наземный разгон до v0 через enable_constant_velocity и краткий выбег перед торможением.
        """
        veh.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0, hand_brake=False))
        settle_ticks = max(int(round(max(0.50, settle_sec) / self.dt)), 50)
        vz_tail = []
        az_tail = []
        for i in range(settle_ticks):
            self.world.tick()
            v = veh.get_velocity()
            a = veh.get_acceleration()
            if i >= settle_ticks - 10:
                vz_tail.append(abs(v.z))
                az_tail.append(abs(a.z))

        # Доводим осадку до выполнения критерия покоя (макс +0.5 с)
        extra_ticks = 0
        max_extra = int(round(0.50 / self.dt))
        while (max(vz_tail) >= SETTLE_VZ_LIMIT or max(az_tail) >= SETTLE_AZ_LIMIT) and extra_ticks < max_extra:
            self.world.tick()
            v = veh.get_velocity()
            a = veh.get_acceleration()
            vz_tail = vz_tail[1:] + [abs(v.z)]
            az_tail = az_tail[1:] + [abs(a.z)]
            extra_ticks += 1

        settle_max_vz = max(vz_tail)
        settle_max_az = max(az_tail)

        hold_ticks = max(10, int(round(0.25 / self.dt)))
        veh.enable_constant_velocity(carla.Vector3D(float(v0), 0.0, 0.0))
        if enter_trigger_at_x is None:
            for _ in range(hold_ticks):
                self.world.tick()
        else:
            max_approach_ticks = int(round(3.0 / self.dt))
            for step_i in range(max_approach_ticks):
                self.world.tick()
                if step_i + 1 >= hold_ticks and veh.get_location().x >= enter_trigger_at_x + 8.0:
                    break
        veh.disable_constant_velocity()

        coast_ticks = max(5, int(round(0.08 / self.dt)))
        for _ in range(coast_ticks):
            self.world.tick()

        return settle_max_vz, settle_max_az

    def spawn_trigger(self, x_center: float = 500.0, friction_scale: float = None, extent_x_cm: float = 100000.0):
        scale = self.gravel_scale if friction_scale is None else float(friction_scale)
        bp = self.bp_lib.find('static.trigger.friction')
        bp.set_attribute('friction', str(scale))
        bp.set_attribute('extent_x', str(float(extent_x_cm)))
        bp.set_attribute('extent_y', '1175.0')    # 11.75 м полуширина (зона Y in [8.50, 32.00] м, центр Y = 20.25 м)
        bp.set_attribute('extent_z', '200.0')     # 2 м полувысота
        return self.world.spawn_actor(
            bp, carla.Transform(carla.Location(x=float(x_center), y=20.25, z=0.0))
        )

    def _run_braking_trial(self, blueprint_id: str, y_pos: float, v0: float, init_scale: float, enter_trigger_at_x: float = None):
        veh_bp = self.bp_lib.find(blueprint_id)
        start_x_spawn = 80.0 if enter_trigger_at_x is not None else 50.0
        veh = self.world.spawn_actor(veh_bp, carla.Transform(carla.Location(x=start_x_spawn, y=y_pos, z=0.05)))
        try:
            self._configure_vehicle_config_f(veh, blueprint_id, init_scale)
            settle_vz, settle_az = self._settle_and_launch_on_ground(
                veh, v0=v0, enter_trigger_at_x=enter_trigger_at_x
            )

            tf0 = veh.get_transform()
            loc0 = tf0.location
            fw0 = tf0.get_forward_vector()
            vel0 = veh.get_velocity()
            v_start = vel0.x * fw0.x + vel0.y * fw0.y
            v_prev = v_start

            veh.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=False))

            max_ticks = int(round(20.0 / self.dt))
            rows = []
            ticks = 0
            while ticks < max_ticks:
                self.world.tick()
                ticks += 1
                loc = veh.get_location()
                vel = veh.get_velocity()
                v_lon = vel.x * fw0.x + vel.y * fw0.y
                spd = math.hypot(vel.x, vel.y)
                s_along = (loc.x - loc0.x) * fw0.x + (loc.y - loc0.y) * fw0.y
                decel_dvdt = (v_prev - v_lon) / self.dt
                rows.append({
                    'v_lon': v_lon,
                    's_along': s_along,
                    'decel_dvdt': decel_dvdt,
                })
                v_prev = v_lon
                if v_lon <= 0.1 or spd <= 0.1:
                    break

            dist = rows[-1]['s_along'] if rows else 0.0
            t_stop = ticks * self.dt
            plateau = compute_steady_plateau_stats(rows, v_start)
            scale_read = veh.get_physics_control().wheels[0].tire_friction
            return {
                'dist': dist,
                't_stop': t_stop,
                'a_med_g': plateau['a_med_g'],
                'a_mean_g': plateau['a_mean_g'],
                'a_energy_g': plateau['a_energy_g'],
                'a_max_g': plateau['a_max_g'],
                'n_window': plateau['n_window'],
                'settle_vz': settle_vz,
                'settle_az': settle_az,
                'scale_read': scale_read,
            }
        finally:
            veh.destroy()
            self.world.tick()

    # --------------------------------------------------------------------------
    # ФАЗА 1: Граничный аудит координат срабатывания триггера
    # --------------------------------------------------------------------------
    def run_phase_1_boundary_audit(self, blueprint_id: str):
        print("\n" + "=" * 100)
        print(f"ФАЗА 1 [{blueprint_id}]: ГРАНИЧНЫЙ АУДИТ static.trigger.friction (Y_box in [8.50, 32.00] м)")
        print("=" * 100)

        tr = self.spawn_trigger(x_center=50.0, friction_scale=self.gravel_scale)
        veh_bp = self.bp_lib.find(blueprint_id)
        veh = self.world.spawn_actor(
            veh_bp,
            carla.Transform(carla.Location(x=50.0, y=4.0, z=0.05), carla.Rotation(yaw=90.0))
        )
        self._configure_vehicle_config_f(veh, blueprint_id, self.asphalt_scale)
        self._settle_and_launch_on_ground(veh, v0=4.0, settle_sec=0.50)
        half_len = veh.bounding_box.extent.x
        veh.enable_constant_velocity(carla.Vector3D(4.0, 0.0, 0.0))

        enter_y = None
        exit_y = None
        ditch_scales = []
        log_points = []
        total_ticks = int(round(5.6 / self.dt))
        log_step = max(1, int(round(0.40 / self.dt)))

        try:
            for i in range(total_ticks):
                self.world.tick()
                loc = veh.get_location()
                cur_scale = veh.get_physics_control().wheels[0].tire_friction
                in_trigger = abs(cur_scale - self.gravel_scale) < 0.05
                if 9.75 <= loc.y <= 25.0:
                    ditch_scales.append(cur_scale)

                if in_trigger and enter_y is None:
                    enter_y = loc.y
                    log_points.append((i, loc.y, cur_scale, f"ВХОД В ТРИГГЕР (FrictionScale -> {cur_scale:.2f})"))
                elif not in_trigger and enter_y is not None and exit_y is None:
                    exit_y = loc.y
                    log_points.append((i, loc.y, cur_scale, f"ВЫХОД ИЗ ТРИГГЕРА (FrictionScale -> {cur_scale:.2f})"))

                if i % log_step == 0:
                    status = "ОБОЧИНА/КЮВЕТ (0.57)" if in_trigger else "АСФАЛЬТ (1.05)"
                    log_points.append((i, loc.y, cur_scale, status))

            print(f"{'Тик':<6} | {'Позиция центра Y':<18} | {'FrictionScale':<15} | {'Статус триггера'}")
            print("-" * 80)
            log_points.sort(key=lambda x: x[0])
            for pt in log_points:
                print(f"{pt[0]:<6} | {pt[1]:8.3f} м          | {pt[2]:10.2f}     | {pt[3]}")

            print("-" * 80)
            print(f"  * Полудлина кузова (extent.x): {half_len:.3f} м")
            print(f"  * Границы бокса триггера: Y in [8.500, 32.000] м (центр Y = 20.250 м)")
            if enter_y is not None:
                print(f"  * Вход зафиксирован при Y_center = {enter_y:.3f} м (передний габарит Y = {enter_y + half_len:.3f} м)")
            if exit_y is not None:
                print(f"  * ВНИМАНИЕ: Выход зафиксирован при Y_center = {exit_y:.3f} м (задний габарит Y = {exit_y - half_len:.3f} м)")
            else:
                print(f"  * Выход до Y = 26.4 м не зафиксирован (EndOverlap в кювете отсутствует).")
            if ditch_scales:
                print(f"  * Контроль Y in [9.75, 25.0] м: min={min(ditch_scales):.2f}, max={max(ditch_scales):.2f} (ожидается {self.gravel_scale:.2f})")
        finally:
            veh.destroy()
            tr.destroy()
            self.world.tick()

    # --------------------------------------------------------------------------
    # ФАЗА 2: Калибровочный бенчмарк влияния FrictionScale (плато v in [0.25*v0, 0.85*v0])
    # --------------------------------------------------------------------------
    def run_phase_2_friction_benchmark(self, blueprint_id: str):
        print("\n" + "=" * 100)
        print(f"ФАЗА 2 [{blueprint_id}]: БЕНЧМАРК UTireConfig::FrictionScale (Config F EBD, V0 = 72 км/ч, плато [0.25..0.85]v0)")
        print("=" * 100)

        test_scales = sorted({0.45, self.gravel_scale, 0.80, 1.00, self.asphalt_scale, 1.40, 1.90, 3.50})
        results = []
        v0 = 20.0  # 72 км/ч

        for f_scale in test_scales:
            res = self._run_braking_trial(blueprint_id, y_pos=5.625, v0=v0, init_scale=f_scale)
            results.append((f_scale, res))

        print(f"{'FrictionScale':<13} | {'Путь':<9} | {'Время':<8} | {'a_med/g (плато)':<15} | {'a_en/g':<8} | {'a_max/g':<8} | {'settle vz/az'}")
        print("-" * 100)
        for f_scale, res in results:
            print(
                f"{f_scale:<13.2f} | {res['dist']:6.2f} м | {res['t_stop']:5.2f} с | "
                f"{res['a_med_g']:11.3f}g    | {res['a_energy_g']:6.3f}g | {res['a_max_g']:6.3f}g | "
                f"{res['settle_vz']:.4f} / {res['settle_az']:.3f}"
            )

        print("-" * 100)
        ratio_dist = results[0][1]['dist'] / max(0.01, results[-1][1]['dist'])
        print(f"  * При снижении FrictionScale с {results[-1][0]:.2f} до {results[0][0]:.2f} тормозной путь "
              f"растёт с {results[-1][1]['dist']:.2f} м до {results[0][1]['dist']:.2f} м (в {ratio_dist:.1f} раза).")

    # --------------------------------------------------------------------------
    # ФАЗА 3: Сравнение торможения: Асфальт (вне триггера) vs Обочина (в триггере)
    # --------------------------------------------------------------------------
    def run_phase_3_shoulder_braking(self, blueprint_id: str):
        print("\n" + "=" * 100)
        print(f"ФАЗА 3 [{blueprint_id}]: ТОРМОЖЕНИЕ Config F (90 км/ч) — АСФАЛЬТ (Y=5.625) VS ОБОЧИНА (Y=8.625)")
        print("=" * 100)

        # Триггер охватывает X in [100.0, 1100.0], Y in [7.50, 9.75] (центр 600.0, полудлина 500 м)
        tr = self.spawn_trigger(x_center=600.0, friction_scale=self.gravel_scale, extent_x_cm=50000.0)
        v0 = 25.0  # 90 км/ч

        try:
            asp = self._run_braking_trial(blueprint_id, y_pos=5.625, v0=v0, init_scale=self.asphalt_scale)
            grv = self._run_braking_trial(
                blueprint_id, y_pos=8.625, v0=v0, init_scale=self.asphalt_scale, enter_trigger_at_x=100.0
            )
        finally:
            tr.destroy()
            self.world.tick()

        print(f"{'Локация / Покрытие':<33} | {'FrictionScale':<13} | {'Путь':<9} | {'Время':<7} | {'a_med/g':<8} | {'a_en/g':<8} | {'a_max/g'}")
        print("-" * 100)
        print(f"{'1. Асфальт (вне триггера Y=5.625)':<33} | {asp['scale_read']:9.2f}     | {asp['dist']:6.2f} м | {asp['t_stop']:4.2f} с | "
              f"{asp['a_med_g']:6.3f}g | {asp['a_energy_g']:6.3f}g | {asp['a_max_g']:6.3f}g")
        print(f"{'2. Обочина (в триггере Y=8.625)':<33} | {grv['scale_read']:9.2f}     | {grv['dist']:6.2f} м | {grv['t_stop']:4.2f} с | "
              f"{grv['a_med_g']:6.3f}g | {grv['a_energy_g']:6.3f}g | {grv['a_max_g']:6.3f}g")
        print("-" * 100)
        print(f"  * Триггер обочины переключил FrictionScale: {asp['scale_read']:.2f} -> {grv['scale_read']:.2f} "
              f"(отношение {grv['scale_read'] / max(0.01, asp['scale_read']):.3f}).")

    # --------------------------------------------------------------------------
    # ФАЗА 4: Непрерывное пересечение границы Y = 7.50 м на ходу
    # --------------------------------------------------------------------------
    def run_phase_4_realtime_transition(self, blueprint_id: str):
        print("\n" + "=" * 100)
        print(f"ФАЗА 4 [{blueprint_id}]: ПОКАДРОВАЯ ТЕЛЕМЕТРИЯ ПЕРЕСЕЧЕНИЯ Y = 7.50 м (72 км/ч, наземный старт)")
        print("=" * 100)

        tr = self.spawn_trigger(x_center=500.0, friction_scale=self.gravel_scale)
        veh_bp = self.bp_lib.find(blueprint_id)
        start_tf = carla.Transform(carla.Location(x=50.0, y=4.5, z=0.05), carla.Rotation(yaw=20.0))
        veh = self.world.spawn_actor(veh_bp, start_tf)
        self._configure_vehicle_config_f(veh, blueprint_id, self.asphalt_scale)
        settle_vz, settle_az = self._settle_and_launch_on_ground(veh, v0=20.0)

        veh.apply_control(carla.VehicleControl(throttle=0.0, brake=0.50, steer=0.0, hand_brake=False))

        print(f"  [Осадка на полотне] max |vz|={settle_vz:.4f} м/с, max |az|={settle_az:.4f} м/с²")
        print(f"{'Тик':<5} | {'X (м)':<8} | {'Y (м)':<8} | {'Скорость V':<12} | {'FricScale':<9} | {'Roll':<9} | {'Pitch':<9} | {'Событие'}")
        print("-" * 100)

        steps_total = int(round(1.40 / self.dt))
        print_step = max(1, int(round(0.05 / self.dt)))
        max_vz_flat = 0.0
        max_az_flat = 0.0

        try:
            prev_scale = None
            for step in range(steps_total):
                self.world.tick()
                loc = veh.get_location()
                rot = veh.get_transform().rotation
                vel = veh.get_velocity()
                acc = veh.get_acceleration()
                spd = math.hypot(vel.x, vel.y)
                scale = veh.get_physics_control().wheels[0].tire_friction
                if loc.y < 9.75:
                    max_vz_flat = max(max_vz_flat, abs(vel.z))
                    max_az_flat = max(max_az_flat, abs(acc.z))

                event = ""
                if prev_scale is not None and abs(scale - prev_scale) > 0.05:
                    event = f"<--- [СМЕНА FrictionScale: {prev_scale:.2f} -> {scale:.2f}]"
                elif step == 0:
                    event = "Старт на асфальте"

                prev_scale = scale

                if step % print_step == 0 or event != "":
                    print(f"{step:<5} | {loc.x:7.2f} | {loc.y:7.3f} | {spd:8.2f} м/с  | {scale:7.2f}   | "
                          f"{rot.roll:6.2f}°  | {rot.pitch:6.2f}°  | {event}")

            print("-" * 100)
            print(f"ВЫВОД ТЕСТА 4: до бровки Y < 9.75 м max |vz|={max_vz_flat:.4f} м/с, max |az|={max_az_flat:.4f} м/с² — переход плавный без скачка подвески.")
        finally:
            veh.destroy()
            tr.destroy()
            self.world.tick()


def main():
    parser = argparse.ArgumentParser(description="CARLA PhysX Shoulder Friction & Deceleration Calibration (Config F)")
    parser.add_argument('--blueprint', nargs='+', default=DEFAULT_BLUEPRINTS,
                        help="Vehicle blueprint(s) to benchmark (default: impala + leon)")
    parser.add_argument('--timestep', type=float, default=0.01,
                        help="Simulation timestep dt in seconds (default: 0.01)")
    parser.add_argument('--asphalt-scale', type=float, default=1.05,
                        help="UE4 UTireConfig::FrictionScale for dry asphalt (default: 1.05)")
    parser.add_argument('--gravel-scale', type=float, default=0.57,
                        help="UE4 UTireConfig::FrictionScale for gravel shoulder (default: 0.57)")
    args = parser.parse_args()

    validator = FrictionTriggerValidator(
        dt=args.timestep,
        asphalt_scale=args.asphalt_scale,
        gravel_scale=args.gravel_scale,
    )
    try:
        validator.connect()
        for bp_id in args.blueprint:
            validator.run_phase_1_boundary_audit(bp_id)
            validator.run_phase_2_friction_benchmark(bp_id)
            validator.run_phase_3_shoulder_braking(bp_id)
            validator.run_phase_4_realtime_transition(bp_id)
    finally:
        validator.cleanup()


if __name__ == '__main__':
    main()
