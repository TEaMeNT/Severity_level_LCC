"""
Оркестратор одиночного физического прогона в CARLA (`AutopilotManager`):
- Фаза 0: инициализация актеров, назначение физики Config F (EBD, sweep, FrictionScale),
  прикрепление датчиков коллизий на Ego и Target, спавн триггера обочины static.trigger.friction.
- Фаза 1: синхронный разгон до 100 км/ч и ПИД-удержание Target в слепой зоне.
- Фаза 2: маневр перестроения и пост-ударное управление через изолированную модель водителя `hara_lcc.driver`.
- Двусторонний мониторинг ударов об ограждение и метод граничных условий на бровке кювета Y = 9.75 м.
"""

from __future__ import annotations

import math
import os
import sys
import traceback

from hara_lcc.config import (
    BARRIER_FILTER_MAX_Y,
    BARRIER_HOLD_CUTOFF_Y,
    CARLA_API_PATH,
    CRUISE_SPEED_MPS,
    DITCH_BOUNDARY_Y,
    EGO_MASS,
    LANE_LEFT_CENTER_Y,
    LANE_RIGHT_CENTER_Y,
    OUT_OF_BOUNDS_Y,
    OUT_OF_BOUNDS_Z,
    PID_MAX_THROTTLE,
    RESTITUTION_E,
    SAFE_PASS_HOLD_SEC,
    SAFE_PASS_LANE_TOL_M,
    SAFE_PASS_VY_TOL_MPS,
    SAFE_PASS_YAW_TOL_DEG,
    SUBCLASS_A,
    SUBCLASS_B,
    TARGET_MASS_DEFAULT,
)
from hara_lcc.driver.post_impact import (
    PostImpactDriverModel,
    PostImpactState,
    create_driver_model,
    normalize_deg,
)
from hara_lcc.driver.reaction import sample_reaction_and_hold_times
from hara_lcc.severity.classifiers import (
    classify_ditch_boundary,
    compute_barrier_impact,
    compute_side_collision_metrics,
    select_worst_ditch_metrics,
)
from hara_lcc.simulation.physics import (
    configure_ego_physics,
    configure_target_physics,
    spawn_shoulder_friction_trigger,
)
from hara_lcc.simulation.telemetry import extract_telemetry_frame


def _get_vehicle_pid_controller_cls():
    """Ленивый импорт VehiclePIDController из CARLA PythonAPI (C:\\CARLA\\PythonAPI\\carla)."""
    if CARLA_API_PATH not in sys.path and os.path.isdir(CARLA_API_PATH):
        sys.path.append(CARLA_API_PATH)
    try:
        from agents.navigation.controller import VehiclePIDController  # type: ignore

        return VehiclePIDController
    except ImportError:
        return None


def is_ego_settled_in_target_lane(
    subclass: str,
    ego_lane_id: int | None,
    start_lane_id: int | None,
    ego_y: float,
    ego_vy: float,
    ego_yaw_deg: float,
) -> bool:
    """
    Чистый критерий завершения перестроения Ego в целевую полосу (откалиброван по 200 frame_logs):
    - Ego перешёл в целевую полосу (lane_id != start_lane_id);
    - |ego_y - target_lane_center_y| <= 0.35 м (покрывает штатное перерегулирование ПИД ~0.28 м);
    - |ego_vy| <= 0.35 м/с и |yaw_deg| <= 2.5°.
    """
    if ego_lane_id is None or start_lane_id is None or ego_lane_id == start_lane_id:
        return False
    target_center_y = LANE_LEFT_CENTER_Y if subclass == SUBCLASS_A else LANE_RIGHT_CENTER_Y
    yaw_norm = abs(normalize_deg(ego_yaw_deg))
    return (
        abs(ego_y - target_center_y) <= SAFE_PASS_LANE_TOL_M
        and abs(ego_vy) <= SAFE_PASS_VY_TOL_MPS
        and yaw_norm <= SAFE_PASS_YAW_TOL_DEG
    )


class AutopilotManager:
    """
    Менеджер симуляции HARA LCC Blind Spot Failure:
    - Фаза 0: Инициализация актеров, назначение масс PhysX, прикрепление датчиков коллизий к Ego и Target.
    - Фаза 1: Синхронный разгон до 100 км/ч и ПИД-удержание целевого ТС в слепой зоне.
    - Фаза 2: Инициация маневра перестроения, когнитивная модель водителя SAE J2980.
    - Двусторонний мониторинг ударов об отбойник (Ego + Target) по ISO 26262-3.
    - Метод граничных условий на бровке кювета (Y = 9.75 м) без искусственной террамеханики.
    """

    def __init__(
        self,
        client,
        world,
        subclass: str = SUBCLASS_B,
        driver_post_shock: str = 'passive',
        dt: float = 0.01,
        log_frames: bool = False,
        target_mass: float = TARGET_MASS_DEFAULT,
        driver_model: PostImpactDriverModel | None = None,
    ):
        self.client = client
        self.world = world
        self.subclass = subclass
        self.driver_post_shock = driver_post_shock
        self.driver_model = driver_model if driver_model is not None else create_driver_model(driver_post_shock)
        self.dt = float(dt)
        self.log_frames = log_frames
        self.target_mass = float(target_mass)
        self.frame_buffer: list[dict] = []

        if self.client is not None:
            self.tm = self.client.get_trafficmanager(8000)
            self.tm.set_synchronous_mode(True)
        else:
            self.tm = None

        self.ego_vehicle = None
        self.target_vehicle = None
        self.configured = False
        self.is_running = False
        self.spectator = None
        self.tick_count = 0
        self.callback_id = None

        self.lane_change_triggered = False
        self.lost_control = False
        self.collision_tick = None
        self.post_lane_change_safe_time_sec = 0.0
        self.safe_pass_confirmed = False

        self.target_v0_mps = CRUISE_SPEED_MPS
        self.ego_tm_speed = self.target_v0_mps + 0.4

        self.initial_lon_offset = 0.0
        self.target_err_integral = 0.0
        self.stability_time_sec = 0.0

        self.pid_controller = None
        self.start_lane_id = None

        # Датчики столкновений для обоих ТС (ISO 26262-3)
        self.ego_collision_sensor = None
        self.target_collision_sensor = None
        self.friction_trigger = None
        self.speed_history: dict[int, dict] = {}

        # ----------------------------------------------------------------------
        # Модель реакции водителей Ego и Target (Green 2000, Taoka 1989, SAE J2980:2023 5.2.3.2(d))
        # ----------------------------------------------------------------------
        self.ego_shock_duration_sec, self.ego_spasm_duration_sec = sample_reaction_and_hold_times()
        self.target_shock_duration_sec, self.target_spasm_duration_sec = sample_reaction_and_hold_times()

        self.ego_shock_duration_ticks = max(1, int(round(self.ego_shock_duration_sec / self.dt)))

        # Обратная совместимость с CSV-выгрузкой (shock_duration_sec)
        self.shock_duration_sec = self.ego_shock_duration_sec
        self.shock_duration_ticks = self.ego_shock_duration_ticks

        # Кэш параметров управления Ego до и после удара
        self.last_steer = 0.0
        self.last_throttle = 0.55
        self.impact_steer = 0.0
        self.impact_throttle = 0.55
        self.current_steer = 0.0
        self.ego_brake_cmd = 0.0

        # Кэш параметров управления Target до и после удара
        self.last_target_steer = 0.0
        self.last_target_throttle = 0.55
        self.target_impact_steer = 0.0
        self.target_impact_throttle = 0.55
        self.target_current_steer = 0.0
        self.target_brake_cmd = 0.0

        # ----------------------------------------------------------------------
        # Первичное столкновение ТС с ТС (SAE J2980 Table B1)
        # ----------------------------------------------------------------------
        self.side_collision_frame = None
        self.v_closing_lat = 0.0
        self.delta_v_ego = 0.0
        self.delta_v_target = 0.0
        self.effective_side_delta_v = 0.0
        self.sev_side_val = 0

        # ----------------------------------------------------------------------
        # Вторичный удар об ограждение (Подкласс А)
        # ----------------------------------------------------------------------
        self.barrier_collision_frame = None
        self.barrier_impact_speed = 0.0
        self.barrier_impact_type = ""
        self.sev_barrier_val = 0
        self.barrier_hit_actors: set[str] = set()  # {'ego', 'target'}

        # ----------------------------------------------------------------------
        # Граничные условия вылета в кювет на бровке Y = 9.75 м (Подкласс B)
        # ----------------------------------------------------------------------
        self.ditch_metrics_by_actor: dict[str, dict] = {}  # actor_name ('ego'/'target') -> dict of metrics

    def _activate_shock_phase(self, frame: int):
        """Симметричный перевод обоих ТС в пост-ударное состояние при первом контакте."""
        if self.lost_control:
            return
        import carla

        self.lost_control = True
        self.collision_tick = self.tick_count

        self.impact_steer = self.last_steer
        self.impact_throttle = self.last_throttle
        self.current_steer = self.impact_steer
        self.ego_brake_cmd = 0.0

        self.target_impact_steer = self.last_target_steer
        self.target_impact_throttle = self.last_target_throttle
        self.target_current_steer = self.target_impact_steer
        self.target_brake_cmd = 0.0

        for veh, thr, steer in (
            (self.ego_vehicle, self.impact_throttle, self.impact_steer),
            (self.target_vehicle, self.target_impact_throttle, self.target_impact_steer),
        ):
            if veh is not None:
                try:
                    veh.set_autopilot(False)
                    veh.apply_control(carla.VehicleControl(
                        throttle=float(thr),
                        steer=float(steer),
                        brake=0.0,
                        hand_brake=False,
                        manual_gear_shift=False,
                    ))
                except Exception:
                    pass

        print(
            f"[{frame}] ->> ПОСТ-УДАРНАЯ ФАЗА АКТИВИРОВАНА (EGO & TARGET): "
            f"Ego [T_react={self.ego_shock_duration_sec:.2f}c, T_hold={self.ego_spasm_duration_sec:.2f}c, "
            f"steer={self.impact_steer:+.3f}, thr={self.impact_throttle:.2f}] | "
            f"Target [T_react={self.target_shock_duration_sec:.2f}c, T_hold={self.target_spasm_duration_sec:.2f}c, "
            f"steer={self.target_impact_steer:+.3f}, thr={self.target_impact_throttle:.2f}]. "
            f"Режим после реакции: '{self.driver_post_shock}'."
        )

    # --------------------------------------------------------------------------
    # Обработчик столкновений (Collision Sensor Callback для обоих ТС)
    # --------------------------------------------------------------------------
    def on_collision(self, event, source_actor: str = 'ego'):
        """
        Универсальный обработчик столкновений для Ego и Target по ISO 26262-3:
        - Обнаруживает первичное боковое столкновение ТС-ТС.
        - Обнаруживает вторичный удар о жёсткое ограждение для Ego или Target.
        """
        other = event.other_actor
        frame = event.frame

        # Извлечение кинематики строго за 1 кадр до контакта (PhysX clean vectors)
        hist = self.speed_history.get(frame - 1)
        if not hist:
            hist = self.speed_history.get(frame - 2)
        if not hist:
            return

        ego_v = hist['ego_v']
        ego_right = hist['ego_right']
        ego_forward = hist['ego_forward']
        target_v = hist['target_v']
        target_forward = hist['target_forward']
        lane_forward = hist['lane_forward']
        lane_right = hist['lane_right']

        # ----------------------------------------------------------------------
        # 1. Первичный боковой удар ТС с ТС (SAE J2980 Table B1)
        # ----------------------------------------------------------------------
        if 'vehicle' in other.type_id.lower():
            if self.side_collision_frame is None:
                self.side_collision_frame = frame

                # Поперечная скорость сближения строго за 1 кадр до контакта (frame - 1)
                v_rel_x = ego_v.x - target_v.x
                v_rel_y = ego_v.y - target_v.y
                v_closing_lat = abs((v_rel_x * ego_right.x) + (v_rel_y * ego_right.y))

                side_metrics = compute_side_collision_metrics(
                    v_closing_lat=v_closing_lat,
                    m_ego=EGO_MASS,
                    m_target=self.target_mass,
                    restitution=RESTITUTION_E,
                )
                self.v_closing_lat = side_metrics.v_closing_lat
                self.delta_v_ego = side_metrics.delta_v_ego
                self.delta_v_target = side_metrics.delta_v_target
                self.effective_side_delta_v = side_metrics.effective_side_dv
                self.sev_side_val = side_metrics.sev_side

                print(
                    f"\n[{frame}] ->> СТОЛКНОВЕНИЕ ТС-ТС (SAE J2980, m_tgt={self.target_mass:.1f} кг): "
                    f"v_closing_lat = {self.v_closing_lat:.2f} м/с | "
                    f"Delta-V Ego = {self.delta_v_ego:.2f} м/с, "
                    f"Target = {self.delta_v_target:.2f} м/с | "
                    f"S_side = S{self.sev_side_val}"
                )

            self._activate_shock_phase(frame)

        # ----------------------------------------------------------------------
        # 2. Вторичный удар об ограждение (Подкласс А) для Ego или Target
        # ----------------------------------------------------------------------
        else:
            if self.subclass != SUBCLASS_A:
                return
            actor_veh = self.ego_vehicle if source_actor == 'ego' else self.target_vehicle
            if actor_veh is None:
                return
            try:
                loc_y = actor_veh.get_location().y
            except Exception:
                return

            # Ограждение (0.80 м) расположено на Y = 0.0 м. С учётом габаритов Impala (L=5.36 м,
            # полудиагональ ~2.86 м) отсекаем срабатывания при Y > 3.5 м (правая полоса / обочина).
            if loc_y > BARRIER_FILTER_MAX_Y:
                return

            # Кинематика участника за 1 кадр до удара (frame - 1)
            if source_actor == 'ego':
                v_actor = ego_v
                fw_actor = ego_forward
            else:
                v_actor = target_v
                fw_actor = target_forward

            barrier_metrics = compute_barrier_impact(
                v_actor_x=v_actor.x,
                v_actor_y=v_actor.y,
                fw_actor_x=fw_actor.x,
                fw_actor_y=fw_actor.y,
                lane_forward_x=lane_forward.x,
                lane_forward_y=lane_forward.y,
                lane_right_x=lane_right.x,
                lane_right_y=lane_right.y,
            )
            v_barrier = barrier_metrics.v_barrier
            cur_impact_type = barrier_metrics.impact_type
            cur_sev = barrier_metrics.sev_barrier

            is_new_actor = source_actor not in self.barrier_hit_actors
            self.barrier_hit_actors.add(source_actor)
            # Сохранение первичного или наихудшего удара
            is_first = (self.barrier_collision_frame is None)
            is_worse = (cur_sev > self.sev_barrier_val) or (
                cur_sev == self.sev_barrier_val and v_barrier > self.barrier_impact_speed
            )

            if is_first or is_worse:
                self.barrier_collision_frame = frame
                self.barrier_impact_speed = v_barrier
                self.barrier_impact_type = cur_impact_type
                self.sev_barrier_val = cur_sev
            if is_first or is_worse or is_new_actor:
                print(
                    f"\n[{frame}] ->> УДАР ОБ ОГРАЖДЕНИЕ ({source_actor.upper()}): "
                    f"Тип: {cur_impact_type} | "
                    f"Скорость удара = {v_barrier:.2f} м/с | "
                    f"S_barrier = S{cur_sev} (макс. по прогону: S{self.sev_barrier_val})"
                )

            self._activate_shock_phase(frame)

    # --------------------------------------------------------------------------
    # Покадровое логирование телеметрии обоих автомобилей
    # --------------------------------------------------------------------------
    def _record_telemetry_frame(self, snapshot, ego_snap, target_snap, lane_forward, lane_right):
        """Фиксация полной покадровой телеметрии Ego и Target в буфер памяти."""
        try:
            frame_data = extract_telemetry_frame(
                snapshot=snapshot,
                tick_count=self.tick_count,
                dt=self.dt,
                lane_change_triggered=self.lane_change_triggered,
                lost_control=self.lost_control,
                collision_tick=self.collision_tick,
                side_collision_frame=self.side_collision_frame,
                barrier_collision_frame=self.barrier_collision_frame,
                ego_snap=ego_snap,
                target_snap=target_snap,
                ego_vehicle=self.ego_vehicle,
                target_vehicle=self.target_vehicle,
                lane_forward=lane_forward,
                lane_right=lane_right,
            )
            self.frame_buffer.append(frame_data)
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # Метод граничных условий на бровке кювета Y = 9.75 м (Подклассы А и B)
    # --------------------------------------------------------------------------
    def check_ditch_boundary_conditions(self, actor_name: str, vehicle, lane_right, snapshot_frame: int):
        """
        Проверка пересечения бровки откоса кювета (Y >= 9.75 м) и фиксация граничных условий.
        Классификация тяжести S0-S3 строго на бровке без внутрикюветной оценки крена/ASI.
        """
        if vehicle is None or actor_name in self.ditch_metrics_by_actor:
            return

        loc = vehicle.get_location()

        # Фиксация происходит ровно в кадр пересечения бровки откоса кювета (Y >= 9.75 м)
        if loc.y >= DITCH_BOUNDARY_Y:
            tf = vehicle.get_transform()
            vel = vehicle.get_velocity()
            ang_vel = vehicle.get_angular_velocity()  # В CARLA z возвращается в градусах/сек

            # 1. Полная горизонтальная скорость вылета (м/с)
            v_dep = math.hypot(vel.x, vel.y)

            # 2. Нормальная скорость ухода в кювет перпендикулярно оси дороги
            v_perp_road = abs(vel.x * lane_right.x + vel.y * lane_right.y)

            # 3. Угол траектории вылета относительно дороги (Encroachment Angle, град)
            sin_theta = min(1.0, v_perp_road / max(v_dep, 0.1))
            theta_enc = math.degrees(math.asin(sin_theta))

            # 4. Кинематические проекции в локальных осях кузова автомобиля
            fw_vec = tf.get_forward_vector()
            rt_vec = tf.get_right_vector()
            v_lon_car = abs(vel.x * fw_vec.x + vel.y * fw_vec.y)
            v_lat_car = abs(vel.x * rt_vec.x + vel.y * rt_vec.y)

            # 5. Угол бокового заноса кузова (Sideslip Angle beta, град)
            beta = math.degrees(math.atan2(v_lat_car, max(v_lon_car, 0.01)))

            # 6. Угловая скорость рыскания (Yaw Rate omega_z, град/с)
            omega_z = abs(ang_vel.z)

            # 7. Классификация механизма и тяжести по чистой функции матрицы граничных условий
            actor_sev, mechanism = classify_ditch_boundary(
                v_dep=v_dep,
                v_perp_road=v_perp_road,
                v_lat_car=v_lat_car,
                beta=beta,
                omega_z=omega_z,
            )

            self.ditch_metrics_by_actor[actor_name] = {
                'v_dep': v_dep,
                'v_perp_road': v_perp_road,
                'theta_enc': theta_enc,
                'v_lat_car': v_lat_car,
                'beta': beta,
                'omega_z': omega_z,
                'mechanism': mechanism,
                'actor_sev': actor_sev,
                'frame': snapshot_frame,
                'y_loc': loc.y,
            }

            print(
                f"\n[{snapshot_frame}] ->> СХОД В КЮВЕТ ({actor_name.upper()} на бровке Y=9.75м): "
                f"V_dep={v_dep*3.6:.1f} км/ч ({v_dep:.2f} м/с) | "
                f"V_perp={v_perp_road:.2f} м/с (угол {theta_enc:.1f}°) | "
                f"V_lat={v_lat_car:.2f} м/с (занос beta={beta:.1f}°, omega_z={omega_z:.1f}°/с) | "
                f"Механизм: '{mechanism}' -> S{actor_sev}"
            )

    def _apply_post_impact_control(
        self,
        actor_name: str,
        vehicle,
        snap,
        vel,
        loc,
        impact_steer: float,
        impact_throttle: float,
        current_steer: float,
        current_brake: float,
        reaction_sec: float,
        hold_sec: float,
        time_since_impact: float,
    ) -> tuple[float, float]:
        """Применение пост-ударного управления к одному из ТС (Ego или Target)."""
        if vehicle is None:
            return current_steer, current_brake
        import carla

        in_ditch_or_barrier = (
            (loc.y >= DITCH_BOUNDARY_Y or actor_name in self.ditch_metrics_by_actor)
            or (loc.y <= BARRIER_HOLD_CUTOFF_Y or actor_name in self.barrier_hit_actors)
        )
        tf = snap.get_transform()
        yaw_deg = tf.rotation.yaw
        fw = tf.get_forward_vector()
        rt = tf.get_right_vector()
        v_lon_car = abs((vel.x * fw.x) + (vel.y * fw.y))
        v_lat_car = abs((vel.x * rt.x) + (vel.y * rt.y))
        speed_xy = math.hypot(vel.x, vel.y)
        beta_deg = math.degrees(math.atan2(v_lat_car, max(v_lon_car, 0.01))) if speed_xy >= 1.0 else 0.0

        state = PostImpactState(
            actor_name=actor_name,
            subclass=self.subclass,
            time_since_impact=time_since_impact,
            dt=self.dt,
            loc_y=loc.y,
            yaw_deg=yaw_deg,
            vy=vel.y,
            impact_steer=impact_steer,
            impact_throttle=impact_throttle,
            current_steer=current_steer,
            current_brake=current_brake,
            reaction_sec=reaction_sec,
            hold_sec=hold_sec,
            in_ditch_or_barrier=in_ditch_or_barrier,
            beta_deg=beta_deg,
        )
        cmd = self.driver_model.compute_command(state)
        vehicle.apply_control(carla.VehicleControl(
            throttle=float(cmd.throttle),
            steer=float(cmd.steer),
            brake=float(cmd.brake),
            hand_brake=False,
            manual_gear_shift=False,
        ))
        return cmd.steer, cmd.brake

    # --------------------------------------------------------------------------
    # Основной цикл симуляции (on_tick)
    # --------------------------------------------------------------------------
    def on_tick(self, snapshot):
        if not self.is_running:
            return
        try:
            import carla

            # ------------------------------------------------------------------
            # ФАЗА 0: Инициализация и конфигурация актеров при старте
            # ------------------------------------------------------------------
            if not self.configured:
                vehicles = self.world.get_actors().filter('vehicle.*')
                for v in vehicles:
                    role_name = v.attributes.get('role_name', '').lower()
                    type_str = v.type_id.lower()
                    if role_name == 'ego':
                        self.ego_vehicle = v
                    elif role_name == 'target':
                        self.target_vehicle = v
                    elif 'impala' in type_str and self.ego_vehicle is None:
                        self.ego_vehicle = v
                    elif 'leon' in type_str and self.target_vehicle is None:
                        self.target_vehicle = v

                if self.ego_vehicle is not None and self.target_vehicle is not None:
                    ego_loc = self.ego_vehicle.get_transform().location
                    target_loc = self.target_vehicle.get_transform().location

                    carla_map = self.world.get_map()
                    ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                    self.start_lane_id = ego_wp.lane_id

                    ego_fw = self.ego_vehicle.get_transform().get_forward_vector()
                    disp = target_loc - ego_loc
                    self.initial_lon_offset = (disp.x * ego_fw.x) + (disp.y * ego_fw.y)

                    target_wp = carla_map.get_waypoint(target_loc, project_to_road=True)
                    right_vec = target_wp.transform.get_right_vector()
                    disp_lat = target_loc - target_wp.transform.location
                    lateral_offset = (disp_lat.x * right_vec.x) + (disp_lat.y * right_vec.y)

                    # Настройка синхронного Traffic Manager для обоих ТС
                    self.ego_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.ego_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.ego_vehicle, 100)
                    self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    self.target_vehicle.set_autopilot(True, self.tm.get_port())
                    self.tm.auto_lane_change(self.target_vehicle, False)
                    self.tm.ignore_vehicles_percentage(self.target_vehicle, 100)
                    self.tm.vehicle_lane_offset(self.target_vehicle, float(lateral_offset))

                    # Назначение физических масс, Config F FrictionScale, EBD и sweep-коллайдеров колес
                    configure_ego_physics(self.ego_vehicle)
                    configure_target_physics(self.target_vehicle, self.target_mass)

                    # Инициализация ПИД-контроллера траектории для Ego (max_throttle=1.0 снимает лимит CARLA 0.75)
                    pid_cls = _get_vehicle_pid_controller_cls()
                    if pid_cls is not None:
                        args_lateral = {'K_P': 1.1, 'K_I': 0.03, 'K_D': 0.15, 'dt': self.dt}
                        args_long = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': self.dt}
                        self.pid_controller = pid_cls(
                            self.ego_vehicle,
                            args_lateral=args_lateral,
                            args_longitudinal=args_long,
                            max_throttle=PID_MAX_THROTTLE,
                        )

                    # Датчики коллизий на ОБОИХ автомобилях (ISO 26262-3)
                    bp_sensor = self.world.get_blueprint_library().find('sensor.other.collision')

                    self.ego_collision_sensor = self.world.spawn_actor(
                        bp_sensor,
                        carla.Transform(),
                        attach_to=self.ego_vehicle,
                    )
                    self.ego_collision_sensor.listen(lambda event: self.on_collision(event, source_actor='ego'))

                    self.target_collision_sensor = self.world.spawn_actor(
                        bp_sensor,
                        carla.Transform(),
                        attach_to=self.target_vehicle,
                    )
                    self.target_collision_sensor.listen(lambda event: self.on_collision(event, source_actor='target'))

                    # Зона пониженного FrictionScale (обочина + кювет) через static.trigger.friction
                    try:
                        self.friction_trigger = spawn_shoulder_friction_trigger(self.world, ego_loc.x)
                    except Exception as e:
                        print(f"[ПРЕДУПРЕЖДЕНИЕ] Не удалось заспавнить static.trigger.friction: {e}")
                        self.friction_trigger = None

                    self.configured = True
                    try:
                        self.spectator = self.world.get_spectator()
                    except Exception:
                        self.spectator = None

                    print(
                        f"[{snapshot.frame}] Актеры сконфигурированы: "
                        f"Ego ({self.ego_vehicle.type_id}, {EGO_MASS:.1f} кг, Lane {self.start_lane_id}) | "
                        f"Target ({self.target_vehicle.type_id}, {self.target_mass:.1f} кг) | "
                        f"Подкласс: {self.subclass} | "
                        f"Пост-ударный режим: {self.driver_post_shock}"
                    )

            # ------------------------------------------------------------------
            # ФАЗЫ 1 И 2 ПОСЛЕ КОНФИГУРАЦИИ
            # ------------------------------------------------------------------
            else:
                ego_snap = snapshot.find(self.ego_vehicle.id)
                target_snap = snapshot.find(self.target_vehicle.id)
                if not ego_snap or not target_snap:
                    return

                ego_v = ego_snap.get_velocity()
                target_v = target_snap.get_velocity()
                ego_mps = math.hypot(ego_v.x, ego_v.y)
                target_mps = math.hypot(target_v.x, target_v.y)

                ego_loc = ego_snap.get_transform().location
                target_loc = target_snap.get_transform().location
                ego_fw = ego_snap.get_transform().get_forward_vector()
                ego_right = ego_snap.get_transform().get_right_vector()

                target_fw = target_snap.get_transform().get_forward_vector()
                target_right = target_snap.get_transform().get_right_vector()

                carla_map = self.world.get_map()
                current_ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                lane_forward = current_ego_wp.transform.get_forward_vector()
                lane_right = current_ego_wp.transform.get_right_vector()

                # Кэширование пред-аварийной кинематики за 1 кадр до контакта для обоих ТС
                self.speed_history[snapshot.frame] = {
                    'ego_v': ego_v,
                    'ego_forward': ego_fw,
                    'ego_right': ego_right,
                    'target_v': target_v,
                    'target_forward': target_fw,
                    'target_right': target_right,
                    'lane_forward': lane_forward,
                    'lane_right': lane_right,
                }

                # Очистка старых кадров из словаря
                if len(self.speed_history) > 200:
                    old_keys = sorted(self.speed_history.keys())[:-100]
                    for k in old_keys:
                        del self.speed_history[k]

                # Камера зрителя следует за Ego
                if self.spectator is not None:
                    try:
                        cam_loc = ego_loc - (ego_fw * 12.0) + carla.Location(z=4.5)
                        cam_rot = ego_snap.get_transform().rotation
                        cam_rot.pitch -= 14.0
                        self.spectator.set_transform(carla.Transform(cam_loc, cam_rot))
                    except Exception:
                        pass

                self.tick_count += 1

                # --------------------------------------------------------------
                # ПОКАДРОВОЕ ЛОГИРОВАНИЕ ТЕЛЕМЕТРИИ (EGO + TARGET)
                # --------------------------------------------------------------
                if self.log_frames:
                    self._record_telemetry_frame(snapshot, ego_snap, target_snap, lane_forward, lane_right)

                # --------------------------------------------------------------
                # МОНИТОРИНГ БРОВКИ КЮВЕТА Y = 9.75 М ДЛЯ ОБОИХ ТС (Подклассы A и B)
                # --------------------------------------------------------------
                self.check_ditch_boundary_conditions('ego', self.ego_vehicle, lane_right, snapshot.frame)
                self.check_ditch_boundary_conditions('target', self.target_vehicle, lane_right, snapshot.frame)

                # Безопасное гашение скорости за пределами рельефа карты (Y > 28.0 м или Z < -2.0 м)
                for veh in [self.ego_vehicle, self.target_vehicle]:
                    if veh is not None:
                        try:
                            vloc = veh.get_location()
                            if vloc.y > OUT_OF_BOUNDS_Y or vloc.z < OUT_OF_BOUNDS_Z:
                                veh.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                                veh.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
                        except Exception:
                            pass

                # Кэширование актуальных педали газа и угла руля обоих ТС до момента столкновения
                if not self.lost_control:
                    try:
                        e_ctrl = self.ego_vehicle.get_control()
                        self.last_steer = e_ctrl.steer
                        self.last_throttle = e_ctrl.throttle
                    except Exception:
                        pass
                    try:
                        t_ctrl = self.target_vehicle.get_control()
                        self.last_target_steer = t_ctrl.steer
                        self.last_target_throttle = t_ctrl.throttle
                    except Exception:
                        pass

                # --------------------------------------------------------------
                # ФАЗА 1: Синхронный разгон и эластичная сцепка в слепой зоне
                # --------------------------------------------------------------
                if not self.lane_change_triggered:
                    disp = target_loc - ego_loc
                    current_lon_offset = (disp.x * ego_fw.x) + (disp.y * ego_fw.y)
                    error_x = current_lon_offset - self.initial_lon_offset

                    # Вывод мониторинга разгона с интервалом 1.0 с
                    log_interval_p1 = max(1, int(round(1.0 / self.dt)))
                    if self.tick_count % log_interval_p1 == 0:
                        sim_time = self.tick_count * self.dt
                        print(
                            f"[{sim_time:4.1f}c] Ego: {ego_mps*3.6:5.1f} км/ч | "
                            f"Target: {target_mps*3.6:5.1f} км/ч | Отклонение: {error_x:+.2f} м | "
                            f"Окно стабильности: {self.stability_time_sec:.2f}/1.00 c"
                        )

                    d_error = target_mps - ego_mps
                    self.target_err_integral += error_x * self.dt
                    self.target_err_integral = max(-10.0, min(self.target_err_integral, 10.0))

                    p_gain = 0.8
                    i_gain = 0.4
                    d_gain = 0.5

                    correction = -(error_x * p_gain + self.target_err_integral * i_gain + d_error * d_gain)
                    correction = max(-5.0, min(correction, 5.0))
                    target_desired_mps = max(5.0, min(ego_mps + correction + 0.4, 40.0))
                    self.tm.set_desired_speed(self.target_vehicle, float(target_desired_mps * 3.6))

                    # Периодическая подстройка круиз-контроля Ego раз в ~0.20 с
                    ego_corr_interval = max(1, int(round(0.20 / self.dt)))
                    if self.tick_count % ego_corr_interval == 0 and ego_mps > self.target_v0_mps * 0.5:
                        speed_err = self.target_v0_mps - ego_mps
                        if abs(speed_err) > 0.1:
                            self.ego_tm_speed += speed_err * 0.5
                            self.ego_tm_speed = max(10.0, min(self.ego_tm_speed, 28.8))
                            self.tm.set_desired_speed(self.ego_vehicle, float(self.ego_tm_speed * 3.6))

                    is_speed_ready = ego_mps >= (CRUISE_SPEED_MPS - 0.15)
                    is_speed_sync = abs(ego_mps - target_mps) <= 0.15
                    is_dist_sync = abs(error_x) <= 0.15

                    # Проверка условий стабильности по непрерывному физическому времени (1.0 с)
                    if is_speed_ready and is_speed_sync and is_dist_sync:
                        self.stability_time_sec += self.dt
                    else:
                        self.stability_time_sec = max(0.0, self.stability_time_sec - 2.0 * self.dt)

                    # Запасной триггер после 8.0 секунд стабильного движения
                    sim_time = self.tick_count * self.dt
                    forced_trigger = (sim_time >= 8.0 and ego_mps >= (CRUISE_SPEED_MPS - 0.25) and abs(error_x) <= 0.18)

                    if self.stability_time_sec >= 1.0 or forced_trigger:
                        self.lane_change_triggered = True
                        self.ego_vehicle.set_autopilot(False)
                        self.tm.set_desired_speed(self.target_vehicle, float(target_desired_mps * 3.6))
                        print(
                            f"\n[{snapshot.frame}] >>> МАНЕВР ИНИЦИИРОВАН ({self.subclass}): "
                            f"Ego начинает перестроение в слепую зону <<<"
                        )

                # --------------------------------------------------------------
                # ФАЗА 2: Опасное перестроение и пост-ударное управление
                # --------------------------------------------------------------
                else:
                    if not self.lost_control:
                        # До первого удара: ведение автомобиля Ego ПИД-контроллером в соседнюю полосу
                        ego_wp = carla_map.get_waypoint(ego_loc, project_to_road=True)
                        if ego_wp.lane_id == self.start_lane_id:
                            if self.subclass == SUBCLASS_A:
                                target_lane_wp = ego_wp.get_left_lane()
                            else:
                                target_lane_wp = ego_wp.get_right_lane()
                        else:
                            target_lane_wp = ego_wp

                        if target_lane_wp is not None:
                            target_wps = target_lane_wp.next(25.0)
                            target_wp = target_wps[0] if target_wps else target_lane_wp
                            if self.pid_controller is not None:
                                control = self.pid_controller.run_step(self.target_v0_mps * 3.6, target_wp)
                                self.last_steer = control.steer
                                self.last_throttle = control.throttle
                                self.ego_vehicle.apply_control(control)

                        # Правило Safe Pass: через 1.0 с после завершения перестроения без удара
                        # отключаем слепой режим Target, чтобы исключить догоняющие «фантомные S0»
                        if not self.safe_pass_confirmed:
                            ego_yaw_deg = ego_snap.get_transform().rotation.yaw
                            if is_ego_settled_in_target_lane(
                                subclass=self.subclass,
                                ego_lane_id=ego_wp.lane_id,
                                start_lane_id=self.start_lane_id,
                                ego_y=ego_loc.y,
                                ego_vy=ego_v.y,
                                ego_yaw_deg=ego_yaw_deg,
                            ):
                                self.post_lane_change_safe_time_sec += self.dt
                                if self.post_lane_change_safe_time_sec >= SAFE_PASS_HOLD_SEC - 1e-9:
                                    self.safe_pass_confirmed = True
                                    if self.tm is not None and self.target_vehicle is not None:
                                        try:
                                            self.tm.ignore_vehicles_percentage(self.target_vehicle, 0)
                                        except Exception:
                                            pass
                                    print(
                                        f"\n[{snapshot.frame}] ->> SAFE PASS ПОДТВЕРЖДЁН: "
                                        f"Ego завершил перестроение без столкновения (+{SAFE_PASS_HOLD_SEC:.1f} с); "
                                        f"слепой режим Target отключён (ignore_vehicles=0%)."
                                    )
                            else:
                                self.post_lane_change_safe_time_sec = 0.0

                    else:
                        # ПОСЛЕ УДАРА: единый пост-ударный контроллер для Ego и Target
                        ticks_since_impact = self.tick_count - self.collision_tick
                        time_since_impact = ticks_since_impact * self.dt

                        self.current_steer, self.ego_brake_cmd = self._apply_post_impact_control(
                            actor_name='ego',
                            vehicle=self.ego_vehicle,
                            snap=ego_snap,
                            vel=ego_v,
                            loc=ego_loc,
                            impact_steer=self.impact_steer,
                            impact_throttle=self.impact_throttle,
                            current_steer=self.current_steer,
                            current_brake=self.ego_brake_cmd,
                            reaction_sec=self.ego_shock_duration_sec,
                            hold_sec=self.ego_spasm_duration_sec,
                            time_since_impact=time_since_impact,
                        )

                        self.target_current_steer, self.target_brake_cmd = self._apply_post_impact_control(
                            actor_name='target',
                            vehicle=self.target_vehicle,
                            snap=target_snap,
                            vel=target_v,
                            loc=target_loc,
                            impact_steer=self.target_impact_steer,
                            impact_throttle=self.target_impact_throttle,
                            current_steer=self.target_current_steer,
                            current_brake=self.target_brake_cmd,
                            reaction_sec=self.target_shock_duration_sec,
                            hold_sec=self.target_spasm_duration_sec,
                            time_since_impact=time_since_impact,
                        )

                    # Периодический мониторинг маневра в консоль с интервалом 0.5 с
                    log_interval_p2 = max(1, int(round(0.5 / self.dt)))
                    if self.tick_count % log_interval_p2 == 0:
                        cur_steer = self.current_steer if self.lost_control else self.last_steer
                        sim_time = self.tick_count * self.dt
                        print(
                            f"[{sim_time:4.1f}c] МАНЕВР: Ego Y={ego_loc.y:5.2f} (v={ego_mps*3.6:5.1f} км/ч) "
                            f"| Target Y={target_loc.y:5.2f} (v={target_mps*3.6:5.1f} км/ч) | "
                            f"dY={target_loc.y - ego_loc.y:+5.2f} | Руль Ego: {cur_steer:+.3f} | Удар: {self.lost_control}"
                        )

        except Exception:
            traceback.print_exc()

    # --------------------------------------------------------------------------
    # Извлечение итоговых метрик по кювету и барьеру
    # --------------------------------------------------------------------------
    def compute_ditch_metrics(self):
        """
        Возвращает метрики схода в кювет по зафиксированным граничным условиям на бровке Y=9.75 м.
        Если съехали оба ТС, выбирается ТС с наихудшим классом тяжести (или большей скоростью вылета).
        """
        return select_worst_ditch_metrics(self.ditch_metrics_by_actor)

    def get_barrier_collision_actor(self) -> str:
        """Возвращает строку участников удара в отбойник ('ego', 'target', 'both', или 'None')."""
        if 'ego' in self.barrier_hit_actors and 'target' in self.barrier_hit_actors:
            return 'both'
        elif 'ego' in self.barrier_hit_actors:
            return 'ego'
        elif 'target' in self.barrier_hit_actors:
            return 'target'
        else:
            return 'None'

    def start(self):
        self.is_running = True
        self.callback_id = self.world.on_tick(self.on_tick)

    def stop(self):
        self.is_running = False
        self.spectator = None
        if self.world is not None and self.callback_id is not None:
            try:
                self.world.remove_on_tick(self.callback_id)
            except Exception:
                pass
            self.callback_id = None

        # Обязательное уничтожение обоих датчиков столкновений
        if self.ego_collision_sensor is not None:
            try:
                if self.ego_collision_sensor.is_alive:
                    self.ego_collision_sensor.destroy()
            except Exception:
                pass
            self.ego_collision_sensor = None

        if self.target_collision_sensor is not None:
            try:
                if self.target_collision_sensor.is_alive:
                    self.target_collision_sensor.destroy()
            except Exception:
                pass
            self.target_collision_sensor = None

        # Обязательное уничтожение физического триггера трения обочины
        if self.friction_trigger is not None:
            try:
                if self.friction_trigger.is_alive:
                    self.friction_trigger.destroy()
            except Exception:
                pass
            self.friction_trigger = None

        if self.tm is not None:
            try:
                self.tm.set_synchronous_mode(False)
            except Exception:
                pass
