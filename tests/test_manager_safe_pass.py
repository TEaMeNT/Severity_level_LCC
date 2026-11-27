"""Оффлайн-тесты Фазы 2 AutopilotManager: max_throttle=1.0, сохранение скорости Target, Safe Pass (1.0 с) и анти-спам барьера."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from hara_lcc.config import (
    CRUISE_SPEED_MPS,
    LANE_LEFT_CENTER_Y,
    LANE_RIGHT_CENTER_Y,
    PID_MAX_THROTTLE,
    SAFE_PASS_HOLD_SEC,
    SUBCLASS_A,
    SUBCLASS_B,
)
from hara_lcc.simulation import manager as manager_mod
from hara_lcc.simulation.manager import AutopilotManager, is_ego_settled_in_target_lane


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __sub__(self, other):
        return _Vec(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return _Vec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, scalar: float):
        return _Vec(self.x * scalar, self.y * scalar, self.z * scalar)


class _Rot:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = float(pitch), float(yaw), float(roll)


class _Transform:
    def __init__(self, loc: _Vec, rot: _Rot | None = None):
        self.location = loc
        self.rotation = rot if rot is not None else _Rot()

    def get_forward_vector(self):
        return _Vec(1.0, 0.0, 0.0)

    def get_right_vector(self):
        return _Vec(0.0, 1.0, 0.0)


class _MockTM:
    def __init__(self):
        self.ignore_calls: list[tuple[int, int]] = []
        self.desired_speeds: list[tuple[int, float]] = []

    def set_synchronous_mode(self, enabled: bool):
        pass

    def get_port(self):
        return 8000

    def auto_lane_change(self, veh, enabled: bool):
        pass

    def ignore_vehicles_percentage(self, veh, pct: int):
        self.ignore_calls.append((veh.id, pct))

    def set_desired_speed(self, veh, speed_kmh: float):
        self.desired_speeds.append((veh.id, speed_kmh))

    def vehicle_lane_offset(self, veh, offset: float):
        pass


class _MockClient:
    def __init__(self, tm: _MockTM):
        self._tm = tm

    def get_trafficmanager(self, port: int):
        return self._tm


class _MockVehicle:
    def __init__(self, vid: int, role: str, y: float):
        self.id = vid
        self.type_id = 'vehicle.chevrolet.impala' if role == 'ego' else 'vehicle.seat.leon'
        self.attributes = {'role_name': role}
        self._loc = _Vec(100.0, y, 0.2)
        self._rot = _Rot(0.0, 0.0, 0.0)
        self._vel = _Vec(CRUISE_SPEED_MPS, 0.0, 0.0)
        self._ctrl = SimpleNamespace(steer=0.0, throttle=0.60, brake=0.0)
        self.autopilot_calls: list[bool] = []
        self.applied_controls: list = []

    def get_transform(self):
        return _Transform(self._loc, self._rot)

    def get_location(self):
        return self._loc

    def get_velocity(self):
        return self._vel

    def get_control(self):
        return self._ctrl

    def set_autopilot(self, enabled: bool, *args):
        self.autopilot_calls.append(enabled)

    def apply_control(self, ctrl):
        self.applied_controls.append(ctrl)
        self._ctrl = ctrl


class _MockWaypoint:
    def __init__(self, lane_id: int, y: float):
        self.lane_id = lane_id
        self.transform = _Transform(_Vec(100.0, y, 0.0))

    def get_left_lane(self):
        return _MockWaypoint(-1, LANE_LEFT_CENTER_Y)

    def get_right_lane(self):
        return _MockWaypoint(-2, LANE_RIGHT_CENTER_Y)

    def next(self, dist: float):
        return [self]


class _MockMap:
    def get_waypoint(self, loc: _Vec, project_to_road: bool = True):
        if loc.y < 3.75:
            return _MockWaypoint(-1, LANE_LEFT_CENTER_Y)
        return _MockWaypoint(-2, LANE_RIGHT_CENTER_Y)


class _MockActorSnap:
    def __init__(self, veh: _MockVehicle):
        self._veh = veh

    def get_transform(self):
        return self._veh.get_transform()

    def get_velocity(self):
        return self._veh.get_velocity()


class _MockSnapshot:
    def __init__(self, frame: int, actors: dict[int, _MockVehicle]):
        self.frame = frame
        self._actors = actors

    def find(self, vid: int):
        v = self._actors.get(vid)
        return _MockActorSnap(v) if v is not None else None


class _MockWorld:
    def __init__(self, ego: _MockVehicle, target: _MockVehicle):
        self._ego = ego
        self._target = target
        self._map = _MockMap()

    def get_map(self):
        return self._map

    def get_actors(self):
        return SimpleNamespace(filter=lambda _: [self._ego, self._target])

    def get_blueprint_library(self):
        return SimpleNamespace(find=lambda _: SimpleNamespace())

    def spawn_actor(self, bp, tf, attach_to=None):
        return SimpleNamespace(listen=lambda cb: None, is_alive=True, destroy=lambda: None)

    def get_spectator(self):
        return None


def _install_fake_carla(monkeypatch):
    fake_carla = SimpleNamespace(
        Transform=lambda *a, **kw: SimpleNamespace(),
        Location=lambda x=0.0, y=0.0, z=0.0: _Vec(x, y, z),
        Vector3D=lambda x=0.0, y=0.0, z=0.0: _Vec(x, y, z),
        VehicleControl=lambda **kw: SimpleNamespace(**kw),
    )
    monkeypatch.setitem(sys.modules, 'carla', fake_carla)


def test_is_ego_settled_in_target_lane_helper():
    # SubA: цель — левая полоса Y=1.875 (lane_id=-1, start=-2)
    assert is_ego_settled_in_target_lane(SUBCLASS_A, -1, -2, 1.65, -0.30, 0.6) is True
    assert is_ego_settled_in_target_lane(SUBCLASS_A, -2, -2, 1.88, 0.0, 0.0) is False  # ещё в старой полосе
    assert is_ego_settled_in_target_lane(SUBCLASS_A, -1, -2, 2.30, 0.0, 0.0) is False  # > 0.35 м от центра
    assert is_ego_settled_in_target_lane(SUBCLASS_A, -1, -2, 1.88, -0.50, 0.0) is False  # |vy| > 0.35 м/с
    assert is_ego_settled_in_target_lane(SUBCLASS_A, -1, -2, 1.88, 0.0, 3.0) is False   # |yaw| > 2.5°

    # SubB: цель — правая полоса Y=5.625 (lane_id=-2, start=-1)
    assert is_ego_settled_in_target_lane(SUBCLASS_B, -2, -1, 5.90, 0.10, -0.7) is True
    assert is_ego_settled_in_target_lane(SUBCLASS_B, -1, -1, 5.62, 0.0, 0.0) is False


def test_phase0_passes_max_throttle_1_0_to_pid(monkeypatch):
    _install_fake_carla(monkeypatch)
    captured_kwargs = {}

    class _FakePID:
        def __init__(self, vehicle, args_lateral, args_longitudinal, **kwargs):
            captured_kwargs.update(kwargs)

        def run_step(self, target_speed, waypoint):
            return SimpleNamespace(steer=0.0, throttle=0.85, brake=0.0)

    monkeypatch.setattr(manager_mod, "_get_vehicle_pid_controller_cls", lambda: _FakePID)
    monkeypatch.setattr(manager_mod, "configure_ego_physics", lambda v: None)
    monkeypatch.setattr(manager_mod, "configure_target_physics", lambda v, m: None)
    monkeypatch.setattr(manager_mod, "spawn_shoulder_friction_trigger", lambda w, x: None)

    tm = _MockTM()
    ego = _MockVehicle(1, 'ego', LANE_RIGHT_CENTER_Y)
    target = _MockVehicle(2, 'target', LANE_LEFT_CENTER_Y)
    world = _MockWorld(ego, target)

    mgr = AutopilotManager(_MockClient(tm), world, subclass=SUBCLASS_A, dt=0.01)
    mgr.is_running = True
    mgr.on_tick(_MockSnapshot(1, {1: ego, 2: target}))

    assert mgr.configured is True
    assert captured_kwargs.get("max_throttle") == PID_MAX_THROTTLE == 1.0


@pytest.mark.parametrize("subclass, start_lane, settled_y", [
    (SUBCLASS_A, -2, LANE_LEFT_CENTER_Y + 0.20),
    (SUBCLASS_B, -1, LANE_RIGHT_CENTER_Y + 0.20),
])
def test_safe_pass_triggers_after_1s_and_disables_target_blind_mode(monkeypatch, subclass, start_lane, settled_y):
    _install_fake_carla(monkeypatch)
    tm = _MockTM()
    ego = _MockVehicle(1, 'ego', settled_y)
    target = _MockVehicle(2, 'target', settled_y)
    world = _MockWorld(ego, target)

    mgr = AutopilotManager(_MockClient(tm), world, subclass=subclass, dt=0.01)
    mgr.is_running = True
    mgr.configured = True
    mgr.lane_change_triggered = True
    mgr.start_lane_id = start_lane
    mgr.ego_vehicle = ego
    mgr.target_vehicle = target
    mgr.pid_controller = SimpleNamespace(
        run_step=lambda spd, wp: SimpleNamespace(steer=0.0, throttle=0.60, brake=0.0)
    )

    ticks_for_1s = int(round(SAFE_PASS_HOLD_SEC / mgr.dt))
    # Первые 99 тиков (0.99 с) Safe Pass ещё НЕ подтверждён
    for frame in range(1, ticks_for_1s):
        mgr.on_tick(_MockSnapshot(frame, {1: ego, 2: target}))
    assert mgr.safe_pass_confirmed is False
    assert tm.ignore_calls == []

    # На 100-м тике (1.00 с) Safe Pass подтверждается и вызывается ignore_vehicles_percentage(target, 0)
    mgr.on_tick(_MockSnapshot(ticks_for_1s, {1: ego, 2: target}))
    assert mgr.safe_pass_confirmed is True
    assert tm.ignore_calls == [(target.id, 0)]

    # Повторные тики не дублируют вызов TM
    for frame in range(ticks_for_1s + 1, ticks_for_1s + 20):
        mgr.on_tick(_MockSnapshot(frame, {1: ego, 2: target}))
    assert tm.ignore_calls == [(target.id, 0)]


def test_safe_pass_does_not_trigger_after_collision(monkeypatch):
    _install_fake_carla(monkeypatch)
    tm = _MockTM()
    ego = _MockVehicle(1, 'ego', LANE_LEFT_CENTER_Y)
    target = _MockVehicle(2, 'target', LANE_LEFT_CENTER_Y)
    world = _MockWorld(ego, target)

    mgr = AutopilotManager(_MockClient(tm), world, subclass=SUBCLASS_A, dt=0.01)
    mgr.is_running = True
    mgr.configured = True
    mgr.lane_change_triggered = True
    mgr.lost_control = True
    mgr.collision_tick = 1
    mgr.start_lane_id = -2
    mgr.ego_vehicle = ego
    mgr.target_vehicle = target

    for frame in range(1, 150):
        mgr.on_tick(_MockSnapshot(frame, {1: ego, 2: target}))

    assert mgr.safe_pass_confirmed is False
    assert tm.ignore_calls == []


def test_barrier_console_spam_guard_preserved(monkeypatch, capsys):
    _install_fake_carla(monkeypatch)
    tm = _MockTM()
    ego = _MockVehicle(1, 'ego', 0.8)
    target = _MockVehicle(2, 'target', 0.9)
    world = _MockWorld(ego, target)

    mgr = AutopilotManager(_MockClient(tm), world, subclass=SUBCLASS_A, dt=0.01)
    mgr.ego_vehicle = ego
    mgr.target_vehicle = target
    mgr.lost_control = True
    mgr.collision_tick = 1

    # Заполняем предкрашевый буфер: удар target 4.0 м/с, затем два более слабых контакта 2.5 м/с
    for fr, vy in [(10, -4.0), (11, -2.5), (12, -2.5)]:
        mgr.speed_history[fr - 1] = {
            'ego_v': _Vec(25.0, -1.0, 0.0),
            'ego_forward': _Vec(1.0, 0.0, 0.0),
            'ego_right': _Vec(0.0, 1.0, 0.0),
            'target_v': _Vec(25.0, vy, 0.0),
            'target_forward': _Vec(1.0, 0.0, 0.0),
            'lane_forward': _Vec(1.0, 0.0, 0.0),
            'lane_right': _Vec(0.0, 1.0, 0.0),
        }
        ev = SimpleNamespace(frame=fr, other_actor=SimpleNamespace(type_id='static.guardrail'))
        mgr.on_collision(ev, source_actor='target')

    out = capsys.readouterr().out
    # Печать должна произойти ровно 1 раз (только на первом контакте, без спама на кадрах 11 и 12)
    assert out.count("УДАР ОБ ОГРАЖДЕНИЕ") == 1
