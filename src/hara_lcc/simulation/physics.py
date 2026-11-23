"""
Конфигурация физики мира и транспортных средств в CARLA / PhysX (Config F):
- PhysX substepping (fixed_delta_seconds, max_substep_delta_time = 0.01 с, max_substeps = 10);
- VehiclePhysicsControl: массы, ASPHALT_TIRE_FRICTION_SCALE, EBD max_brake_torque, use_sweep_wheel_collision = True;
- Триггер сцепления обочины/кювета static.trigger.friction (зона колёс Y >= 7.50 м до 32.0 м).
"""

from __future__ import annotations

from hara_lcc.config import (
    ASPHALT_TIRE_FRICTION_SCALE,
    EGO_FRONT_BRAKE_TORQUE_NM,
    EGO_MASS,
    EGO_REAR_BRAKE_TORQUE_NM,
    FRICTION_TRIGGER_CENTER_Y,
    FRICTION_TRIGGER_EXTENT_X_CM,
    FRICTION_TRIGGER_EXTENT_Y_CM,
    FRICTION_TRIGGER_EXTENT_Z_CM,
    FRICTION_TRIGGER_X_OFFSET_M,
    GRAVEL_TIRE_FRICTION_SCALE,
    PHYSX_MAX_SUBSTEPS,
    PHYSX_MAX_SUBSTEP_DT,
    TARGET_FRONT_BRAKE_NM_PER_KG,
    TARGET_REAR_BRAKE_NM_PER_KG,
    USE_SWEEP_WHEEL_COLLISION,
)


def configure_world_physx(world, timestep: float):
    """Настройка синхронного режима и PhysX substepping для стабильной контактной динамики."""
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(timestep)
    settings.substepping = True
    settings.max_substep_delta_time = PHYSX_MAX_SUBSTEP_DT
    settings.max_substeps = PHYSX_MAX_SUBSTEPS
    world.apply_settings(settings)
    return settings


def configure_vehicle_physics(
    vehicle,
    mass_kg: float,
    tire_friction_scale: float = ASPHALT_TIRE_FRICTION_SCALE,
    front_brake_torque_nm: float = EGO_FRONT_BRAKE_TORQUE_NM,
    rear_brake_torque_nm: float = EGO_REAR_BRAKE_TORQUE_NM,
    use_sweep_wheel_collision: bool = USE_SWEEP_WHEEL_COLLISION,
):
    """Назначение VehiclePhysicsControl (Config F) на автомобиль в Фазе 0 до начала тиков."""
    phys = vehicle.get_physics_control()
    phys.mass = float(mass_kg)
    phys.use_sweep_wheel_collision = bool(use_sweep_wheel_collision)
    wheels = phys.wheels
    for idx, w in enumerate(wheels):
        w.tire_friction = float(tire_friction_scale)
        w.max_brake_torque = float(front_brake_torque_nm if idx < 2 else rear_brake_torque_nm)
    phys.wheels = wheels
    vehicle.apply_physics_control(phys)
    return phys


def configure_ego_physics(vehicle):
    """Применение параметров Config F к защищаемому ТС Ego (Chevrolet Impala, 3490 кг)."""
    return configure_vehicle_physics(
        vehicle=vehicle,
        mass_kg=EGO_MASS,
        tire_friction_scale=ASPHALT_TIRE_FRICTION_SCALE,
        front_brake_torque_nm=EGO_FRONT_BRAKE_TORQUE_NM,
        rear_brake_torque_nm=EGO_REAR_BRAKE_TORQUE_NM,
        use_sweep_wheel_collision=USE_SWEEP_WHEEL_COLLISION,
    )


def configure_target_physics(vehicle, target_mass_kg: float):
    """Применение параметров Config F к целевому ТС Target (Seat Leon, пропорциональный EBD)."""
    mass = float(target_mass_kg)
    return configure_vehicle_physics(
        vehicle=vehicle,
        mass_kg=mass,
        tire_friction_scale=ASPHALT_TIRE_FRICTION_SCALE,
        front_brake_torque_nm=TARGET_FRONT_BRAKE_NM_PER_KG * mass,
        rear_brake_torque_nm=TARGET_REAR_BRAKE_NM_PER_KG * mass,
        use_sweep_wheel_collision=USE_SWEEP_WHEEL_COLLISION,
    )


def spawn_shoulder_friction_trigger(
    world,
    ego_x: float,
    friction_scale: float = GRAVEL_TIRE_FRICTION_SCALE,
):
    """
    Спавн зоны пониженного FrictionScale (обочина + кювет) через static.trigger.friction.
    Левая граница бокса сдвинута до Y = 8.50 м (компенсация ~1.0 м полуширины кузова
    для срабатывания при реальном выходе колес за Y >= 7.50 м без срабатывания на асфальте);
    правая граница расширена до Y = 32.00 м (без ложного EndOverlap в кювете до Y = 30.0 м).
    Центр Y = 20.25 м, полуширина 11.75 м (extent_y = 1175.0 см).
    """
    import carla

    bp_friction = world.get_blueprint_library().find('static.trigger.friction')
    bp_friction.set_attribute('friction', str(friction_scale))
    bp_friction.set_attribute('extent_x', str(FRICTION_TRIGGER_EXTENT_X_CM))
    bp_friction.set_attribute('extent_y', str(FRICTION_TRIGGER_EXTENT_Y_CM))
    bp_friction.set_attribute('extent_z', str(FRICTION_TRIGGER_EXTENT_Z_CM))
    friction_transform = carla.Transform(
        carla.Location(x=float(ego_x + FRICTION_TRIGGER_X_OFFSET_M), y=FRICTION_TRIGGER_CENTER_Y, z=0.0)
    )
    return world.spawn_actor(bp_friction, friction_transform)
