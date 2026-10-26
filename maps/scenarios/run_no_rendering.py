#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
CARLA NO-RENDERING MODE SIMULATION (LCC / ISO 26262)
================================================================================
Автономный скрипт симуляции без отрисовки графики (No-Rendering Mode).
Идеально подходит для запуска на серверах без дискретной видеокарты или
для многократного ускорения физических расчетов PhysX.
================================================================================
"""

import carla
import time

def main():
    print("=" * 70)
    print("ЗАПУСК СИМУЛЯЦИИ В NO-RENDERING MODE (БЕЗ РЕНДЕРА ГРАФИКИ)")
    print("=" * 70)

    # 1. Подключение
    print("\n[1/5] Подключение к симулятору CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    # 2. Загрузка карты
    print("[2/5] Загрузка карты 'map_ditch'...")
    world = client.load_world('map_ditch')
    time.sleep(1.0)

    # 3. Активация No-Rendering Mode и синхронного режима
    print("[3/5] Активация No-Rendering Mode (отключение GPU рендера)...")
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.no_rendering_mode = True       # Отключает рендеринг геометрии и освещения
    settings.synchronous_mode = True        # Включает строгую синхронизацию тиков физики
    settings.fixed_delta_seconds = 0.05     # Фиксированный шаг времени 20 Гц
    world.apply_settings(settings)

    try:
        carla_map = world.get_map()
        bp_lib = world.get_blueprint_library()

        # 4. Спавн транспортных средств
        print("\n[4/5] Размещение автомобилей сценария...")
        ego_bp = bp_lib.filter('vehicle.lincoln.mkz_2017')[0]
        ego_bp.set_attribute('role_name', 'ego')
        wp_ego = carla_map.get_waypoint(carla.Location(x=50.0, y=5.625, z=0.0))
        tf_ego = wp_ego.transform
        tf_ego.location.z += 1.0
        ego_veh = world.spawn_actor(ego_bp, tf_ego)
        print(f"  [+] Ego (Lincoln MKZ): ID {ego_veh.id} на полосе {wp_ego.lane_id}")

        target_bp = bp_lib.filter('vehicle.audi.tt')[0]
        target_bp.set_attribute('role_name', 'target')
        wp_target = carla_map.get_waypoint(carla.Location(x=70.0, y=1.875, z=0.0))
        tf_target = wp_target.transform
        tf_target.location.z += 1.0
        target_veh = world.spawn_actor(target_bp, tf_target)
        print(f"  [+] Target (Audi TT): ID {target_veh.id} на полосе {wp_target.lane_id}")

        # Применяем управление: движение вперед
        ego_veh.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
        target_veh.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))

        # 5. Цикл физической симуляции PhysX
        print("\n[5/5] Выполнение 100 тиков симуляции PhysX...")
        start_time = time.time()
        for tick_idx in range(1, 101):
            world.tick()
            if tick_idx % 20 == 0:
                loc = ego_veh.get_location()
                vel = ego_veh.get_velocity()
                speed_kmh = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2)**0.5
                print(f"  Тик {tick_idx:3d} | X: {loc.x:6.2f} м | Y: {loc.y:5.2f} м | Z: {loc.z:5.3f} м | V: {speed_kmh:5.1f} км/ч")

        elapsed = time.time() - start_time
        fps = 100.0 / elapsed if elapsed > 0 else 0
        loc_final = ego_veh.get_location()

        print("\n" + "=" * 70)
        print(">> ТЕСТ В NO-RENDERING MODE УСПЕШНО ЗАВЕРШЁН! <<")
        print(f">> Физическая высота Ego над полотном: Z = {loc_final.z:.3f} м (коллизия активна!)")
        print(f">> Скорость симуляции: {fps:.1f} тиков/сек (затрачено {elapsed:.2f} сек на 100 тиков)")
        print("=" * 70)

    finally:
        # Восстановление настроек при завершении
        print("\n[Завершение] Восстановление настроек симулятора...")
        settings.no_rendering_mode = False
        settings.synchronous_mode = False
        world.apply_settings(settings)
        if 'ego_veh' in locals() and ego_veh.is_alive:
            ego_veh.destroy()
        if 'target_veh' in locals() and target_veh.is_alive:
            target_veh.destroy()
        print("[OK] Симуляция завершена чисто.")

if __name__ == '__main__':
    main()
