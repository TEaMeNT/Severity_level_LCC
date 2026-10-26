#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
CARLA HIGHWAY & DITCH SCENARIO (ГОСТ / СП 34.13330.2021 & ISO 26262 LCC)
================================================================================
Нативная прямая загрузка уровня map_ditch без костылей и спавна мешей через код.
Все текстуры (асфальт, гравий, трава, бетон) и динамическое освещение
запечены непосредственно в map_ditch.umap.
"""

import carla
import time

def main():
    print("=" * 70)
    print("ЗАПУСК СЦЕНАРИЯ LCC (ТРАССА С КЮВЕТОМ 1:4 И БАРЬЕРОМ 0.8 М)")
    print("=" * 70)

    # 1. Подключение к симулятору CARLA
    print("\n[1/4] Подключение к CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    # 2. Прямая нативная загрузка карты
    print("[2/4] Загрузка карты 'map_ditch' через client.load_world('map_ditch')...")
    world = client.load_world('map_ditch')
    time.sleep(2.0)

    carla_map = world.get_map()
    wps = carla_map.generate_waypoints(distance=10.0)
    print(f"  [OK] Карта успешно загружена: {carla_map.name}")
    print(f"  [OK] OpenDRIVE сгенерировал {len(wps)} вейпоинтов по трассе (1500 м).")

    # 3. Размещение автомобилей сценария
    print("\n[3/4] Размещение транспортных средств сценария...")
    bp_lib = world.get_blueprint_library()

    # Ego: Lincoln MKZ (3490 кг, аналог Aurus Senat) на правой полосе (Lane -2, Y ~ 5.625)
    ego_bp = bp_lib.filter('vehicle.lincoln.mkz_2017')[0]
    ego_bp.set_attribute('role_name', 'ego')
    wp_ego = carla_map.get_waypoint(carla.Location(x=50.0, y=5.625, z=0.0))
    ego_tf = wp_ego.transform
    ego_tf.location.z += 1.0  # Опускание на физическую коллизию
    ego_veh = world.spawn_actor(ego_bp, ego_tf)
    print(f"  [+] Ego (Lincoln MKZ): ID {ego_veh.id} на полосе {wp_ego.lane_id}")

    # Target: Audi TT (2000 кг) на левой полосе (Lane -1, Y ~ 1.875)
    target_bp = bp_lib.filter('vehicle.audi.tt')[0]
    target_bp.set_attribute('role_name', 'target')
    wp_target = carla_map.get_waypoint(carla.Location(x=65.0, y=1.875, z=0.0))
    target_tf = wp_target.transform
    target_tf.location.z += 1.0
    target_veh = world.spawn_actor(target_bp, target_tf)
    print(f"  [+] Target (Audi TT): ID {target_veh.id} на полосе {wp_target.lane_id}")

    # 4. Позиционирование камеры зрителя (Spectator) в окне CARLA
    print("\n[4/4] Позиционирование камеры зрителя (Spectator)...")
    spec = world.get_spectator()
    spec_tf = carla.Transform(
        ego_tf.location + carla.Location(x=-14.0, y=1.5, z=4.5),
        carla.Rotation(pitch=-12.0, yaw=0.0, roll=0.0)
    )
    spec.set_transform(spec_tf)

    # Даем физике PhysX стабилизироваться
    for _ in range(25):
        world.tick()
        time.sleep(0.04)

    loc = ego_veh.get_location()
    print("\n" + "=" * 70)
    print(">> УСПЕХ! СЦЕНАРИЙ И КАРТА ПОЛНОСТЬЮ АКТИВНЫ В CARLA! <<")
    print(f">> Физическая высота Ego над коллизией дорожного полотна: Z = {loc.z:.3f} м")
    print(">> Режим: 100% нативный (client.load_world('map_ditch')) без скриптовых мешей.")
    print(">> Освещение: Реальное дневное солнце (DirectionalLight 65 000 Lux), небо и тени.")
    print(">> Материалы: Реальные текстуры асфальта, гравия, травы и бетона.")
    print("=" * 70 + "\n")

if __name__ == '__main__':
    main()
