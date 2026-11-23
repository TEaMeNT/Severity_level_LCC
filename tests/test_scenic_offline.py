"""Оффлайн-тесты генерации сцен Scenic (N >= 500 на подкласс, 0 коллизий спавна, валидные границы)."""

from __future__ import annotations

import random

import pytest
import scenic

from hara_lcc.config import SCENIC_PATH, SUBCLASS_A, SUBCLASS_B
from hara_lcc.simulation.assets import resolve_map_assets

EGO_LENGTH = 5.36
EGO_WIDTH = 2.03
TARGET_LENGTH = 4.19
TARGET_WIDTH = 1.82
SCENES_PER_SUBCLASS = 500


@pytest.mark.parametrize("subclass", [SUBCLASS_A, SUBCLASS_B])
def test_scenic_offline_scene_generation_500_per_subclass(subclass: str):
    _, map_xodr_path = resolve_map_assets()
    random.seed(42)

    scenario = scenic.scenarioFromFile(
        SCENIC_PATH,
        params={'subclass': subclass, 'map': map_xodr_path, 'timestep': 0.01},
    )

    spawn_collisions = 0
    for _ in range(SCENES_PER_SUBCLASS):
        scene, _ = scenario.generate()
        assert len(scene.objects) == 2

        ego = next(o for o in scene.objects if getattr(o, "rolename", "") == "ego")
        target = next(o for o in scene.objects if getattr(o, "rolename", "") == "target")

        # В Scenic координаты при движении на Восток: X — продольная, Y — поперечная (влево +Y, вправо -Y)
        dx = abs(ego.position.x - target.position.x)
        dy = abs(ego.position.y - target.position.y)

        # Проверка отсутствия перекрытия габаритных прямоугольников на спавне
        if dx < (EGO_LENGTH + TARGET_LENGTH) / 2.0 and dy < (EGO_WIDTH + TARGET_WIDTH) / 2.0:
            spawn_collisions += 1

        # Проверка стороны слепой зоны (SubA: Target слева -> target.y > ego.y; SubB: Target справа -> target.y < ego.y)
        if subclass == SUBCLASS_A:
            assert target.position.y > ego.position.y
        else:
            assert target.position.y < ego.position.y

        # Проверка стохастической массы Target
        target_mass = float(scene.params['target_mass_kg'])
        assert 1100.0 <= target_mass <= 2050.0

    assert spawn_collisions == 0
