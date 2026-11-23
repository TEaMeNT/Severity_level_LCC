"""
Разрешение канонических путей к ассетам карты OpenDRIVE (maps/assets/map_ditch.xodr).
"""

from __future__ import annotations

import os

from hara_lcc.config import CANONICAL_XODR_FILENAME, DEFAULT_MAPS_DIR


def resolve_map_assets(custom_dir: str | None = None) -> tuple[str, str]:
    """
    Разрешение пути к папке ассетов карты и каноническому файлу OpenDRIVE `map_ditch.xodr`.
    Устаревшие fallback-ссылки на `map.xodr` и корень проекта удалены:
    единственный канонический путь по умолчанию — `maps/assets/map_ditch.xodr`.
    Поддерживается CLI-переопределение через `--map-dir`.
    """
    candidates: list[str] = []
    if custom_dir:
        candidates.append(os.path.abspath(custom_dir))
    if os.path.abspath(DEFAULT_MAPS_DIR) not in candidates:
        candidates.append(os.path.abspath(DEFAULT_MAPS_DIR))

    for c_dir in candidates:
        if os.path.isdir(c_dir):
            xodr_path = os.path.join(c_dir, CANONICAL_XODR_FILENAME)
            if os.path.isfile(xodr_path):
                return c_dir, xodr_path

    searched = ", ".join(os.path.join(d, CANONICAL_XODR_FILENAME) for d in candidates)
    raise FileNotFoundError(
        f"Канонический файл карты '{CANONICAL_XODR_FILENAME}' не найден. Проверенные пути: {searched}"
    )
