"""
Скрипт автономной установки ассетов карты map_ditch в симулятор CARLA.
Позволяет развернуть карту на любом ПК, где установлен CARLA Simulator.

Использование:
    python install_map_to_carla.py [--carla-dir "C:\\CARLA"]
"""

import os
import sys
import shutil
import argparse

if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_CONTENT_DIR = os.path.join(BASE_DIR, "maps", "carla_content")
SOURCE_ASSETS_DIR = os.path.join(BASE_DIR, "maps", "assets")

DEFAULT_CARLA_PATHS = [
    r"C:\CARLA",
    r"D:\CARLA",
    os.environ.get("CARLA_ROOT", ""),
]


def find_carla_dir(custom_path: str = None) -> str:
    candidates = []
    if custom_path:
        candidates.append(os.path.abspath(custom_path))
    candidates.extend([p for p in DEFAULT_CARLA_PATHS if p])

    for c_path in candidates:
        if os.path.isdir(c_path):
            exe_path = os.path.join(c_path, "CarlaUE4.exe")
            content_path = os.path.join(c_path, "CarlaUE4", "Content")
            if os.path.isfile(exe_path) or os.path.isdir(content_path):
                return c_path
    return None


def install_assets(carla_root: str):
    print("=" * 70)
    print(" УСТАНОВКА КАРТЫ MAP_DITCH В CARLA SIMULATOR")
    print("=" * 70)
    print(f"Директория проекта: {BASE_DIR}")
    print(f"Целевая директория CARLA: {carla_root}")

    target_content_dir = os.path.join(carla_root, "CarlaUE4", "Content")
    if not os.path.isdir(target_content_dir):
        print(f"[ОШИБКА] Директория Content не найдена: {target_content_dir}")
        sys.exit(1)

    # 1. Копирование мешей и uasset
    print("\n[1/3] Копирование бинарных мешей (.uasset / .uexp)...")
    mesh_files = [f for f in os.listdir(SOURCE_CONTENT_DIR) if os.path.isfile(os.path.join(SOURCE_CONTENT_DIR, f))]
    for fname in mesh_files:
        src = os.path.join(SOURCE_CONTENT_DIR, fname)
        dst = os.path.join(target_content_dir, fname)
        shutil.copy2(src, dst)
        print(f"  [+] Скопирован: {fname}")

    # 2. Копирование карты уровней (.umap / .xodr)
    print("\n[2/3] Копирование уровня карты (Carla/Maps/map_ditch)...")
    src_map_dir = os.path.join(SOURCE_CONTENT_DIR, "Carla", "Maps", "map_ditch")
    dst_map_dir = os.path.join(target_content_dir, "Carla", "Maps", "map_ditch")
    os.makedirs(dst_map_dir, exist_ok=True)

    for root, dirs, files in os.walk(src_map_dir):
        rel_path = os.path.relpath(root, src_map_dir)
        target_root = os.path.join(dst_map_dir, rel_path) if rel_path != "." else dst_map_dir
        os.makedirs(target_root, exist_ok=True)
        for f in files:
            s_file = os.path.join(root, f)
            d_file = os.path.join(target_root, f)
            shutil.copy2(s_file, d_file)
            print(f"  [+] Скопирован: {os.path.join(rel_path, f) if rel_path != '.' else f}")

    # Копирование OpenDRIVE в общую папку OpenDrive, если она есть
    dst_opendrive = os.path.join(target_content_dir, "Carla", "Maps", "OpenDrive")
    if os.path.isdir(dst_opendrive):
        src_xodr = os.path.join(SOURCE_ASSETS_DIR, "map_ditch.xodr")
        if os.path.isfile(src_xodr):
            shutil.copy2(src_xodr, os.path.join(dst_opendrive, "map_ditch.xodr"))
            print("  [+] Скопирован OpenDRIVE: OpenDrive/map_ditch.xodr")

    print("\n[3/3] Проверка файлов...")
    target_umap = os.path.join(dst_map_dir, "map_ditch.umap")
    if os.path.isfile(target_umap):
        print(f"  [OK] Бинарный файл карты готов: {target_umap} ({os.path.getsize(target_umap)} байт)")
    else:
        print("  [ПРЕДУПРЕЖДЕНИЕ] Файл map_ditch.umap не найден.")

    print("\n" + "=" * 70)
    print(" УСТАНОВКА УСПЕШНО ЗАВЕРШЕНА!")
    print(" Теперь вы можете запускать runner.py на этом ПК.")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Автономный установщик карты map_ditch в CARLA")
    parser.add_argument("--carla-dir", type=str, default=None, help="Путь к папке CARLA (по умолчанию автопоиск)")
    args = parser.parse_args()

    carla_dir = find_carla_dir(args.carla_dir)
    if not carla_dir:
        print("[ОШИБКА] Не удалось обнаружить установленный CARLA Simulator.")
        print("Укажите путь вручную: python install_map_to_carla.py --carla-dir C:\\CARLA")
        sys.exit(1)

    install_assets(carla_dir)


if __name__ == "__main__":
    main()
