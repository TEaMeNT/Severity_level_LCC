# Severity_level_LCC

Комплексный исследовательский проект и пайплайн симуляции в **CARLA 0.9.16** для анализа тяжести последствий (HARA / Severity S1–S3 по стандарту **ISO 26262-3** и **SAE J2980**) при отказе систем контроля слепых зон (Blind Spot Detection / LCC — Lane Centering Control) и съезде в кювет / столкновении с отбойником.

---

## 📂 Структура репозитория

```
Severity_level_LCC/
├── Severity level/             # Симуляция HARA и оценка уровня тяжести (Severity)
│   ├── runner.py               # Главный управляющий скрипт симуляции CARLA + PhysX
│   ├── LCC_blindSpot.scenic    # Сценарий на предметно-ориентированном языке Scenic
│   ├── map.xodr / map.snet     # Локальная дорожная разметка и сеть
│   ├── prompt.md               # Инженерное ТЗ и аудит физики
│   └── results/                # Результаты экспериментов (CSV)
│       └── lcc_hara_results.csv
│
├── maps/                       # Трасса 1-й технической категории (СП 34.13330.2021)
│   ├── assets/                 # FBX модели и разметка OpenDRIVE (map_ditch.xodr)
│   ├── generators/             # Генераторы геометрии и валидаторы уклонов
│   ├── pipeline/               # Скрипты сборки, кукинга и деплоя в Unreal Engine
│   ├── scenarios/              # Вспомогательные сценарии тестирования
│   ├── screenshots/            # Контрольные скриншоты визуализации
│   ├── ARCHITECTURE.md         # Описание архитектуры окружения
│   └── AGENT_GUIDE.md          # Инженерный регламент
│
└── .gitignore                  # Исключения временных файлов Python, кэша и архивов
```

---

## 🛠 Технологический стек

* **Симулятор:** CARLA Simulator 0.9.16 (Unreal Engine 4 + NVIDIA PhysX)
* **Язык сценариев:** Python 3.12, Scenic DSL
* **Стандарты:** ISO 26262-3 (HARA), SAE J2980
* **3D & Дороги:** OpenDRIVE (XODR), Blender, FBX
