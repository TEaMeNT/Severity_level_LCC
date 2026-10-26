#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Удобный скрипт быстрого запуска сценария из корня проекта.
Делегирует исполнение в scenarios/run_scenario.py.
"""
import os
import sys

scenario_path = os.path.join(os.path.dirname(__file__), 'scenarios', 'run_scenario.py')
with open(scenario_path, 'r', encoding='utf-8') as f:
    code = compile(f.read(), scenario_path, 'exec')
    exec(code)
