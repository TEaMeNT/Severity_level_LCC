"""Общие фикстуры для оффлайн-автотестов пакета hara_lcc."""

from __future__ import annotations

from pathlib import Path
import pytest

from hara_lcc.analysis.loader import load_simulation_results
from hara_lcc.config import PROJECT_ROOT


@pytest.fixture(scope="session")
def project_root() -> Path:
    return Path(PROJECT_ROOT)


@pytest.fixture(scope="session")
def baseline_csv_path(project_root: Path) -> Path:
    path = project_root / "results" / "lcc_hara_results.csv"
    assert path.is_file(), f"Эталонный CSV не найден: {path}"
    return path


@pytest.fixture(scope="session")
def baseline_report_path(project_root: Path) -> Path:
    path = project_root / "results" / "summary_report.md"
    assert path.is_file(), f"Эталонный отчёт не найден: {path}"
    return path


@pytest.fixture(scope="session")
def baseline_records(baseline_csv_path: Path) -> list[dict]:
    return load_simulation_results(str(baseline_csv_path))
