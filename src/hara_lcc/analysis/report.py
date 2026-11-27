"""
Шаблонизация и генерация Markdown-отчета summary_report.md и консольной сводки HARA.
"""

from __future__ import annotations

from collections import defaultdict
import os
import re

from hara_lcc.analysis.stats import (
    calc_clopper_pearson_ci,
    calc_stats,
    format_iso_margin,
    format_iso_pct,
    plural_ru,
)
from hara_lcc.config import (
    DEFAULT_TEMPLATE_PATH,
    PROJECT_ROOT,
    SEVERITY_RANK,
    SUBCLASS_A,
    SUBCLASS_B,
)
from hara_lcc.severity.iso26262 import (
    determine_asil,
    is_crash_run,
    is_valid_barrier_hit,
    is_valid_ditch_departure,
)

DEFAULT_REPORT_TEMPLATE = r"""# ОТЧЕТ ПО АНАЛИЗУ ОПАСНОСТЕЙ И ОЦЕНКЕ РИСКОВ (HARA)
## Функция: Пассивный контроль слепых зон при смене полосы (Lane Change Control, LCC / BSD — без вмешательства в управление)
### Стандарты: **ISO 26262-3:2018** (Clause 6, Annex B), **SAE J2980:2023** (5.2.1.5(g), 5.2.2.2, 5.2.3, Appendix B)

---

## 1. РЕЗЮМЕ ОЦЕНКИ И ИТОГОВЫЕ ВЕРДИКТЫ ТЯЖЕСТИ (SEVERITY)

* **Исследуемый функциональный отказ:** Необнаружение попутного транспортного средства в слепой зоне при начале маневра перестроения на загородной автомагистрали ($100\text{ км/ч}$).
* **Общий объем стохастического ансамбля сценариев:** $N_{total} = {total_runs}$ прогонов (Подкласс А: $N_A = {runs_a}$, Подкласс B: $N_B = {runs_b}$).
* **Репрезентативная выборка состоявшихся ДТП (Accident-Involved Runs):** $N_{crash} = {total_crashes}$ прогонов (Подкласс А: $n_A = {crashes_a}$, Подкласс B: $n_B = {crashes_b}$). Классификация тяжести последствий (Severity, S) по стандарту ISO 26262-3 / SAE J2980:2023 выносится строго по данной выборке.

**Таблица физических параметров сценария и дорожных условий ODD:**

| Параметр моделирования | Физическое значение | Обоснование / Нормативный стандарт |
| :--- | :---: | :--- |
| **Масса Ego (Chevrolet Impala)** | 3490 кг | Спец. масса защищаемого ОА представительского класса (аналог Aurus Senat) |
| **Масса Target (Seat Leon)** | `TruncatedNormal(1480, 220, [1100, 2050])` кг | Массовый C-класс; $m_{ego}/m_{tgt} \in [1.70; 3.17]$ (EEA 2023/2024, автопарк РФ) |
| **Начальная крейсерская скорость** | 100 км/ч (27.78 м/с) | Типовая крейсерская скорость загородной автомагистрали |
| **Профиль кювета (Подкласс B)** | Обочина $2.25$ м (консерват. отн. ГОСТ $3.0\text{–}3.5$ м), откос 1:4, $Z = -1.20$ м | Водоотводный кювет по СП 34.13330.2021 п. 7.27 / AASHTO RDG; граничные условия $Y = 9.75$ м (оба подкласса) |
| **Ограждение (Подкласс А)** | Жёсткое вертикальное ограждение $0.80$ м ($Y \le 0.0$ м) | Вертикальная жёсткая стенка (ГОСТ 33128-2014), учёт проекции скорости строго внутрь барьера |
| **Сцепление и тормоза (`Config F`)** | `FrictionScale`: `1.05` (асфальт, ~`0.81–0.86 g`), `0.57` (обочина/кювет $Y \ge 7.50$ м до `32 м`, ~`0.46–0.48 g`); EBD `3800/2050 Н·м` (`Ego`) и `1.00/0.54·m` (`Target`), `use_sweep_wheel_collision = True` | Пропорциональный EBD (~65/35%), sweep-коллайдеры колес, инвариантное замедление `Target` и сохранение управляемости без оракулов `loc_y` (FMVSS 135 / SAE 830612) |
| **Модель реакции и удержания** | $T_{react} \in [0.65, 1.60]$ с (мед. $1.05$ с), $T_{hold} \in [0.20, 0.35]$ с ($T_{hold} \le T_{react}$, внутри $T_{react}$) | Усечённо-логнормальная задержка (Green 2000; Taoka 1989; SAE J2980:2023 5.2.3.2(d)) |
| **Шаг и горизонт симуляции** | $dt = 0.01$ с (100 Гц, Substepping 100 Гц), горизонт $45$ с ($4500$ тиков) | Полный разгон `Impala 3490 кг` до $100\text{ км/ч}$ и завершение пост-ударной фазы без обрезки |

### 1.1. Итоговые вердикты HARA по раздельным строкам `1.1.A` / `1.1.B` и `Worst-Case` (ISO 26262-3 / SAE J2980:2023 §5.2.1.5(g), §5.2.2.2 Note 1):
1. **Строка HARA `1.1.A` — Подкласс А (Ограждение слева, $n_A = {crashes_a}$):** **Класс {assigned_class_a}**
   * *Обоснование:* {decision_reason_a}
   * *Точечная оценка и 95% ДИ Клоппера–Пирсона (информационно, SAE J2980 Appendix B §B.1):* $P_A(\ge S2 \mid \text{ДТП}) = {p_ge_s2_a_pct}$ (95% ДИ: $[{ci_ge_s2_a_low}, {ci_ge_s2_a_high}]$).
2. **Строка HARA `1.1.B` — Подкласс B (Кювет справа, $n_B = {crashes_b}$):** **Класс {assigned_class_b}**
   * *Обоснование:* {decision_reason_b}
   * *Точечная оценка и 95% ДИ Клоппера–Пирсона (информационно, SAE J2980 Appendix B §B.1):* $P_B(\ge S2 \mid \text{ДТП}) = {p_ge_s2_b_pct}$ (95% ДИ: $[{ci_ge_s2_b_low}, {ci_ge_s2_b_high}]$).
3. **Главный консервативный итог HARA (`Worst-Case Subclass`):** **Класс {s_worst}**
   * *Определение:* $S_{worst} = \max(S_A, S_B) = \max({assigned_class_a}, {assigned_class_b}) = {s_worst_bold}$.
   * *Область применения:* В соответствии с **SAE J2980:2023 §5.2.1.5(g)** и **§5.2.2.2 Note 1** различные дорожные профили (жёсткое ограждение `1.1.A` и водоотводный кювет `1.1.B`) ведутся в HARA **раздельными строками** без разбавляющего усреднения тяжести весами ODD (веса ${weight_a}$ и ${weight_b}$ учитываются в экспозиции $E$). При сведении в единый консервативный вердикт сценария принимается $S_{worst} = {s_worst_bold}$. *(Справочный расчёт взвешенного пула ODD приведён в отдельной информационной строке таблицы Раздела 2 и не используется для назначения класса тяжести HARA.)*

### 1.2. Методологическое обоснование раздельных строк HARA (исключение Double Weighting Paradox)
{odd_paradox_content}

### 1.3. Статистика реализации кинематического конфликта в слепой зоне (Pre-Crash Conflict Funnel)
{conflict_funnel_content}

### 1.4. Статистическая неопределённость: 95% ДИ Клоппера–Пирсона для $P(\ge S2 \mid \text{ДТП})$ и анализ исходов $S3$
{s3_analysis_content}

### 1.5. Количественная оценка управляемости (Controllability, C) по кампании `active`
{controllability_section}

### 1.6. Итоговая матрица классификации опасного события и назначение ASIL (ISO 26262-3)
{asil_matrix_content}

---

## 2. СВОДНАЯ ТАБЛИЦА РАСПРЕДЕЛЕНИЯ И КУМУЛЯТИВНЫХ ВЕРОЯТНОСТЕЙ

{table_severity_distribution}

---

## 3. СВОДНЫЙ ПРОТОКОЛ ПРИМЕНЕНИЯ ПРАВИЛА >10% (ISO 26262-3 / SAE J2980:2023)

Кумулятивная проверка порога репрезентативности $10.0\%$ проводится строго сверху вниз по выборке состоявшихся ДТП:
{protocol_steps_content}

---

## 4. СВОДНАЯ СТАТИСТИКА КИНЕМАТИЧЕСКИХ ПАРАМЕТРОВ СИМУЛЯЦИЙ

{table_kinematics_stats}

---

## 5. АНАЛИЗ МЕХАНИЗМОВ АВАРИЙ И АСИММЕТРИИ УЯЗВИМОСТИ УЧАСТНИКОВ

### 5.1. Подкласс А (Удары о жёсткое ограждение $0.80\text{ м}$)
{barrier_mechanisms_content}

### 5.2. Подкласс B (Механизмы схода в кювет на бровке $Y = 9.75\text{ м}$)
{ditch_mechanisms_content}

### 5.3. Асимметрия уязвимости при соотношении масс ($m_{ego}/m_{tgt} \in [1.70; 3.17]$)
При столкновении тяжёлого седана Ego ($3490\text{ кг}$) с легковым Target (`Seat Leon`, $1100\text{–}2050\text{ кг}$):
{actor_asymmetry_content}

---

## 6. ОБНОВЛЕННЫЕ ИНЖЕНЕРНЫЕ ГРАФИКИ

В директории `results/plots/` сформированы 5 публикационных графиков (300 DPI):
1. `01_severity_distribution.png` — Фактическое распределение исходов S0–S3 среди состоявшихся ДТП ($N_{crash} = {total_crashes}$) с информационными 95% ДИ $P(\ge S2 \mid \text{ДТП})$ для `1.1.A` и `1.1.B`.
2. `02_cumulative_iso_decision.png` — Воронка кумулятивного правила >10% для раздельных строк HARA `1.1.A` и `1.1.B` с усами 95% ДИ Клоппера–Пирсона, серой зоной отсечки и нормативным порогом 10.0%.
3. `03_ditch_phase_portrait.png` — Двухпанельный фазовый портрет кювета: Механизм 1 (чистый Cooperrider $V_{lat} > 5.5\text{ м/с}$) и Механизм 2 (удары о дно/откос $V_\perp > 12 / 18\text{ м/с}$) по всем сходам в кювет (SubB: {ditch_departures_b} + перекрёстные SubA: {ditch_departures_a_cross}, всего $n = {ditch_departures_total}$) с разметкой #run_id.
4. `04_subclass_a_dv_cascade.png` — Двухпанельный каскад скоростей удара в Подклассе А строго по состоявшимся ДТП ($n = {crashes_a}$) с адаптивной разметкой для малых и больших выборок (ТС–ТС vs Ограждение).
5. `05_kinematics_vs_severity_boxplots.png` — Дашборд эскалации тяжести при ДТП (Панель А, справочный взвешенный профиль ODD, $N_{crash} = {total_crashes}$) и воронка реализации конфликта в слепой зоне с распределением участников (Панель Б, $N_{total} = {total_runs}$).

---
*Отчет сформирован автоматически модулем `analyze_results.py` на основе физического моделирования CARLA / Scenic.*
"""


def render_template(template_str: str, context: dict) -> str:
    """
    Безопасная подстановка переменных {var_name} в шаблон Markdown.
    Игнорирует фигурные скобки KaTeX/LaTeX, заменяя только совпадения с ключами context.
    """
    pattern = re.compile(r'\{([a-zA-Z0-9_]+)\}')

    def _replace(match):
        key = match.group(1)
        if key in context:
            return str(context[key])
        return match.group(0)

    return pattern.sub(_replace, template_str)


def resolve_template_path(specified_path: str | None = None) -> str:
    """
    Определение пути к файлу шаблона отчета report_template.md.
    Приоритет поиска:
      1. Явно указанный путь (CLI-аргумент --template)
      2. <PROJECT_ROOT>/templates/report_template.md
      3. <PROJECT_ROOT>/report_template.md
    Если ни один файл не существует, автоматически создается templates/report_template.md
    с дефолтным содержимым DEFAULT_REPORT_TEMPLATE.
    """
    if specified_path:
        if os.path.isfile(specified_path):
            return specified_path
        else:
            print(f"  [WARN] Указанный файл шаблона '{specified_path}' не найден. Поиск шаблона по умолчанию...")

    default_paths = [
        DEFAULT_TEMPLATE_PATH,
        os.path.join(PROJECT_ROOT, "report_template.md"),
        os.path.join("templates", "report_template.md"),
        "report_template.md",
    ]
    for p in default_paths:
        if os.path.isfile(p):
            return p

    fallback_path = DEFAULT_TEMPLATE_PATH
    os.makedirs(os.path.dirname(fallback_path), exist_ok=True)
    with open(fallback_path, mode='w', encoding='utf-8') as f:
        f.write(DEFAULT_REPORT_TEMPLATE.strip() + "\n")
    print(f"  [INFO] Базовый шаблон отчета создан: {fallback_path}")
    return fallback_path


def _build_conflict_funnel_content(analysis_a: dict, analysis_b: dict) -> str:
    """Формирование текстового блока воронки реализации кинематического конфликта (Раздел 1.3)."""
    total_runs = analysis_a['total_runs'] + analysis_b['total_runs']
    total_crashes = analysis_a['total_crashes'] + analysis_b['total_crashes']
    total_safe = analysis_a['safe_pass_runs'] + analysis_b['safe_pass_runs']
    spawn_err_a = analysis_a.get('spawn_error_runs', 0)
    spawn_err_b = analysis_b.get('spawn_error_runs', 0)
    total_spawn_err = spawn_err_a + spawn_err_b
    p_total_conflict = (total_crashes / max(1, total_runs)) * 100.0
    p_total_safe = (total_safe / max(1, total_runs)) * 100.0
    p_total_err = (total_spawn_err / max(1, total_runs)) * 100.0

    err_str_a = f" Ошибок спавна акторов (`SimulationCreationError`): **{spawn_err_a}** ({spawn_err_a/max(1, analysis_a['total_runs'])*100:.1f}%)." if spawn_err_a > 0 else ""
    err_str_b = f" Ошибок спавна акторов (`SimulationCreationError`): **{spawn_err_b}** ({spawn_err_b/max(1, analysis_b['total_runs'])*100:.1f}%)." if spawn_err_b > 0 else ""
    err_str_tot = f", а в **{total_spawn_err} прогонах** ({p_total_err:.1f}%) зафиксирован отказ генерации сцены (`SimulationCreationError: Unable to spawn object`)" if total_spawn_err > 0 else ""

    safe_word = plural_ru(total_safe, ('прогон', 'прогона', 'прогонов'))
    safe_verb = 'завершился' if (total_safe % 10 == 1 and total_safe % 100 != 11) else 'завершились'
    lines = [
        f"* **Подкласс А (Ограждение слева):** Сгенерировано **{analysis_a['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_A$): **{analysis_a['total_crashes']}** ({analysis_a['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_a['safe_pass_runs']}** ({analysis_a['safe_pass_runs']/max(1, analysis_a['total_runs'])*100:.1f}%).{err_str_a}",
        f"* **Подкласс B (Кювет справа):** Сгенерировано **{analysis_b['total_runs']}** стохастических сцен. Реализовано кинематических столкновений ($n_B$): **{analysis_b['total_crashes']}** ({analysis_b['conflict_rate']:.1f}%). Безаварийных разъездов (Safe Pass): **{analysis_b['safe_pass_runs']}** ({analysis_b['safe_pass_runs']/max(1, analysis_b['total_runs'])*100:.1f}%).{err_str_b}",
        f"* **Суммарный ансамбль по ODD:** Из $N_{{total}} = {total_runs}$ стохастических прогонов состоялось **$N_{{crash}} = {total_crashes}$ ДТП** ({p_total_conflict:.1f}%), **{total_safe} {safe_word}** ({p_total_safe:.1f}%) {safe_verb} безопасным опережением без физического контакта ТС{err_str_tot}.",
    ]

    if total_safe > 0:
        lines.extend([
            "",
            "**Физическая природа безаварийных разъездов (Safe Pass):**",
            "В соответствии с ODD диапазон продольного смещения спавна Target (`Seat Leon`, $4.19\\text{ м}$) относительно Ego (`Chevrolet Impala`, $5.36\\text{ м}$) охватывает всю область перекрытия слепой зоны (`target_offset_y` от $-5.680$ до $-1.415\\text{ м}$). При генерации Target в дальней кормовой части диапазона динамика перестроения Ego на скорости $100\\text{ км/ч}$ ($27.78\\text{ м/с}$) в ряде сцен позволяет Ego завершить смещение на смежную полосу с опережением и без физического контакта кузовов.",
        ])
    else:
        lines.extend([
            "",
            "**Реализация конфликта:** В текущей выборке все валидные сценарии привели к физическому столкновению (`Safe Pass = 0`).",
        ])
    return "\n".join(lines)


def _get_cum_ci_95(analysis: dict, key: str) -> tuple[float, float]:
    """Безопасное извлечение 95% ДИ Клоппера–Пирсона для кумулятивной доли ('S3', '>=S2', '>=S1')."""
    cum_ci = analysis.get('cum_ci_95')
    if isinstance(cum_ci, dict) and key in cum_ci:
        return cum_ci[key]
    n = int(analysis.get('total_crashes', 0))
    counts = analysis.get('counts', {})
    if key == 'S3':
        k = int(counts.get('S3', 0))
    elif key == '>=S2':
        k = int(counts.get('S3', 0) + counts.get('S2', 0))
    else:
        k = int(counts.get('S3', 0) + counts.get('S2', 0) + counts.get('S1', 0))
    return calc_clopper_pearson_ci(k, n, confidence=0.95)


def _build_severity_table(analysis_a: dict, analysis_b: dict, analysis_odd: dict, s_worst: str) -> str:
    """Построение сводной таблицы распределения и кумулятивных вероятностей (Раздел 2)."""
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')

    lines = [
        r"| Дорожный профиль ODD | Вес в ODD | N_total (Конфликт %) | N_crash | S0 (Удар без травм, AIS 0) | S1 (Легкие) | S2 (Тяжелые) | S3 (Фатальные) | P(S3 \| ДТП) | P(≥S2 \| ДТП) | 95% ДИ P(≥S2 \| ДТП) | P(≥S1 \| ДТП) | Вердикт ISO 26262 |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    # Строка Подкласса А (1.1.A)
    c_a = analysis_a['counts']
    p_a = analysis_a['percentages']
    cp_a = analysis_a['cum_percentages']
    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    lines.append(
        f"| **1.1.A — Подкласс А (Ограждение)** | {w_a:.2f} | {n_tot_a_str} | {analysis_a['total_crashes']} | "
        f"{c_a.get('S0',0)} ({p_a.get('S0',0.0):.1f}%) | "
        f"{c_a.get('S1',0)} ({p_a.get('S1',0.0):.1f}%) | "
        f"{c_a.get('S2',0)} ({p_a.get('S2',0.0):.1f}%) | "
        f"{c_a.get('S3',0)} ({p_a.get('S3',0.0):.1f}%) | "
        f"{format_iso_pct(cp_a['S3'])} | {format_iso_pct(cp_a['>=S2'])} | [{ci2_a_lo*100:.2f}%, {ci2_a_hi*100:.2f}%] | {format_iso_pct(cp_a['>=S1'])} | **{analysis_a['assigned_class']}** |"
    )

    # Строка Подкласса B (1.1.B)
    c_b = analysis_b['counts']
    p_b = analysis_b['percentages']
    cp_b = analysis_b['cum_percentages']
    n_tot_b_str = f"{analysis_b['total_runs']} ({analysis_b['conflict_rate']:.1f}%)"
    lines.append(
        f"| **1.1.B — Подкласс B (Кювет)** | {w_b:.2f} | {n_tot_b_str} | {analysis_b['total_crashes']} | "
        f"{c_b.get('S0',0)} ({p_b.get('S0',0.0):.1f}%) | "
        f"{c_b.get('S1',0)} ({p_b.get('S1',0.0):.1f}%) | "
        f"{c_b.get('S2',0)} ({p_b.get('S2',0.0):.1f}%) | "
        f"{c_b.get('S3',0)} ({p_b.get('S3',0.0):.1f}%) | "
        f"**{format_iso_pct(cp_b['S3'])}** | **{format_iso_pct(cp_b['>=S2'])}** | [{ci2_b_lo*100:.2f}%, {ci2_b_hi*100:.2f}%] | {format_iso_pct(cp_b['>=S1'])} | **{analysis_b['assigned_class']}** |"
    )

    tot_runs = analysis_odd['total_runs']
    tot_crashes = analysis_odd['total_crashes']
    p_conf_odd = (tot_crashes / max(1, tot_runs)) * 100.0
    n_tot_odd_str = f"{tot_runs} ({p_conf_odd:.1f}%)"

    # Главная итоговая строка Worst-Case
    lines.append(
        f"| **WORST-CASE SUBCLASS (главный итог HARA)** | — | **{n_tot_odd_str}** | **{tot_crashes}** | "
        f"— | — | — | — | "
        f"— | — | — | — | **{s_worst}** |"
    )

    # Справочная строка Взвешенного ODD (не используется для вердикта HARA)
    p_odd = analysis_odd['percentages']
    cp_odd = analysis_odd['cum_percentages']
    lines.append(
        f"| *Взвешенный ODD (справочно; для HARA — 1.1.A и 1.1.B)* | *1.00* | *{n_tot_odd_str}* | *{tot_crashes}* | "
        f"*{p_odd.get('S0',0.0):.1f}%* | "
        f"*{p_odd.get('S1',0.0):.1f}%* | "
        f"*{p_odd.get('S2',0.0):.1f}%* | "
        f"*{p_odd.get('S3',0.0):.1f}%* | "
        f"*{format_iso_pct(cp_odd['S3'])}* | *{format_iso_pct(cp_odd['>=S2'])}* | *—* | *{format_iso_pct(cp_odd['>=S1'])}* | *{analysis_odd['assigned_class']} (справ.)* |"
    )

    return "\n".join(lines)


def _build_iso_steps_content(analysis_a: dict, analysis_b: dict, analysis_odd: dict) -> str:
    """Построение компактной сводной аналитики применения правила >10% (Раздел 3) для раздельных строк 1.1.A и 1.1.B."""
    def format_subclass_summary(name: str, an: dict, ci_s3_str: str = "", ci_s2_str: str = "") -> list[str]:
        cp = an['cum_percentages']
        p3 = cp.get('S3', 0.0)
        p2 = cp.get('>=S2', 0.0)
        p1 = cp.get('>=S1', 0.0)
        crashes = an['total_crashes']
        assigned = an['assigned_class']

        p3_str = format_iso_pct(p3).replace('%', '\\%')
        p2_str = format_iso_pct(p2).replace('%', '\\%')
        p1_str = format_iso_pct(p1).replace('%', '\\%')

        step_lines = [f"* **{name} (выборка ДТП $n = {crashes}$):**"]
        if crashes == 0:
            step_lines.append("  * Выборки состоявшихся ДТП отсутствуют (по умолчанию **Класс S0**).")
            step_lines.append(f"  * **Итоговый вердикт:** **Класс {assigned}**.")
            return step_lines

        if assigned == 'S3':
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 1**: $P(S3 \\mid \\text{{ДТП}}) = {p3_str}${ci_s3_str} $> 10.0\\%$ "
                f"— порог превышен на **{format_iso_margin(p3 - 10.0)}** ($P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_str}${ci_s2_str})."
            )
        elif assigned == 'S2':
            step_lines.append(
                f"  * $P(S3 \\mid \\text{{ДТП}}) = {p3_str}${ci_s3_str} $\\le 10.0\\%$ — нормативный порог фатальных травм не достигнут."
            )
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 2**: $P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_str}${ci_s2_str} $> 10.0\\%$ "
                f"— порог преодолен с запасом **{format_iso_margin(p2 - 10.0)}**."
            )
        elif assigned == 'S1':
            step_lines.append(
                f"  * $P(S3 \\mid \\text{{ДТП}}) = {p3_str}${ci_s3_str} $\\le 10.0\\%$, "
                f"$P(\\ge S2 \\mid \\text{{ДТП}}) = {p2_str}${ci_s2_str} $\\le 10.0\\%$ — пороги S3 и S2 не достигнуты."
            )
            step_lines.append(
                f"  * Финальная отсечка на **Шаге 3**: $P(\\ge S1 \\mid \\text{{ДТП}}) = {p1_str} > 10.0\\%$ "
                f"— порог преодолен на **{format_iso_margin(p1 - 10.0)}**."
            )
        else:
            step_lines.append(
                f"  * Кумулятивные вероятности $P(S3) = {p3_str}${ci_s3_str}, $P(\\ge S2) = {p2_str}${ci_s2_str} и "
                f"$P(\\ge S1 \\mid \\text{{ДТП}}) = {p1_str} \\le 10.0\\%$ — ни один порог вреда не превышен (**Шаг 4**)."
            )

        step_lines.append(f"  * **Итоговый вердикт:** **Класс {assigned}**.")
        return step_lines

    ci3_a_lo, ci3_a_hi = _get_cum_ci_95(analysis_a, 'S3')
    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
    ci3_b_lo, ci3_b_hi = _get_cum_ci_95(analysis_b, 'S3')
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')

    ci3_a_str = f" (95% ДИ: [{ci3_a_lo*100:.2f}%, {ci3_a_hi*100:.2f}%])"
    ci2_a_str = f" (95% ДИ: [{ci2_a_lo*100:.2f}%, {ci2_a_hi*100:.2f}%])"
    ci3_b_str = f" (95% ДИ: [{ci3_b_lo*100:.2f}%, {ci3_b_hi*100:.2f}%])"
    ci2_b_str = f" (95% ДИ: [{ci2_b_lo*100:.2f}%, {ci2_b_hi*100:.2f}%])"

    lines = []
    lines.extend(format_subclass_summary("Строка HARA 1.1.A — Подкласс А (Ограждение слева)", analysis_a, ci3_a_str, ci2_a_str))
    lines.append("")
    lines.extend(format_subclass_summary("Строка HARA 1.1.B — Подкласс B (Кювет справа)", analysis_b, ci3_b_str, ci2_b_str))
    lines.append("")
    lines.append(
        f"* *Справочно (не используется для назначения класса тяжести HARA):* "
        f"взвешенный пул ODD ($N_{{crash}} = {analysis_odd['total_crashes']}$) даёт "
        f"$P_{{ODD}}(S3 \\mid \\text{{ДТП}}) = {format_iso_pct(analysis_odd['cum_percentages']['S3']).replace('%', chr(92) + '%')}$, "
        f"$P_{{ODD}}(\\ge S2 \\mid \\text{{ДТП}}) = {format_iso_pct(analysis_odd['cum_percentages']['>=S2']).replace('%', chr(92) + '%')}$ "
        f"(формально **{analysis_odd['assigned_class']}**); по SAE J2980:2023 §5.2.1.5(g) и §5.2.2.2 Note 1 "
        f"усреднение между разнородными дорожными условиями не допускается."
    )

    return "\n".join(lines)


def _build_kinematics_table(records: list[dict]) -> str:
    """Построение сводной таблицы кинематических параметров (Раздел 4)."""
    stats_react = calc_stats([r['shock_duration_sec'] for r in records if r['shock_duration_sec'] is not None])
    stats_dv_side = calc_stats([r['effective_side_dv_mps'] for r in records if r['effective_side_dv_mps'] is not None])
    stats_barrier_v = calc_stats([r['barrier_impact_mps'] for r in records if r['subclass'] == SUBCLASS_A and is_valid_barrier_hit(r)])
    stats_dep_v = calc_stats([r['departure_speed_mps'] for r in records if is_valid_ditch_departure(r)])
    stats_perp_v = calc_stats([r['road_perp_speed_mps'] for r in records if is_valid_ditch_departure(r)])
    stats_beta = calc_stats([r['sideslip_angle_deg'] for r in records if is_valid_ditch_departure(r)])
    stats_yaw = calc_stats([r['departure_yaw_rate_deg_s'] for r in records if is_valid_ditch_departure(r)])

    lines = [
        "| Физический параметр | Число замеров | Среднее | Медиана | Станд. откл. (Std) | Минимум | Максимум | 90-й перцентиль |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    def format_row(name, s, unit):
        return f"| {name} [{unit}] | {s['count']} | {s['mean']:.2f} | {s['median']:.2f} | {s['std']:.2f} | {s['min']:.2f} | {s['max']:.2f} | {s['p90']:.2f} |"

    lines.append(format_row("Время реакции водителя $T_{react}$", stats_react, "с"))
    lines.append(format_row("Эффективная $\\Delta V$ бокового удара ТС-ТС", stats_dv_side, "м/с"))
    lines.append(format_row("Скорость вторичного удара в ограждение $V_{barrier}$", stats_barrier_v, "м/с"))
    lines.append(format_row("Полная скорость схода в кювет $V_{dep}$", stats_dep_v, "м/с"))
    lines.append(format_row("Нормальная скорость ухода в кювет $V_{\\perp}$", stats_perp_v, "м/с"))
    lines.append(format_row("Угол бокового заноса кузова $\\beta$", stats_beta, "град"))
    lines.append(format_row("Угловая скорость рыскания $\\omega_z$", stats_yaw, "град/с"))

    return "\n".join(lines)


def _build_odd_paradox_content(analysis_a: dict, analysis_b: dict, analysis_odd: dict, s_worst: str) -> str:
    """Формирование обоснования раздельных строк HARA 1.1.A / 1.1.B и Worst-Case (Раздел 1.2)."""
    p2_b = analysis_b['cum_percentages']['>=S2']
    p2_a = analysis_a['cum_percentages']['>=S2']
    s_a = analysis_a['assigned_class']
    s_b = analysis_b['assigned_class']
    s_odd = analysis_odd['assigned_class']
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    if s_worst == s_odd:
        diff_note = (
            f"Для текущей выборки справочный взвешенный расчёт $0.20\\cdot A + 0.80\\cdot B$ численно совпадает с "
            f"$S_{{worst}}$ ($S_{{ODD,\\text{{справ.}}}} = {s_odd}$), однако не используется как критерий назначения."
        )
    else:
        diff_note = (
            f"Справочный взвешенный расчёт $0.20\\cdot A + 0.80\\cdot B$ даёт заниженный класс $S_{{ODD,\\text{{справ.}}}} = {s_odd}$ "
            f"относительно $S_{{worst}} = {s_worst}$, что наглядно иллюстрирует эффект разбавления тяжести при смешивании подклассов."
        )

    return (
        f"В соответствии с **SAE J2980:2023 §5.2.1.5(g)** и **§5.2.2.2 Note 1** (а также ISO 26262-3 Clause 6) "
        f"различные дорожные профили — жёсткое ограждение (`1.1.A`: **$S_A = {s_a}$**, $P_A(\\ge S2 \\mid \\text{{ДТП}}) = {p2_a:.1f}\\%$) "
        f"и водоотводный кювет (`1.1.B`: **$S_B = {s_b}$**, $P_B(\\ge S2 \\mid \\text{{ДТП}}) = {p2_b:.1f}\\%$) — "
        f"заносятся в таблицу HARA **раздельными строками**, поскольку доли дорожной инфраструктуры ($W_A = {w_a:.2f}$, $W_B = {w_b:.2f}$) "
        f"уже учитываются в параметре экспозиции $E$ (исключение Double Weighting Paradox и разбавления тяжести). "
        f"Главным сводным итогом HARA является консервативный класс наихудшего подкласса "
        f"**$S_{{worst}} = \\max({s_a}, {s_b}) = {s_worst}$**. {diff_note}"
    )


def _build_s3_analysis_content(
    analysis_b: dict,
    records_b: list[dict],
    analysis_a: dict | None = None,
) -> str:
    """Формирование анализа 95% ДИ Клоппера–Пирсона для P(>=S2 | ДТП) (SubA/SubB) и P(S3 | ДТП) (Раздел 1.4)."""
    k_s3 = analysis_b['counts'].get('S3', 0)
    n_crashes = analysis_b['total_crashes']
    p_s3 = (k_s3 / max(1, n_crashes)) * 100.0
    ci_low, ci_high = _get_cum_ci_95(analysis_b, 'S3')

    k_ge_s2_b = int(analysis_b['counts'].get('S2', 0) + analysis_b['counts'].get('S3', 0))
    p_ge_s2_b = (k_ge_s2_b / max(1, n_crashes)) * 100.0
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')

    s3_runs = [r for r in records_b if is_crash_run(r) and r.get('severity') == 'S3']

    rollover_cnt = sum(1 for r in s3_runs if 'rollover' in str(r.get('ditch_mechanism', '')).lower())
    nose_in_cnt = sum(1 for r in s3_runs if 'nose-in' in str(r.get('ditch_mechanism', '')).lower())
    other_cnt = len(s3_runs) - rollover_cnt - nose_in_cnt

    mech_parts = []
    if rollover_cnt > 0:
        mech_parts.append(f"{rollover_cnt} {plural_ru(rollover_cnt, ('заезд', 'заезда', 'заездов'))} критического бокового срыва (High-Speed Soil-Trip Rollover Risk)")
    if nose_in_cnt > 0:
        mech_parts.append(f"{nose_in_cnt} {plural_ru(nose_in_cnt, ('заезд', 'заезда', 'заездов'))} крутого продольного утыкания (Severe Ditch Nose-In Impact при $V_\\perp > 18\\text{{ м/с}}$)")
    if other_cnt > 0:
        mech_parts.append(f"{other_cnt} {plural_ru(other_cnt, ('заезд', 'заезда', 'заездов'))} прочих механизмов")

    if k_s3 == 0:
        s3_mech_sentence = "В данной выборке состоявшихся ДТП критические исходы S3 полностью отсутствуют ($k_{S3} = 0$)."
    elif k_s3 == 1:
        s3_mech_sentence = f"Единственный заезд S3 зафиксирован по граничному условию на бровке $Y = 9.75\\text{{ м}}$: {' и '.join(mech_parts)}."
    else:
        s3_mech_sentence = f"Все {k_s3} {plural_ru(k_s3, ('заезд', 'заезда', 'заездов'))} S3 зафиксированы по граничному условию на бровке $Y = 9.75\\text{{ м}}$: {' и '.join(mech_parts)}."

    if ci_high * 100.0 < 10.0:
        ci_verdict = (
            f"Верхняя граница 95% доверительного интервала (**{ci_high*100:.2f}%**) лежит строго ниже порога **10.0%** "
            f"(при точечной оценке $P(S3 \\mid \\text{{ДТП}}) = {p_s3:.2f}\\%$). Это подтверждает низкую неопределённость исключения класса S3 для данного ODD. {s3_mech_sentence}"
        )
    elif p_s3 <= 10.0:
        ci_verdict = (
            f"Точечная оценка вероятности $P(S3 \\mid \\text{{ДТП}}) = {p_s3:.2f}\\%$ не превышает порога **10.0%** "
            f"(95% доверительный интервал Клоппера–Пирсона: $[{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$). {s3_mech_sentence}"
        )
    else:
        ci_verdict = (
            f"Точечная оценка вероятности $P(S3 \\mid \\text{{ДТП}}) = {p_s3:.2f}\\%$ превышает нормативный порог **10.0%** "
            f"(95% ДИ Клоппера–Пирсона: $[{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$) — класс S3 статистически репрезентативен. {s3_mech_sentence}"
        )

    lines = [
        "**1. Информационные 95% доверительные интервалы Клоппера–Пирсона для $P(\\ge S2 \\mid \\text{ДТП})$ (`SAE J2980:2023 Appendix B §B.1`):**",
        "Точный биномиальный 95% ДИ Клоппера–Пирсона приводится как **информационный индикатор статистической неопределённости конечной выборки** (при этом нормативная классификация выполняется по точечной оценке вероятности $> 10.0\\%$, верхняя граница ДИ не является жёстким правилом стандарта):",
        "",
    ]

    if analysis_a is not None:
        n_a = analysis_a['total_crashes']
        k_ge_s2_a = int(analysis_a['counts'].get('S2', 0) + analysis_a['counts'].get('S3', 0))
        p_ge_s2_a = (k_ge_s2_a / max(1, n_a)) * 100.0
        ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
        lines.append(
            f"* **Строка `1.1.A` (Подкласс А — ограждение):** "
            f"$P_A(\\ge S2 \\mid \\text{{ДТП}}) = \\frac{{{k_ge_s2_a}}}{{{n_a}}} = {p_ge_s2_a:.2f}\\%$, "
            f"$\\text{{CI}}_{{95\\%}} = [{ci2_a_lo*100:.2f}\\%, {ci2_a_hi*100:.2f}\\%]$."
        )

    lines.extend([
        f"* **Строка `1.1.B` (Подкласс B — кювет):** "
        f"$P_B(\\ge S2 \\mid \\text{{ДТП}}) = \\frac{{{k_ge_s2_b}}}{{{n_crashes}}} = {p_ge_s2_b:.2f}\\%$, "
        f"$\\text{{CI}}_{{95\\%}} = [{ci2_b_lo*100:.2f}\\%, {ci2_b_hi*100:.2f}\\%]$.",
        "",
        "**2. Детальный анализ критических исходов $S3$ в Подклассе B (сход в кювет):**",
        "",
        f"* **Точечная оценка вероятности S3 при ДТП:** $P(S3 \\mid \\text{{ДТП}}) = \\frac{{{k_s3}}}{{{n_crashes}}} = {p_s3:.2f}\\%$.",
        f"* **Точный 95% доверительный интервал:** $\\text{{CI}}_{{95\\%}} = [{ci_low*100:.2f}\\%, {ci_high*100:.2f}\\%]$.",
        "",
        "**Физико-статистическое заключение по S3:**",
        ci_verdict,
    ])

    if s3_runs:
        lines.append("")
        lines.append("**Параметры критических заездов с исходом S3:**")
        lines.append("| ID заезда | Подкласс | Участник схода | $V_{dep}$ [м/с] | $V_\\perp$ [м/с] | Угол заноса $\\beta$ | Скорость рыскания $\\omega_z$ | Механизм динамики | Severity |")
        lines.append("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- | :---: |")
        for r in s3_runs:
            dep_v = f"{float(r['departure_speed_mps']):.2f}" if r.get('departure_speed_mps') else "—"
            perp_v = f"{float(r['road_perp_speed_mps']):.2f}" if r.get('road_perp_speed_mps') else "—"
            beta = f"{float(r['sideslip_angle_deg']):.1f}°" if r.get('sideslip_angle_deg') else "—"
            yaw = f"{float(r['departure_yaw_rate_deg_s']):.1f}°/с" if r.get('departure_yaw_rate_deg_s') else "—"
            mech = r.get('ditch_mechanism', '—')
            lines.append(f"| #{r.get('run_id')} | {r.get('subclass')} | {r.get('ditch_departure_actor')} | {dep_v} | {perp_v} | {beta} | {yaw} | {mech} | **S3** |")

    return "\n".join(lines)


def _build_controllability_content(ctrl_a: dict | None = None, ctrl_b: dict | None = None, ctrl_all: dict | None = None) -> str:
    """Формирование раздела количественной оценки управляемости C по кампании active (SAE J2980:2023 Clause 5.2.3)."""
    if ctrl_all is None:
        return (
            "Отдельный CSV-файл кампании `--driver-post-shock active` (`--active-input`) не передан в данный запуск аналитики. "
            "Предварительные инженерные оценки управляемости в матрице ниже (`C2` при $\\ge 90\\%$ предотвращении S2+ и `C3` при $< 90\\%$) "
            "приведены по нормативным критериям SAE J2980:2023 Clause 5.2.3 и подлежат уточнению по итогам кампании `active`."
        )

    lines = [
        "По результатам моделирования активного контрруления и торможения водителей после фазы реакции $T_{react}$ "
        "(включающей удержание $T_{hold} \\le T_{react}$) оценена доля инцидентов $P_{\\text{avoid}}(< S2 \\mid \\text{конфликт})$, "
        "в которых удалось предотвратить тяжелый вред ($S2/S3$):",
        "",
        "| Выборка (active) | Валидных сцен | ДТП ($n_{\\text{crash}}$) | Предотвратили $S2+$ ($k/n$) | $P_{\\text{avoid}}$ (%) | 95% ДИ Клоппера–Пирсона | Класс C | Статус ДИ |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |",
    ]

    for label, ct in [
        ("Подкласс А (Ограждение)", ctrl_a),
        ("Подкласс B (Кювет)", ctrl_b),
        ("**Объединенная кампания `active`**", ctrl_all),
    ]:
        if ct is None:
            continue
        n_ref = ct['n_crashes'] if ct['n_crashes'] > 0 else ct['n_valid']
        k_ref = ct['k_avoid_crashes'] if ct['n_crashes'] > 0 else ct['k_avoid_valid']
        p_ref = ct['p_avoid_crashes'] if ct['n_crashes'] > 0 else ct['p_avoid_valid']
        ci = ct['ci_crashes'] if ct['n_crashes'] > 0 else ct['ci_valid']
        lines.append(
            f"| {label} | {ct['n_valid']} | {ct['n_crashes']} | {k_ref} / {n_ref} | "
            f"**{p_ref:.1f}%** | [{ci[0]*100:.1f}%, {ci[1]*100:.1f}%] | **{ct['assigned_c']}** | {ct['ci_note']} |"
        )

    lines.extend([
        "",
        "*Критерии назначения C (ISO 26262-3:2018 Table B.4 / SAE J2980:2023 Clause 5.2.3):* "
        "**C1** ($P_{\\text{avoid}} \\ge 99\\%$), **C2** ($90\\% \\le P_{\\text{avoid}} < 99\\%$), **C3** ($P_{\\text{avoid}} < 90\\%$).",
    ])
    return "\n".join(lines)


def _build_asil_matrix_content(
    s_worst: str,
    s_odd: str,
    analysis_a: dict | None = None,
    analysis_b: dict | None = None,
    analysis_odd: dict | None = None,
    ctrl_a: dict | None = None,
    ctrl_b: dict | None = None,
    ctrl_all: dict | None = None,
) -> str:
    """Формирование итоговой матрицы классификации опасного события и назначения ASIL (Раздел 1.6)."""
    p_ge_s2_b = analysis_b['cum_percentages']['>=S2'] if analysis_b else 0.0
    p_ge_s2_a = analysis_a['cum_percentages']['>=S2'] if analysis_a else 0.0
    s_a_lbl = analysis_a['assigned_class'] if analysis_a else 'S0'
    s_b_lbl = analysis_b['assigned_class'] if analysis_b else 'S0'

    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2') if analysis_a else (0.0, 0.0)
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2') if analysis_b else (0.0, 0.0)

    if analysis_b and analysis_b.get('total_crashes', 0) > 0:
        _, ci_high_b = _get_cum_ci_95(analysis_b, 'S3')
        p_s3_b = analysis_b['cum_percentages']['S3']
        if ci_high_b * 100.0 < 10.0:
            s3_excl_str = f"Класс S3 исключен ($P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\%$, верхняя граница 95% ДИ: ${ci_high_b*100:.2f}\\% < 10\\%$)."
        elif p_s3_b <= 10.0:
            s3_excl_str = f"Доля S3 в Подклассе B составляет $P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\% \\le 10\\%$ (95% ДИ до ${ci_high_b*100:.2f}\\%$)."
        else:
            s3_excl_str = f"Доля S3 в Подклассе B составляет $P_B(S3 \\mid \\text{{ДТП}}) = {p_s3_b:.1f}\\% > 10\\%$ (95% ДИ до ${ci_high_b*100:.2f}\\%$)."
    else:
        s3_excl_str = "Статистическая оценка S3 формируется после прогонов в Подклассе B."

    asil_e3_c2 = determine_asil(s_worst, 'E3', 'C2')
    asil_e3_c3 = determine_asil(s_worst, 'E3', 'C3')
    asil_e4_c2 = determine_asil(s_worst, 'E4', 'C2')
    asil_e4_c3 = determine_asil(s_worst, 'E4', 'C3')

    if ctrl_all is not None:
        c_meas = ctrl_all['assigned_c']
        c_row_label = f"**{c_meas}** (замер `active`) / C2–C3"
        p_av = ctrl_all['p_avoid_crashes'] if ctrl_all['n_crashes'] > 0 else ctrl_all['p_avoid_valid']
        c_row_desc = (
            f"• **Измеренный класс {c_meas} (`active`):** предотвращение S2+ в **{p_av:.1f}%** случаев "
            f"({ctrl_all['ci_note']}).<br>"
            "• **C2** — реакция $\\ge 90\\%$ водителей; **C3** — пассивный выбег / $<90\\%$ удержания."
        )
        asil_meas_e3 = determine_asil(s_worst, 'E3', c_meas)
        measured_row = (
            f"| **Измеренный (`active`, E3 / {c_meas})** | **{s_worst}** | **E3** | **{c_meas}** | **{asil_meas_e3}** | "
            f"Прямой результат моделирования активной управляемости в ODD |"
        )
    else:
        c_row_label = "**C2** / **C3** (ожид. `--active-input`)"
        c_row_desc = (
            "• **C2** — нормативная реакция большинства водителей ($\\ge 90\\%$) по парированию заноса после фазы реакции $T_{react}$ ($T_{hold} \\le T_{react}$).<br>"
            "• **C3** — пассивный водитель без контрруления либо невозможность удержания курса ($<90\\%$)."
        )
        measured_row = None

    p2_a_tex = format_iso_pct(p_ge_s2_a).replace('%', '\\%')
    p2_b_tex = format_iso_pct(p_ge_s2_b).replace('%', '\\%')
    lines = [
        f"На основе главного консервативного итога HARA **{s_worst}** (`Worst-Case` по раздельным строкам `1.1.A = {s_a_lbl}`, `1.1.B = {s_b_lbl}`; SAE J2980:2023 §5.2.1.5(g), §5.2.2.2 Note 1 / ISO 26262-3) формируется матрица классификации опасного события по трем параметрам риска:",
        "",
        "| Параметр риска ISO 26262-3 | Назначенный класс | Обоснование и инженерный контекст |",
        "| :--- | :---: | :--- |",
        "| **Опасное событие (Hazardous Event)** | — | **Столкновение с попутным ТС в слепой зоне при перестроении на скорости 100 км/ч (27.78 м/с) с последующим ударом в ограждение или сходом в кювет** |",
        f"| **Тяжесть (Severity, S)** | **{s_worst}** (`1.1.A`: {s_a_lbl}, `1.1.B`: {s_b_lbl}) | Обосновано по раздельным строкам HARA в CARLA: строка `1.1.B` (кювет) — $P_B(\\ge S2 \\mid \\text{{ДТП}}) = {p2_b_tex}$ (95% ДИ $[{ci2_b_lo*100:.2f}\\%, {ci2_b_hi*100:.2f}\\%]$); строка `1.1.A` (ограждение) — $P_A(\\ge S2 \\mid \\text{{ДТП}}) = {p2_a_tex}$ (95% ДИ $[{ci2_a_lo*100:.2f}\\%, {ci2_a_hi*100:.2f}\\%]$). {s3_excl_str} |",
        "| **Экспозиция (Exposure, E)** | **E3** / **E4** | • **E3** — средняя частота возникновения опасной ситуации по ISO 26262-3 Table B.3 (от нескольких раз в год до нескольких раз в месяц, $1\\text{–}10\\%$ времени движения; по частотному расчёту ODD проекта $\\approx 20\\text{–}80\\text{ раз/год}$ или $1.67\\text{–}6.7\\text{ раз/мес}$).<br>• **E4** — консервативная оценка высокой интенсивности трафика ($>10\\%$ длительности поездок / почти в каждой поездке в плотном потоке). |",
        f"| **Контролируемость (Controllability, C)** | {c_row_label} | {c_row_desc} |",
        "",
        "#### Сводная матрица определения уровня полноты безопасности (ASIL Determination по ISO 26262-3 Table 4):",
        "",
        "| Эксплуатационный профиль | Severity (S) | Exposure (E) | Controllability (C) | Целевой ASIL | Инженерная интерпретация |",
        "| :--- | :---: | :---: | :---: | :---: | :--- |",
    ]
    if measured_row is not None:
        lines.append(measured_row)
    lines.extend([
        f"| **Профиль E3 / C2** | **{s_worst}** | **E3** | **C2** | **{asil_e3_c2}** | Типовая загородная трасса, контрруление $\\ge 90\\%$ водителей |",
        f"| **Консервативный E3 / C3** | **{s_worst}** | **E3** | **C3** | **{asil_e3_c3}** | Типовая частота перестроений, удержание $< 90\\%$ / пассивный режим |",
        f"| **Плотный трафик E4 / C2** | **{s_worst}** | **E4** | **C2** | **{asil_e4_c2}** | Высокая интенсивность потока, парирование заноса большинством водителей |",
        f"| **Наихудший случай E4 / C3** | **{s_worst}** | **E4** | **C3** | **{asil_e4_c3}** | Максимальная экспозиция потока и критически низкая управляемость |",
        "",
        "**Рекомендации по утверждению требований функциональной безопасности (Safety Goals):**",
        f"1. Базовый целевой уровень безопасности для функции LCC / BSD в номинальной частоте ODD (E3, C2) составляет **{asil_e3_c2}**.",
        f"2. При подтверждении **C3** в кампании `active` или консервативном сценарии E4 целевой уровень повышается до **{asil_e3_c3}** / **{asil_e4_c3}**.",
    ])
    return "\n".join(lines)


def _build_mechanisms_and_asymmetry_content(records: list[dict], analysis_a: dict, analysis_b: dict) -> tuple[str, str, str]:
    """Формирование текстовых блоков механизмов аварий и асимметрии масс (Раздел 5)."""
    sub_a_crashes = [r for r in records if r['subclass'] == SUBCLASS_A and is_crash_run(r)]
    sub_b_crashes = [r for r in records if r['subclass'] == SUBCLASS_B and is_crash_run(r)]

    barrier_types = defaultdict(int)
    for r in sub_a_crashes:
        if is_valid_barrier_hit(r) and r['barrier_impact_type']:
            barrier_types[f"{r['barrier_impact_type']} ({r['barrier_collision_actor']})"] += 1

    ditch_mechanisms = defaultdict(int)
    for r in sub_b_crashes:
        if is_valid_ditch_departure(r) and r['ditch_mechanism']:
            ditch_mechanisms[r['ditch_mechanism']] += 1

    # 5.1 Ограждение
    barrier_lines = []
    if barrier_types:
        for b_name, count in sorted(barrier_types.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_a['total_crashes']) * 100
            pct_total = count / max(1, analysis_a['total_runs']) * 100
            barrier_lines.append(f"* **{b_name}:** {count} {plural_ru(count, ('прогон', 'прогона', 'прогонов'))} ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
        stats_bar_v = calc_stats([r['barrier_impact_mps'] for r in sub_a_crashes if is_valid_barrier_hit(r)])
        max_vb = stats_bar_v['max']
        barrier_lines.append(
            f"* *Методологическая сноска по Подклассу А:* оценка $S_A = {analysis_a['assigned_class']}$ дана по доударной нормальной скорости "
            f"(`Side-swipe` $\\le {max_vb:.2f}\\text{{ м/с}}$) для гладкого сертифицированного ограждения без зацепа "
            f"(резкая остановка кузова у стенки в CARLA является артефактом твёрдотельного трения PhysX `0.72–1.42`)."
        )
    else:
        barrier_lines.append("* Удары в ограждение не зафиксированы.")
    barrier_content = "\n".join(barrier_lines)

    # 5.2 Кювет
    ditch_lines = []
    if ditch_mechanisms:
        for m_name, count in sorted(ditch_mechanisms.items(), key=lambda x: x[1], reverse=True):
            pct_crash = count / max(1, analysis_b['total_crashes']) * 100
            pct_total = count / max(1, analysis_b['total_runs']) * 100
            ditch_lines.append(f"* **{m_name}:** {count} {plural_ru(count, ('прогон', 'прогона', 'прогонов'))} ({pct_crash:.1f}% от ДТП, {pct_total:.1f}% от N_total)")
    else:
        ditch_lines.append("* Вылеты за бровку проезжей части Y = 9.75 м не зафиксированы.")
    ditch_content = "\n".join(ditch_lines)

    # 5.3 Асимметрия уязвимости
    asym_lines = []
    sub_a_runs = [r for r in records if r['subclass'] == SUBCLASS_A]
    sub_b_runs = [r for r in records if r['subclass'] == SUBCLASS_B]

    if sub_a_runs:
        a_hits = [r for r in sub_a_runs if is_valid_barrier_hit(r)]
        a_tgt_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('target', 'both'))
        a_ego_inv = sum(1 for r in a_hits if r.get('barrier_collision_actor') in ('ego', 'both'))
        a_both = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'both')
        a_tgt_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'target')
        a_ego_only = sum(1 for r in a_hits if r.get('barrier_collision_actor') == 'ego')

        def loc_zaezd(n: int) -> str:
            return 'заезде' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'заездах'

        def loc_sluchai(n: int) -> str:
            return 'случае' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'случаях'

        def loc_progon(n: int) -> str:
            return 'прогоне' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'прогонах'

        detail_parts = []
        if a_both > 0:
            detail_parts.append(f"в {a_both} {loc_zaezd(a_both)} ударились оба ТС")
        if a_tgt_only > 0:
            detail_parts.append(f"в {a_tgt_only} — только Target")
        if a_ego_only > 0:
            detail_parts.append(f"в {a_ego_only} — только Ego")
        details_a_str = f" ({', '.join(detail_parts)})" if detail_parts else ""

        pct_a_bar = (len(a_hits) / max(1, len(sub_a_runs))) * 100.0
        pct_a_crash = (len(a_hits) / max(1, analysis_a['total_crashes'])) * 100.0
        a_cross_deps = [r for r in sub_a_runs if is_valid_ditch_departure(r)]
        cross_str = (
            f" Перекрёстный вылет за правую бровку кювета $Y = 9.75\\text{{ м}}$ в Подклассе А зафиксирован в **{len(a_cross_deps)}** {loc_progon(len(a_cross_deps))}."
            if a_cross_deps else ""
        )
        asym_lines.append(
            f"* В Подклассе А контакт с ограждением зафиксирован в **{pct_a_bar:.1f}%** прогонов "
            f"({len(a_hits)}/{len(sub_a_runs)} от N_total, {pct_a_crash:.1f}% от выборки ДТП). "
            f"Из них Ego контактировал в {a_ego_inv} {loc_sluchai(a_ego_inv)}, а Target — в {a_tgt_inv}{details_a_str}.{cross_str}"
        )

    if sub_b_runs:
        def loc_z_b(n: int) -> str:
            return 'заезде' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'заездах'

        def loc_sl_b(n: int) -> str:
            return 'случае' if (abs(int(n)) % 10 == 1 and abs(int(n)) % 100 != 11) else 'случаях'

        b_deps = [r for r in sub_b_runs if is_valid_ditch_departure(r)]
        b_tgt_inv = sum(1 for r in b_deps if r.get('ditch_departure_actor') in ('target', 'both'))
        b_tgt_only = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'target')
        b_ego_inv = sum(1 for r in b_deps if r.get('ditch_departure_actor') in ('ego', 'both'))
        b_ego_only = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'ego')
        b_both = sum(1 for r in b_deps if r.get('ditch_departure_actor') == 'both')

        single_actor_details = []
        if b_tgt_only > 0:
            single_actor_details.append(f"в {b_tgt_only} {loc_z_b(b_tgt_only)} с полосы сошел только Target")
        if b_ego_only > 0:
            single_actor_details.append(f"в {b_ego_only} {loc_z_b(b_ego_only)} — только Ego")

        if not b_deps:
            single_str = ""
        elif single_actor_details:
            single_str = f" При этом {', '.join(single_actor_details)} (в остальных {b_both} {loc_z_b(b_both)} в кювет сошли оба ТС)."
        else:
            single_str = f" Во всех {b_both} {loc_z_b(b_both)} схода в кювет были вовлечены оба ТС."

        pct_b_dep = (len(b_deps) / max(1, len(sub_b_runs))) * 100.0
        pct_b_crash = (len(b_deps) / max(1, analysis_b['total_crashes'])) * 100.0
        asym_lines.append(
            f"* В Подклассе B сход с проезжей части в кювет произошел в **{pct_b_dep:.1f}%** прогонов "
            f"({len(b_deps)}/{len(sub_b_runs)} от N_total, {pct_b_crash:.1f}% от выборки ДТП). "
            f"При этом Target сносится с полосы в {b_tgt_inv} {loc_sl_b(b_tgt_inv)} ({b_tgt_inv/max(1, len(sub_b_runs))*100:.1f}%), "
            f"а Ego — в {b_ego_inv} {loc_sl_b(b_ego_inv)} ({b_ego_inv/max(1, len(sub_b_runs))*100:.1f}%).{single_str}"
        )

    asym_lines.append("* **Вывод:** Данные подтверждают требование стандарта ISO 26262-3 о двустороннем анализе тяжести последствий как для экипажа Ego, так и для сторонних участников движения (Target).")
    asymmetry_content = "\n".join(asym_lines)

    return barrier_content, ditch_content, asymmetry_content


def generate_markdown_report(
    records: list[dict],
    analysis_a: dict,
    analysis_b: dict,
    analysis_odd: dict,
    s_worst: str,
    output_dir: str,
    template_path: str | None = None,
    ctrl_a: dict | None = None,
    ctrl_b: dict | None = None,
    ctrl_all: dict | None = None,
) -> str:
    """
    Формирование строгого научно-инженерного отчета по функциональной безопасности.
    Архитектурно отделяет бизнес-логику расчетов от представления через Markdown-шаблон.
    """
    resolved_template_path = resolve_template_path(template_path)
    with open(resolved_template_path, mode='r', encoding='utf-8') as f:
        template_str = f.read()

    total_runs = len(records)
    crash_records = [r for r in records if is_crash_run(r)]
    total_crashes = len(crash_records)
    records_b = [r for r in records if r['subclass'] == SUBCLASS_B]
    ditch_departures = [r for r in records if is_valid_ditch_departure(r)]
    ditch_departures_b = sum(1 for r in ditch_departures if r['subclass'] == SUBCLASS_B)
    ditch_departures_a_cross = sum(1 for r in ditch_departures if r['subclass'] == SUBCLASS_A)
    ditch_departures_total = len(ditch_departures)
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})

    table_severity = _build_severity_table(analysis_a, analysis_b, analysis_odd, s_worst)
    protocol_steps = _build_iso_steps_content(analysis_a, analysis_b, analysis_odd)
    table_kinematics = _build_kinematics_table(records)
    odd_paradox = _build_odd_paradox_content(analysis_a, analysis_b, analysis_odd, s_worst)
    conflict_funnel = _build_conflict_funnel_content(analysis_a, analysis_b)
    s3_analysis = _build_s3_analysis_content(analysis_b, records_b, analysis_a=analysis_a)
    controllability_section = _build_controllability_content(ctrl_a, ctrl_b, ctrl_all)
    asil_matrix = _build_asil_matrix_content(
        s_worst, analysis_odd['assigned_class'], analysis_a, analysis_b, analysis_odd,
        ctrl_a=ctrl_a, ctrl_b=ctrl_b, ctrl_all=ctrl_all,
    )
    barrier_content, ditch_content, asymmetry_content = _build_mechanisms_and_asymmetry_content(
        records, analysis_a, analysis_b,
    )

    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')
    p_ge_s2_a_pct = format_iso_pct(analysis_a['cum_percentages']['>=S2']).replace('%', '\\%')
    p_ge_s2_b_pct = format_iso_pct(analysis_b['cum_percentages']['>=S2']).replace('%', '\\%')

    context = {
        'total_runs': total_runs,
        'total_crashes': total_crashes,
        'runs_a': analysis_a['total_runs'],
        'crashes_a': analysis_a['total_crashes'],
        'runs_b': analysis_b['total_runs'],
        'crashes_b': analysis_b['total_crashes'],
        'runs_c': analysis_b['total_runs'],
        'crashes_c': analysis_b['total_crashes'],
        'ditch_departures_b': ditch_departures_b,
        'ditch_departures_a_cross': ditch_departures_a_cross,
        'ditch_departures_total': ditch_departures_total,
        'weight_a': f"{weights.get('A', 0.20):.2f}",
        'weight_b': f"{weights.get('B', 0.80):.2f}",
        'assigned_class_a': analysis_a['assigned_class'],
        'decision_reason_a': analysis_a['decision_reason'],
        'assigned_class_b': analysis_b['assigned_class'],
        'decision_reason_b': analysis_b['decision_reason'],
        'assigned_class_c': analysis_b['assigned_class'],
        'decision_reason_c': analysis_b['decision_reason'],
        'p_ge_s2_a_pct': p_ge_s2_a_pct,
        'ci_ge_s2_a_low': f"{ci2_a_lo*100:.2f}\\%",
        'ci_ge_s2_a_high': f"{ci2_a_hi*100:.2f}\\%",
        'p_ge_s2_b_pct': p_ge_s2_b_pct,
        'ci_ge_s2_b_low': f"{ci2_b_lo*100:.2f}\\%",
        'ci_ge_s2_b_high': f"{ci2_b_hi*100:.2f}\\%",
        's_worst': s_worst,
        's_worst_bold': f"\\mathbf{{{s_worst}}}",
        'assigned_class_odd': analysis_odd['assigned_class'],
        'decision_reason_odd': analysis_odd['decision_reason'],
        'odd_paradox_content': odd_paradox,
        'conflict_funnel_content': conflict_funnel,
        's3_analysis_content': s3_analysis,
        'controllability_section': controllability_section,
        'asil_matrix_content': asil_matrix,
        'table_severity_distribution': table_severity,
        'table_2_content': table_severity,
        'protocol_steps_content': protocol_steps,
        'section_3_content': protocol_steps,
        'table_kinematics_stats': table_kinematics,
        'stats_table_content': table_kinematics,
        'barrier_mechanisms_content': barrier_content,
        'ditch_mechanisms_content': ditch_content,
        'actor_asymmetry_content': asymmetry_content,
    }

    report_content = render_template(template_str, context)

    os.makedirs(output_dir, exist_ok=True)
    report_path = os.path.join(output_dir, "summary_report.md")
    with open(report_path, mode='w', encoding='utf-8') as f:
        f.write(report_content)

    print(f"  [OK] Полный аналитический отчет сохранен в: {report_path} (шаблон: {resolved_template_path})")
    return report_path


def print_console_summary(
    analysis_a: dict,
    analysis_b: dict,
    analysis_odd: dict,
    s_worst: str,
    ctrl_a: dict | None = None,
    ctrl_b: dict | None = None,
    ctrl_all: dict | None = None,
):
    """Форматированный вывод сводной таблицы и многоуровневого вердикта в консоль."""
    weights = analysis_odd.get('weights', {'A': 0.20, 'B': 0.80})
    w_a = weights.get('A', 0.20)
    w_b = weights.get('B', 0.80)

    ci2_a_lo, ci2_a_hi = _get_cum_ci_95(analysis_a, '>=S2')
    ci2_b_lo, ci2_b_hi = _get_cum_ci_95(analysis_b, '>=S2')

    print("\n" + "=" * 124)
    print(" СВОДНАЯ ОЦЕНКА ТЯЖЕСТИ HARA ПО СТАНДАРТАМ ISO 26262-3 / SAE J2980:2023 (>10% RULE ПО ВЫБОРКЕ ДТП)")
    print("=" * 124)
    header = f"{'Профиль ODD':<26} | {'Вес':<4} | {'N_tot (Конфл)':<14} | {'N_crash':<7} | {'S0 (%)':<7} | {'S1 (%)':<7} | {'S2 (%)':<7} | {'S3 (%)':<7} | {'P(≥S2)':<7} | {'95% ДИ P(≥S2)':<16} | {'Вердикт':<8}"
    print(header)
    print("-" * 124)

    def print_row(name, weight, n_tot_str, n_crash, pcts, cum_pcts, ci_str, assigned):
        s0_str = f"{pcts.get('S0',0):.1f}%"
        s1_str = f"{pcts.get('S1',0):.1f}%"
        s2_str = f"{pcts.get('S2',0):.1f}%"
        s3_str = f"{pcts.get('S3',0):.1f}%"
        p_ge_s2 = format_iso_pct(cum_pcts.get('>=S2', 0))
        print(f"{name:<26} | {weight:<4.2f} | {n_tot_str:<14} | {n_crash:<7} | {s0_str:<7} | {s1_str:<7} | {s2_str:<7} | {s3_str:<7} | {p_ge_s2:<7} | {ci_str:<16} | {assigned:<8}")

    n_tot_a_str = f"{analysis_a['total_runs']} ({analysis_a['conflict_rate']:.1f}%)"
    print_row(
        "1.1.A Подкласс А (Барьер)", w_a, n_tot_a_str, analysis_a['total_crashes'],
        analysis_a['percentages'], analysis_a['cum_percentages'],
        f"[{ci2_a_lo*100:.1f}%, {ci2_a_hi*100:.1f}%]",
        f"S{SEVERITY_RANK[analysis_a['assigned_class']]}",
    )

    n_tot_b_str = f"{analysis_b['total_runs']} ({analysis_b['conflict_rate']:.1f}%)"
    print_row(
        "1.1.B Подкласс B (Кювет)", w_b, n_tot_b_str, analysis_b['total_crashes'],
        analysis_b['percentages'], analysis_b['cum_percentages'],
        f"[{ci2_b_lo*100:.1f}%, {ci2_b_hi*100:.1f}%]",
        f"S{SEVERITY_RANK[analysis_b['assigned_class']]}",
    )

    print("-" * 124)
    tot_runs_odd = max(1, analysis_odd['total_runs'])
    n_tot_odd_str = f"{analysis_odd['total_runs']} ({(analysis_odd['total_crashes']/tot_runs_odd)*100:.1f}%)"
    print(f"{'WORST-CASE SUBCLASS (HARA)':<26} | {'-':<4} | {n_tot_odd_str:<14} | {analysis_odd['total_crashes']:<7} | {'-':<7} | {'-':<7} | {'-':<7} | {'-':<7} | {'-':<7} | {'-':<16} | >>> {s_worst} <<<")
    print_row(
        "Взвешенный ODD (справочно)", 1.00, n_tot_odd_str, analysis_odd['total_crashes'],
        analysis_odd['percentages'], analysis_odd['cum_percentages'],
        "-",
        f"{analysis_odd['assigned_class']}*",
    )
    print("=" * 124)
    print("ИТОГОВЫЕ ВЕРДИКТЫ HARA (SAE J2980:2023 §5.2.1.5(g), §5.2.2.2 Note 1 — ПО ВЫБОРКЕ ДТП):")
    print(
        f"  * Строка 1.1.A (Ограждение слева): {analysis_a['assigned_class']} "
        f"({analysis_a['decision_step']}; P(>=S2) 95% CI [{ci2_a_lo*100:.2f}%, {ci2_a_hi*100:.2f}%])"
    )
    print(
        f"  * Строка 1.1.B (Кювет справа):     {analysis_b['assigned_class']} "
        f"({analysis_b['decision_step']}; P(>=S2) 95% CI [{ci2_b_lo*100:.2f}%, {ci2_b_hi*100:.2f}%])"
    )
    print(f"  * Главный итог HARA (Worst-Case):  >>> {s_worst} <<< = max(S_A, S_B)")
    print(f"  * Взвешенный ODD (справочно*):     {analysis_odd['assigned_class']} (не используется для вердикта HARA)")

    if ctrl_all is not None:
        print("-" * 108)
        print("ОЦЕНКА УПРАВЛЯЕМОСТИ CONTROLLABILITY (C) ПО КАМПАНИИ --driver-post-shock active:")
        for lbl, ct in [("Подкласс А", ctrl_a), ("Подкласс B", ctrl_b), ("Итого Active", ctrl_all)]:
            if ct is None:
                continue
            p_av = ct['p_avoid_crashes'] if ct['n_crashes'] > 0 else ct['p_avoid_valid']
            n_av = ct['n_crashes'] if ct['n_crashes'] > 0 else ct['n_valid']
            k_av = ct['k_avoid_crashes'] if ct['n_crashes'] > 0 else ct['k_avoid_valid']
            print(f"  * {lbl:<14}: P_avoid(<S2) = {k_av}/{n_av} ({p_av:.1f}%), {ct['ci_note']} -> {ct['assigned_c']}")
    print("=" * 108)
