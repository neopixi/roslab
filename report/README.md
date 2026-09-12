# Отчёт

Итоговый PDF находится в `../output/pdf/report.pdf`.

Состав каталога:

- `report.tex` — исходник LaTeX в стиле предыдущих отчётов;
- `*.csv` — записи топиков реального прогона ROS 2;
- `generate_plots.py` — построение графиков по CSV;
- `images/` — снимок симуляции и графики для отчёта.

Для пересборки графиков нужен Python с `matplotlib`:

```bash
python3 generate_plots.py
```

PDF собирался XeTeX-совместимым движком Tectonic:

```bash
cd report
tectonic report.tex --outdir ../output/pdf
```

В исходнике используются системные шрифты Times New Roman, Arial и Courier New.
