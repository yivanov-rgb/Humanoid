# Скрипты физического AiNex

Для ходьбы по линии нужен **один файл — `line_follow.py`**. Скопируйте его в `/home/ubuntu/robotrio/` внутри ROS Noetic-контейнера `ainex`. Штатные драйверы HiWonder, MotionManager, камера и ActionGroups должны быть установлены на роботе.

[Инструкция по загрузке и запуску](LINE_TEST_RU.md).

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
cd /home/ubuntu/robotrio

# Наблюдение без движения
python3 line_follow.py --max-seconds 30
```

Движения вызываются через штатный `MotionManager.run_action`. Используются пять испытанных вами ActionGroups; диапазоны сервоприводов дополнительно не проверяются. Первый запуск можно ограничить одним действием:

```bash
python3 line_follow.py --execute --empty-hands \
  --image-orientation standard --max-actions 1 --max-seconds 30
```

`standard` используйте только после проверки ориентации; для зеркального изображения нужен `mirrored`. Голова остаётся в текущем положении. Скрипт экспериментальный, на реальном роботе ещё не проверен.

`tag_grab.py` — отдельный проверенный сценарий захвата двух кубов. Его параметры, порядок захвата и размещения не изменены. Корневые `line_follow.py` и `tag_grab.py` — ссылки на исходники этой папки.
