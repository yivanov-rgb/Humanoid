# Локальный стенд AiNex: ROS Noetic и Gazebo 11

[Сценарии физического робота](../robot/README.md) и
[справочник ActionGroups](ACTION_GROUPS_RU.md) описаны отдельно.
Все команды ниже выполняются из корня Humanoid на локальном компьютере.
Стенд `humanoid-ainex-sim` изолирован от аппаратного контейнера `ainex`:
сеть отключена, устройства робота не подключаются, проект монтируется только
для чтения, результаты записываются в `simulation/artifacts/`.

## Подготовка

Среда контейнера: Ubuntu 20.04, ROS 1 Noetic, Gazebo Classic 11,
Python 3.8 в `/opt/ainex-venv` с ROS/OpenCV из системных пакетов.
ARM-библиотеки процедурной походки из штатного AiNex на x86_64 не запускаются;
для дискретных `.d6a` используется адаптер pulse→rad.
Модель HiWonder закреплена в `vendor.lock`, локальная копия — `simulation/vendor`.

```bash
python3 -m venv --without-pip .venv
bash simulation/sim.sh fetch
bash simulation/sim.sh build
```

Нужен доступ к локальному Docker daemon. Первая сборка скачивает образ и
зависимости; скрипты не изменяют Docker socket, группы или пакеты хоста.
Пересборка нужна при изменении зависимостей, Dockerfile, CMake или vendor-модели.
Python, launch и мир подключены из проекта и доступны при следующем запуске.
Хостовый `.venv` используется для офлайн-проверок и не заменяет среду ROS.

## Запуск, режим модели и остановка

```bash
bash simulation/sim.sh gui --free
bash simulation/sim.sh status
```

`up` запускает headless Gazebo с Xvfb; `gui` (или `up --gui`) — окно в локальном
X11/XWayland-сеансе. Для GUI нужны `DISPLAY`, доступный X-сокет и утилиты
`xauth`, `xdpyinfo`, `timeout`. Используется приватная копия Xauth-cookie;
`xhost +`, privileged и host networking не нужны. SSH X-forwarding не поддерживается.
Перед сменой параметров уже запущенного стенда выполните `stop`.

По умолчанию модель свободна. Она создаётся в исходной позе `stand.d6a`,
с опущенными руками и подошвами на полу. Перед ходьбой дождитесь
`initial_pose_ready: true` в `status`; при неудаче подготовки физика ставится на паузу.
Приводы используют EffortJointInterface и PD-регуляторы; это настройки симуляции.
Режим `gui --anchored` закрепляет корпус на высоте 0,40 м и применяется
для диагностики интерфейсов. Ходьбу по линии проверяйте только с `--free`.

```bash
bash simulation/sim.sh logs
bash simulation/sim.sh shell
# После выхода из shell и завершения опыта:
bash simulation/sim.sh stop
```

`stop` закрывает GUI/ROS-процессы, останавливает и удаляет контейнер стенда,
удаляет его приватную копию Xauth. Закрытия окна Gazebo недостаточно.
Останавливайте только стенд своего опыта; процессы другого пользователя не затрагивайте.

## ActionGroups

Девять действий линии подготовлены в `simulation/actiongroups`.
Для другого каталога передайте непосредственно папку с файлами `.d6a`:

```bash
bash simulation/sim.sh gui --free --actions-dir /absolute/path/to/ActionGroups
```

Альтернатива — переменная `ROBOTRIO_ACTIONS_DIR`. Каталог монтируется только
для чтения и меняется перезапуском контейнера. По умолчанию калибровка берётся
из `servo_controller.yaml` HiWonder; свой файл можно подключить через
`ROBOTRIO_SERVO_CONFIG=/absolute/path/to/servo_controller.yaml` при запуске.
Преобразование: `angle = (pulse - init) / coefficient`,
`coefficient = ±(180 / 3.1415926 / 240 * 1000)`, знак задаёт порядок min/max.

```bash
bash simulation/sim.sh list-actions
bash simulation/sim.sh action-info stand
bash simulation/sim.sh action stand
bash simulation/sim.sh action forward_one_step
bash simulation/sim.sh action forward --repeat 3
bash simulation/sim.sh action turn_right --repeat 8 --pause 0.1
bash simulation/sim.sh stop-action
```

Имя передаётся без `.d6a`. На старте GUI группы автоматически не проигрываются.
`action-info` показывает кадры, длительность, сервоприводы и хеш. Проигрыватель
проверяет формат, pulse, карту ID и пределы URDF, не подрезая значения.
Одновременно работает один проигрыватель. Пауза `/clock` прекращает действие
по watchdog. `stop-action` останавливает команды, сохраняя текущие цели суставов;
для остановки физики используйте Pause, для всего стенда — `stop`.
Перед Reset World/Model/Time остановите действие; сброс позы не заменяет остановку.
Группы задают все сервоприводы своих кадров: удержание руки из `tag_grab.py`
в ручном проигрывателе не применяется автоматически.

## Камера и следование линии

Сцена содержит ровный пол и чёрную линию шириной 30 мм: дугу длиной 2 м,
радиусом 6 м, с поворотом 19,1°. Кубов и AprilTag-меток в сцене нет.
Overlay крепит камеру к `head_tilt_link`, поэтому движение головы меняет изображение.
Привязка камеры и виртуальная поза 500/300 относятся только к симуляции.
Аппаратный `line_follow.py` голову не двигает; адаптер стенда готовит её перед первым действием.

После подготовки стойки, в отдельных терминалах:

```bash
bash simulation/sim.sh line-view
bash simulation/sim.sh line-follow --image-orientation standard --max-seconds 30
# После проверки ориентации изображения и решений, не более трёх действий:
bash simulation/sim.sh line-follow --execute --empty-hands --image-orientation standard --max-actions 3
```

Для зеркального изображения используйте `mirrored`.
Смещение параллельной линии вызывает приставной шаг, наклон — поворот,
центр — `forward_one_step`. Для решения нужны свежие согласованные кадры;
при потере линии или камеры слепого поиска нет. Дополнительный просмотр
из `shell`: `rosrun rqt_image_view rqt_image_view /camera/image_raw`;
состояния суставов: `rostopic echo -n 1 /joint_states`.
Оставляйте один источник движения. Завершение:

```bash
bash simulation/sim.sh line-stop
bash simulation/sim.sh stop
```

Для отдельного испытания свободного следования с измерением отклонения:

```bash
bash -eu <<'BASH'
trap 'bash simulation/sim.sh stop' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash simulation/sim.sh gui --free
docker exec humanoid-ainex-sim /opt/ainex-entrypoint.sh timeout 80s \
  python /workspace/tests/check_initial_pose.py
docker exec humanoid-ainex-sim /opt/ainex-entrypoint.sh timeout 140s \
  python /workspace/tests/check_line_free.py --max-actions 30 --max-seconds 110 --view
BASH
```

Независимая проверка стойки наблюдает модель минимум 5 секунд симуляции.
Тест маршрута измеряет расстояние до дуги: критерий центрирования — 5 см;
при удалении более 8 см, наклоне более 25° или снижении корпуса более 4 см
прекращает движение и ставит физику на паузу. Результаты —
`artifacts/line_free_probe.json`, `line_free.log`, `line_free_{raw,debug}.png`.

## Выбор проверок и текущее состояние

Для документации достаточно ссылок, команд и diff. Для изменения алгоритма
выберите соответствующие офлайн-тесты; при изменении ROS-совместимости —
целевую среду, при изменении модели/динамики — соответствующий сценарий Gazebo.
Полный набор `sim.sh test` на хосте и `sim.sh test-container` в Python 3.8,
а также `sim.sh smoke` доступны по необходимости. Smoke запускает gzserver/Xvfb
и проверяет модель, время, контроллеры, состояния суставов, камеру и отклик головы;
он не подтверждает свободную ходьбу и захват.

Последний описанный свободный прогон на дуге, 18 октября 2026, не пройден:
завершены 6 шагов вперёд и 2 приставных влево; после начала девятого действия
расстояние от линии достигло 0,08006 м и сработала остановка.
Продвижение вдоль дуги — 0,172 м; падения не было. Исторический результат
75 см относится к прежнему контроллеру на прямой. Более поздние изменения
не подтверждают исправление ухода без нового испытания.

Прибытие к метке Б, полный маршрут, перенос кубов и аппаратный алгоритм линии
не подтверждены. Контакты и динамика модели не откалиброваны по роботу;
рабочие pulse и аппаратную калибровку ради URDF не изменяйте.
История прежних экспериментов сохранена в Git: `simulation/TEST_RESULTS_RU.md`.
