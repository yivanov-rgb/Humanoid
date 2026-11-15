#!/usr/bin/env python3
"""AiNex / ROS 1: второй этап — коррекция в приседе и захват ID 0.

Запускается после CROUCHED первого этапа tag_grab_approach.py.
Робот уже в приседе, камера остаётся в 500/330, груз перед роботом.
Выбранную первым этапом руку передайте через --hand left или --hand right.
Этот скрипт не выполняет stand, повторный присед или шаги вперёд/назад.

По умолчанию только наблюдение, MotionManager не создаётся.
При --execute: свежие кадры -> _step_to_left/_step_to_right -> новые кадры ->
clamp_left_from_crouch или clamp_right_from_crouch (захват и подъём).
Рука фиксирована. Пороги X независимы для каждой руки, пока 310..330
вокруг центра кадра 320. Y только выводится; пороги нужно подобрать на роботе.
Нужен один однозначный ID 0 tag36h11 в изображении 640x480.

Примеры внутри аппаратного контейнера ainex:
  python3 tag_grab_crouch.py --hand left
  python3 tag_grab_crouch.py --execute --hand left
  python3 tag_grab_crouch.py --execute --hand right --right-x-min 400 --right-x-max 450

Все три нужные ActionGroups должны находиться в --action-path.
Потеря цели, таймаут, лимит шагов и Ctrl+C прекращают дальнейшие команды
без автоматического вставания или захвата.
"""
import argparse
import math
from pathlib import Path
import time

from tag_grab_approach import Runner as ApproachRunner


SCRIPT_VERSION = 'tag-grab-crouch-2026-11-15'


def bounds_for(hand, args):
    if hand not in ('left', 'right'):
        raise ValueError('Неизвестная рука: ' + str(hand))
    return (getattr(args, hand + '_x_min'), getattr(args, hand + '_x_max'))


def decide(x, bounds):
    # Сохраняем проверенное направление коррекции из первого этапа.
    x_min, x_max = bounds
    if x > x_max:
        return '_step_to_right'
    if x < x_min:
        return '_step_to_left'
    return 'READY'


class Runner(ApproachRunner):
    # Общие подписки, проверка камеры, свежести кадров и исполнение ActionGroups
    # остаются такими же, как на первом этапе. Подготовка стойки не вызывается.
    def motion_actions(self):
        return {'_step_to_left', '_step_to_right',
                'clamp_' + self.args.hand + '_from_crouch'}

    def align(self):
        self.fresh_frames()
        bounds = bounds_for(self.args.hand, self.args)
        self.ros.loginfo('CROUCH_ALIGN HAND=%s X=[%.1f,%.1f]; Y только наблюдение',
                         self.args.hand.upper(), *bounds)
        serial, previous, streak, moves, previous_received = 0, None, 0, 0, None
        start = last_seen = time.monotonic()
        while not self.ros.is_shutdown():
            now = time.monotonic()
            if now - start > self.args.timeout:
                raise RuntimeError('Истекло время коррекции в приседе')
            if now - last_seen > self.args.lost_timeout:
                raise RuntimeError('Потеряна цель ID 0; коррекция и захват прекращены')
            obs = self.frames.wait_new(serial)
            if obs is None:
                continue
            serial = obs.serial
            target = self.fresh_target(obs)
            if target is None:
                previous, streak, previous_received = None, 0, None
                self.ros.loginfo_throttle(1, 'WAIT_TARGET: нужен один свежий ID 0')
                continue
            last_seen = time.monotonic()
            decision = decide(target.x, bounds)
            continuous = (previous_received is not None
                          and obs.received - previous_received <= self.args.max_age)
            streak = streak + 1 if decision == previous and continuous else 1
            previous, previous_received = decision, obs.received
            self.ros.loginfo_throttle(
                0.5, 'ID=0 HAND=%s stamp=%d x=%.1f y=%.1f -> %s [%d/%d]',
                self.args.hand.upper(), obs.stamp, target.x, target.y, decision,
                streak, self.args.stable_frames)
            if not self.args.execute or streak < self.args.stable_frames:
                continue
            if decision == 'READY':
                self.ros.loginfo('ALIGNED: ID=0 HAND=%s x=%.1f y=%.1f',
                                 self.args.hand.upper(), target.x, target.y)
                return
            if moves >= self.args.max_moves:
                raise RuntimeError('Достигнут лимит подшагов в приседе')
            self.action(decision)
            moves += 1
            last_seen = time.monotonic()
            previous, streak, previous_received = None, 0, None
        self.check_running()

    def run(self):
        if not self.args.execute:
            self.ros.loginfo('OBSERVE: измерения в приседе; камера и приводы не перемещаются')
        else:
            self.ros.loginfo('START_CROUCHED: HAND=%s; ожидается камера 500/330 после первого этапа',
                             self.args.hand.upper())
        self.align()
        if self.args.execute:
            self.action('clamp_' + self.args.hand + '_from_crouch')
            self.ros.loginfo('GRASP_SEQUENCE_DONE: HAND=%s; захват и подъём воспроизведены',
                             self.args.hand.upper())


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--version', action='version', version=SCRIPT_VERSION)
    p.add_argument('--execute', action='store_true', help='коррекция из приседа, затем захват и подъём')
    p.add_argument('--hand', choices=('left', 'right'), required=True,
                   help='рука из CROUCHED первого этапа; повторно не выбирается')
    p.add_argument('--topic', default='/tag_detections')
    p.add_argument('--camera-info', default='/camera/camera_info')
    p.add_argument('--family-param', default='/apriltag_ros_continuous_node/tag_family')
    p.add_argument('--action-path', default='/home/ubuntu/software/ainex_controller/ActionGroups')
    p.add_argument('--left-x-min', type=float, default=230)
    p.add_argument('--left-x-max', type=float, default=280)
    p.add_argument('--right-x-min', type=float, default=470)
    p.add_argument('--right-x-max', type=float, default=520)
    p.add_argument('--stable-frames', type=int, default=3)
    p.add_argument('--max-moves', type=int, default=10)
    p.add_argument('--max-age', type=float, default=1.0)
    p.add_argument('--settle', type=float, default=0.5)
    p.add_argument('--lost-timeout', type=float, default=10)
    p.add_argument('--timeout', type=float, default=180)
    a = p.parse_args(argv)
    for side in ('left', 'right'):
        x_min, x_max = bounds_for(side, a)
        if not 0 <= x_min <= x_max < 640:
            p.error('Пороги X каждой руки должны быть внутри кадра 640x480')
    if (not all(math.isfinite(v) for v in (a.max_age, a.settle, a.lost_timeout, a.timeout))
            or a.max_age <= 0 or a.settle < 0 or a.lost_timeout <= 0 or a.timeout <= 0
            or a.stable_frames < 1 or a.max_moves < 1):
        p.error('Неверный таймаут или лимит')
    a.action_path = str(Path(a.action_path).resolve())
    return a


def main():
    args = arguments()
    import rospy
    rospy.init_node('robotrio_tag_grab_crouch')
    rospy.loginfo('SCRIPT_VERSION=%s FILE=%s', SCRIPT_VERSION, str(Path(__file__).resolve()))
    try:
        Runner(args, rospy).run()
        return 0
    except (KeyboardInterrupt, InterruptedError, rospy.ROSInterruptException):
        return 130
    except Exception as error:
        rospy.logerr('STOP: %s', error)
        return 1
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown('tag_grab_crouch finished')


if __name__ == '__main__':
    raise SystemExit(main())
