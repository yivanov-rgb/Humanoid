#!/usr/bin/env python3
"""List, inspect, play or stop simulation ActionGroups through one server.

No action is executed locally. Ctrl+C/timeout requests cancellation using this
client's ownership token. A separate `stop` command stops any server playback.
Successful playback acknowledges commands and simulated durations, not grasp.
"""
import argparse
import json
import math
import queue
import sys
import threading
import time
import uuid

import rospy
from robotrio_sim.srv import ActionCommand, ActionCommandRequest


def bounded(operation, deadline):
    result = queue.Queue(maxsize=1)

    def worker():
        try:
            result.put((True, operation()))
        except Exception as error:
            result.put((False, error))

    threading.Thread(target=worker, daemon=True).start()
    while time.monotonic() < deadline:
        try:
            ok, value = result.get(timeout=min(0.1, max(0.001, deadline - time.monotonic())))
            if not ok:
                raise RuntimeError(str(value))
            return value
        except queue.Empty:
            continue
    raise TimeoutError('Action service exceeded wall-time timeout')


def parser():
    result = argparse.ArgumentParser(description=__doc__)

    def options(target, suppress=False):
        target.add_argument('--action-path', default=argparse.SUPPRESS if suppress else '/actions')
        target.add_argument('--timeout', type=float,
                            default=argparse.SUPPRESS if suppress else 3600.0,
                            help='total service wall-time timeout (seconds)')

    options(result)
    commands = result.add_subparsers(dest='command', required=True)
    for name in ('list', 'info', 'play', 'stop'):
        command = commands.add_parser(name)
        options(command, suppress=True)  # options may precede or follow subcommand
        if name in ('info', 'play'):
            command.add_argument('name')
        if name == 'play':
            command.add_argument('--repeat', type=int, default=1)
            command.add_argument('--pause', type=float, default=0.0,
                                 help='simulated seconds between repetitions')
    return result


def main():
    args = parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise SystemExit('--timeout must be finite and positive')
    if args.command == 'play' and (not 1 <= args.repeat <= 1000
                                  or not math.isfinite(args.pause) or args.pause < 0):
        raise SystemExit('--repeat must be 1..1000; --pause finite and nonnegative')
    token = uuid.uuid4().hex if args.command == 'play' else ''
    request = ActionCommandRequest(command=args.command, action_path=args.action_path,
                                   name=getattr(args, 'name', ''), repeat=getattr(args, 'repeat', 1),
                                   pause=getattr(args, 'pause', 0.0), token=token)
    deadline = time.monotonic() + args.timeout

    def stop_owned():
        if token:
            try:
                response = bounded(lambda: rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(
                    ActionCommandRequest(command='stop', token=token)), time.monotonic() + 5.0)
                print('Cancellation: ' + response.message, file=sys.stderr)
            except Exception as error:
                print('Cancellation could not be confirmed; use stop-action: ' + str(error), file=sys.stderr)

    try:
        bounded(lambda: rospy.init_node('robotrio_sim_action_client', anonymous=True,
                                        disable_signals=True), deadline)
        bounded(lambda: rospy.wait_for_service('/robotrio_sim/actions',
                                               timeout=max(0.01, deadline - time.monotonic())), deadline)
        response = bounded(lambda: rospy.ServiceProxy('/robotrio_sim/actions', ActionCommand)(request), deadline)
        result = {'success': response.success, 'message': response.message,
                  'result': json.loads(response.json_result)}
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if response.success else 1
    except KeyboardInterrupt:
        stop_owned()
        return 130
    except Exception as error:
        stop_owned()
        print(json.dumps({'success': False, 'message': str(error)}, ensure_ascii=False))
        return 1
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown('action client finished')


if __name__ == '__main__':
    raise SystemExit(main())
