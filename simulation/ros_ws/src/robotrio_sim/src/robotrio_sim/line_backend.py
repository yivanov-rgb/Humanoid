"""Gazebo adapter for the otherwise standalone hardware line follower.

All values in this module, including the camera pose, belong to the simulator.
The robot script imports this adapter only when explicitly using --backend sim.
"""
import json
import math
import threading
import time
import uuid

from line_follow import bounded


ACTIONS = ('forward_one_step', 'turn_left', 'turn_right', 'move_left', 'move_right')
HEAD_POSITIONS = ((23, 500), (24, 300))


class SimBackend:
    def __init__(self, args, ros, check):
        self.args, self.ros, self.check = args, ros, check
        self.abort = threading.Event()
        self.token = None
        self.sent_frames = 0
        if (not ros.get_param('/use_sim_time', False)
                or ros.get_param('/robotrio_sim/mode', '') not in
                ('anchored_inspection', 'dynamic_experiment')):
            raise RuntimeError('Simulation markers are missing; refusing action service')
        from robotrio_sim.srv import ActionCommand, ActionCommandRequest
        self.service_type, self.request_type = ActionCommand, ActionCommandRequest
        bounded(lambda: ros.wait_for_service('/robotrio_sim/actions', timeout=8),
                8, check=self._check)
        try:
            for name in ACTIONS:
                info = json.loads(self.rpc('info', name=name).json_result)
                duration = info['duration_seconds']
                if (type(duration) not in (int, float) or not math.isfinite(duration)
                        or duration <= 0 or duration + 1 >= args.action_timeout):
                    raise ValueError('--action-timeout must exceed each action duration + 1 second')
            if getattr(args, 'execute', False) and not getattr(args, 'keep_head', False):
                self.prepare_head()
        except BaseException:
            self.cancel()
            raise

    def _check(self):
        if self.abort.is_set() or self.ros.is_shutdown():
            raise InterruptedError('Simulation motion cancelled')
        self.check()

    def rpc(self, command, name='', token='', timeout=None, check=True):
        request = self.request_type(command=command, action_path=self.args.action_path,
                                    name=name, repeat=1, pause=0.0, token=token)
        response = bounded(
            lambda: self.ros.ServiceProxy('/robotrio_sim/actions', self.service_type)(request),
            self.args.action_timeout if timeout is None else timeout,
            check=self._check if check else None)
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def prepare_head(self):
        """Use only the simulator's established 500/300 pose and verify feedback."""
        from ros_robot_controller.msg import BusServoPosition, SetBusServosPosition
        from ros_robot_controller.srv import GetBusServosPosition
        publisher = self.ros.Publisher('/ros_robot_controller/bus_servo/set_position',
                                       SetBusServosPosition, queue_size=1)
        try:
            deadline = time.monotonic() + 6
            while publisher.get_num_connections() == 0:
                self._check()
                if time.monotonic() >= deadline:
                    raise RuntimeError('Simulation head command subscriber unavailable')
                time.sleep(0.02)
            self._check()
            # No action owns this one-off head command. The simulation stop RPC
            # with an empty token also cancels an active direct bridge command.
            self.token = ''
            self.ros.loginfo('SIM_HEAD_PREPARE pan=500 tilt=300')
            publisher.publish(SetBusServosPosition(duration=0.6, position=[
                BusServoPosition(id=sid, position=pulse) for sid, pulse in HEAD_POSITIONS]))
            read = self.ros.ServiceProxy('/ros_robot_controller/bus_servo/get_position',
                                         GetBusServosPosition)
            started, actual = time.monotonic(), {}
            while time.monotonic() < deadline:
                self._check()
                response = bounded(lambda: read([23, 24]), 2, check=self._check)
                actual = {position.id: position.position for position in response.position}
                if (response.success and time.monotonic() - started >= 0.9
                        and all(abs(actual.get(sid, -10000) - pulse) <= 12
                                for sid, pulse in HEAD_POSITIONS)):
                    return
                time.sleep(0.03)
            raise RuntimeError('Simulation head did not reach 500/300; measured: %s' % actual)
        finally:
            publisher.unregister()

    def play(self, name):
        self._check()
        if name not in ACTIONS:
            raise ValueError('Unsupported line-follow action: ' + name)
        # A fresh token prevents a delayed stop cancelling a subsequent action.
        self.token = uuid.uuid4().hex
        response = self.rpc('play', name=name, token=self.token)
        result = json.loads(response.json_result)
        self.sent_frames += result.get('acknowledged_frames', 0)

    def cancel(self):
        self.abort.set()
        if self.token is not None:
            try:
                self.rpc('stop', token=self.token, timeout=2, check=False)
            except Exception as error:
                self.ros.logerr('Simulation cancellation not confirmed: %s; use sim.sh stop-action',
                               error)
