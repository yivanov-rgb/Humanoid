"""Actual-camera negative/stop tests on a FREE model, never on hardware."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rospy
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
from check_line_follow import Checks, wait_until


class FreeChecks(Checks):
    def start(self, name, *arguments):
        # The preceding route prepared the camera; these tests isolate walking.
        return super().start(name, '--keep-head', *arguments)

    def run(self):
        xml = ET.fromstring(rospy.get_param('/robot_description'))
        self.verify('free_simulation_without_fixture', self.report['mode'] == 'dynamic_experiment'
                    and rospy.get_param('/robotrio_sim/initial_pose_ready', False)
                    and xml.find("joint[@name='inspection_world_mount']") is None)
        rospy.wait_for_service('/robotrio_sim/actions', timeout=20)
        wait_until(lambda: self.images and self.joints, 20)
        start_bus, start_debug = len(self.bus), self.debug
        process, path = self.start('free_observe')
        wait_until(lambda: self.debug >= start_debug + 4 or process.poll() is not None)
        stopped = self.stop_follower()
        log = self.finish(process, path)
        self.verify('observe_without_motion', stopped.success and 'OBSERVE' in log
                    and self.debug >= start_debug + 4 and len(self.bus) == start_bus)

        self.move_line(1.0)
        start_bus = len(self.bus)
        process, path = self.start('free_lost_line', '--execute', '--empty-hands')
        log = self.finish(process, path)
        self.verify('lost_line_no_blind_movement', 'NO_STABLE_LINE' in log
                    and len(self.bus) == start_bus, {'log': str(path)})
        self.move_line(0)

        start_bus = len(self.bus)
        process, path = self.start('free_missing_camera', '--execute', '--empty-hands',
                                    '--image-topic', '/line_check/no_camera', '--frame-timeout', '0.7')
        log = self.finish(process, path)
        self.verify('missing_camera_no_movement', 'CAMERA_STALE' in log and len(self.bus) == start_bus)

        start_bus = len(self.bus)
        process, path = self.start('free_manual_stop', '--execute', '--empty-hands',
                                    '--max-actions', '10', '--settle', '3')
        wait_until(lambda: 'ACTION 1/10:' in path.read_text() or process.poll() is not None)
        import re
        match = re.search(r'ACTION 1/10: (\w+)', path.read_text())
        if match is None:
            raise AssertionError('No actual movement before manual-stop test')
        count = json.loads(self.call('info', match.group(1)).json_result)['frames']
        wait_until(lambda: len(self.bus) >= start_bus + count or process.poll() is not None)
        last = self.bus[-1]
        wait_until(lambda: rospy.Time.now().to_sec() > last[1] + last[2].duration + 0.05)
        stopped = self.stop_follower()
        log = self.finish(process, path)
        self.verify('manual_stop_prevents_second_action', stopped.success
                    and log.count('ACTION ') == 1 and len(self.bus) - start_bus == count)
        current = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('ainex', 'world')
        q = current.pose.orientation
        import math
        tilt = math.degrees(math.acos(max(-1, min(1, 1-2*(q.x*q.x+q.y*q.y)))))
        self.verify('model_still_upright_after_manual_stop', tilt < 25 and current.pose.position.z > 0.17,
                    {'tilt_degrees': tilt, 'z': current.pose.position.z})

        start_bus = len(self.bus)
        process, path = self.start('free_paused_camera', '--execute', '--empty-hands',
                                    '--max-actions', '10', '--frame-timeout', '0.8')
        wait_until(lambda: len(self.bus) > start_bus or process.poll() is not None)
        self.verify('action_started_before_intentional_pause', len(self.bus) > start_bus)
        rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
        report_log = self.finish(process, path, seconds=10)
        self.verify('paused_camera_cancels_current_action', process.returncode != 0
                    and report_log.count('ACTION ') == 1 and 'CAMERA_STALE' in report_log)
        self.report['intentional_physics_pause_at_end'] = True
        self.report['ok'] = True


def main():
    rospy.init_node('free_line_safety', anonymous=True)
    checks = FreeChecks()
    try:
        checks.run()
    except Exception as error:
        checks.report['error'] = str(error)
        print('FAILED: '+str(error), flush=True)
    finally:
        try:
            checks.cleanup()
        except Exception as error:
            checks.report.update(ok=False, cleanup_error=str(error))
        Path('/workspace/simulation/artifacts/line_free_safety.json').write_text(
            json.dumps(checks.report, indent=2)+'\n')
        rospy.signal_shutdown('free safety checks finished')
    return 0 if checks.report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
